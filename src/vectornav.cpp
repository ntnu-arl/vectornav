// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "vectornav/vectornav.hpp"

namespace vectornav
{
VectorNav::VectorNav(ros::NodeHandle & pnh)
{
  // Get parameters
  ReadParams(pnh);
  VerifyParams();

  // Setup logging sinks
  logger_console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  logger_console_sink_->set_level(log_level_);
  logger_file_sink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
    log_directory_ + ros::this_node::getName(), true);
  logger_file_sink_->set_level(log_level_);

  // Create logger
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(logger_console_sink_);
  sinks.push_back(logger_file_sink_);
  logger_ = std::make_shared<spdlog::logger>("logger", sinks.begin(), sinks.end());
  logger_->set_level(log_level_);
  logger_->flush_on(log_level_);

  // Setup Publishers
  logger_->debug("Set up publishers start");
  pub_imu_ = pnh.advertise<sensor_msgs::Imu>("imu", 1000, false);
  pub_uncomp_imu_ = pnh.advertise<sensor_msgs::Imu>("uncomp_imu", 1000, false);
  pub_mag_ = pnh.advertise<sensor_msgs::MagneticField>("magetic_field", 1000, false);
  pub_uncomp_mag_ = pnh.advertise<sensor_msgs::MagneticField>("uncomp_magetic_field", 1000, false);
  pub_temp_ = pnh.advertise<sensor_msgs::Temperature>("temperature", 1000, false);
  pub_pres_ = pnh.advertise<sensor_msgs::FluidPressure>("pressure", 1000, false);
  logger_->debug("Set up publishers end");
}

VectorNav::~VectorNav() {}

void VectorNav::ReadParams(ros::NodeHandle & pnh)
{
  // Read parameters
  pnh.param<std::string>("port", port_, "/dev/ttyUSB0");
  int i_param;
  pnh.param<int>("baud_rate", i_param, 115200);
  baud_rate_ = static_cast<uint32_t>(i_param);
  pnh.param<int>("async_mode", i_param, 2);
  async_mode_ = static_cast<AsyncMode>(i_param);
  pnh.param<int>("async_rate_divisor", i_param, 4);
  async_rate_divisor_ = static_cast<uint16_t>(i_param);
  pnh.param<bool>("is_triggered", is_triggered_, false);
  pnh.param<int>("sync_in_skip_factor", i_param, 0);
  sync_in_skip_factor_ = static_cast<uint16_t>(i_param);
  pnh.param<bool>("is_triggering", is_triggering_, true);
  pnh.param<int>("sync_out_skip_factor", i_param, 39);
  sync_out_skip_factor_ = static_cast<uint16_t>(i_param);
  pnh.param<int>("sync_out_pulse_width", i_param, 1.0e+9);
  sync_out_pulse_width_ = static_cast<uint32_t>(i_param);
  pnh.param<bool>("publish_uncomp_imu", publish_uncomp_imu_, false);
  pnh.param<bool>("publish_uncomp_mag", publish_uncomp_mag_, false);
  pnh.param<std::string>("frame_id", frame_id_, "imu_link");
  pnh.param<float>("temp_variance", temp_variance_, 0.1);
  pnh.param<float>("pres_variance", pres_variance_, 0.1);
  pnh.param<std::string>("log_directory", log_directory_, "/tmp/vectornav/");
  pnh.param<int>("log_level", i_param, 0);
  log_level_ = static_cast<spdlog::level::level_enum>(i_param);
}

void VectorNav::VerifyParams()
{
  assert(
    std::find(
      sensor_->SupportedBaudRates().begin(), sensor_->SupportedBaudRates().end(), baud_rate_) !=
    sensor_->SupportedBaudRates().end());
  assert(async_mode_ >= AsyncMode::ASYNCMODE_NONE && async_mode_ <= AsyncMode::ASYNCMODE_BOTH);
  assert(frame_id_.size() > 0);
  assert(temp_variance_ > 0);
  assert(pres_variance_ > 0);
}

void VectorNav::SetupSensor()
{
  // Should try to set the baud rate instead of blindly trusting the baud rate to be available
  // Connect to sensor
  logger_->info("Connecting to sensor at {} with baud rate {}", port_, baud_rate_);
  sensor_.connect(port_, baud_rate_);

  if (!sensor_.verifySensorConnectivity()) {
    logger_->critical("Sensor connectivity check failed");
    return;
  }
  logger_->info("Sensor connectivity check passed");

  PrintSensorInfo();

  // TODO: Feature: Factory data reset/ reset the sensor before any configuration change is made. This
  // way only the configuration changes available thorough the driver will be avaialble. To be
  // checked whether this is feasible and sensible.

  // Stop any sort of data coming from the sensor before writing a config
  logger_->debug("Turning off data streaming");
  sensor_.writeAsyncDataOutputType(VNOFF);

  // Setup using the binary output registers. This is significantly faster than using ASCII output
  logger_->debug("Setting up binary output registers");
  vn::sensors::BinaryOutputRegister bor(
    async_mode_, async_rate_divisor_,
    COMMONGROUP_TIMESTARTUP | COMMONGROUP_TIMESYNCIN | COMMONGROUP_QUATERNION |
      COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES |
      (is_triggered_ ? COMMONGROUP_TIMESYNCIN | COMMONGROUP_SYNCINCNT : COMMONGROUP_NONE) |
      (publish_uncomp_imu_ ? COMMONGROUP_IMU : COMMONGROUP_NONE),
    TIMEGROUP_NONE, publish_uncomp_mag_ ? IMUGROUP_UNCOMPMAG : IMUGROUP_NONE, GPSGROUP_NONE,
    ATTITUDEGROUP_NONE, INSGROUP_NONE, GPSGROUP_NONE);

  // Note: Time since startup and time since syncin are measured in nanoseconds

  vn::sensors::BinaryOutputRegister bor_none(
    ASYNCMODE_NONE, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE,
    ATTITUDEGROUP_NONE, INSGROUP_NONE, GPSGROUP_NONE);

  // Use only one binary output register to minimize data usage. The other registers are
  // explicitly disabled so that they dont get used due to a configuration already on the sensor
  sensor_.writeBinaryOutput1(bor);
  sensor_.writeBinaryOutput2(bor_none);
  sensor_.writeBinaryOutput3(bor_none);

  // Setup synchonization
  logger_->debug("Setting up synchronization");
  sensor_.writeSynchronizationControl(
    is_triggered_ ? SYNCINMODE_IMU : SYNCINMODE_COUNT, SYNCINEDGE_RISING, sync_in_skip_factor_,
    is_triggering_ ? SYNCOUTMODE_ITEMSTART : SYNCOUTMODE_NONE, SYNCOUTPOLARITY_POSITIVE,
    sync_out_skip_factor_, sync_out_pulse_width_);
}

void VectorNav::StopSensor()
{
  // Might need sleeps here to ensure the connection is closed properly - Need to check
  logger_->info("Disconnecting from sensor");
  sensor_.unregisterAsyncPacketReceivedHandler();
  sensor_.disconnect();
}

void VectorNav::SetupAsyncMessageCallback(vn::sensors::VnSensor::AsyncPacketReceivedHandler handler)
{
  // Setup callback
  logger_->debug("Setting up async message callback");
  sensor_.registerAsyncPacketReceivedHandler(this, handler);
}

void VectorNav::BinaryAsyncMessageCallback(Packet & p, size_t index)
{
  const ros::Time arrival_stamp = ros::Time::now();
  logger_->trace("Received async message at timestamp: {}", arrival_stamp.toSec());

  logger_->trace("Parsing binary async message");
  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);

  // Get corrected timestamp
  ros::Time corrected_stamp;

  // The system time since startup measured in nano seconds. The time since startup is based upon the internal
  // TXCO oscillator for the MCU. The accuracy of the internal TXCO is +/- 20ppm (-40C to 85C).
  uint64_t startup_time = cd.timeStartup();

  // The time since the last SyncIn trigger event expressed in nano seconds.
  uint64_t sync_in_time = cd.timeSyncIn();

  CorrectTimestamp(arrival_stamp, startup_time, sync_in_time, corrected_stamp);

  logger_->trace("Publishing parsed data");
  // IMU
  if (pub_imu_.getNumSubscribers()) {
    sensor_msgs::Imu msg;
    PopulateImuMsg(msg, cd, corrected_stamp, false);
    pub_imu_.publish(msg);
  }

  // Uncompensated IMU
  if (pub_uncomp_imu_.getNumSubscribers() && publish_uncomp_imu_) {
    sensor_msgs::Imu msg;
    PopulateImuMsg(msg, cd, corrected_stamp, true);
    pub_uncomp_imu_.publish(msg);
  }

  // Magnetic Field
  if (pub_mag_.getNumSubscribers()) {
    sensor_msgs::MagneticField msg;
    PopulateMagMsg(msg, cd, corrected_stamp, false);
    pub_mag_.publish(msg);
  }

  // Uncompensated Magnetic Field
  if (pub_uncomp_mag_.getNumSubscribers() && publish_uncomp_mag_) {
    sensor_msgs::MagneticField msg;
    PopulateMagMsg(msg, cd, corrected_stamp, true);
    pub_uncomp_mag_.publish(msg);
  }

  // Temperature
  if (pub_temp_.getNumSubscribers()) {
    sensor_msgs::Temperature msg;
    PopulateTempMsg(msg, cd, corrected_stamp);
    pub_temp_.publish(msg);
  }

  // Pressure
  if (pub_pres_.getNumSubscribers()) {
    sensor_msgs::FluidPressure msg;
    PopulatePresMsg(msg, cd, corrected_stamp);
    pub_pres_.publish(msg);
  }
}

void VectorNav::CorrectTimestamp(
  const ros::Time & arrival_stamp, uint64_t startup_time, uint64_t sync_in_time,
  ros::Time & corrected_stamp)
{
  // TODO: Implement this
  // Possible timestamp corrections (which method to use could be a parameter):
  // 1. Do nothing - Host timestamping (inaccurate due to transmission and processing delays)
  // 2. Get the ros time and time since startup from the sensor and use that as reference
  // 3. Synchronize with an external micro controller that is triggering the sensor and use the sync in time and the known triggers to correct the time
  // 4. Assume that the sensor has an almost constant rate and correct the timestamp based on the timestamp that is expected to arrive next - This is implemented here: https://github.com/dawonn/vectornav/blob/6824e8b668b889a76214636cbc00a21c0b208a29/src/main.cpp#L725-L749
  corrected_stamp = arrival_stamp;
  logger_->trace(
    "Arrival timestamp: {}\nCorrected timestamp: {}", arrival_stamp.toSec(),
    corrected_stamp.toSec());
}

void VectorNav::PrintSensorInfo()
{
  logger_->info(
    "Sensor information:\n\tModel Number: {}\n\tSerial Number: {}\n\tFirmware Version: "
    "{}\n\tHardware Revision: {}",
    sensor_.readModelNumber(), sensor_.readSerialNumber(), sensor_.readFirmwareVersion(),
    sensor_.readHardwareRevision());
}

void VectorNav::PopulateImuMsg(
  sensor_msgs::Imu & msg, vn::sensors::CompositeData & cd, const ros::Time & stamp, bool uncomp)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;

  vn::math::vec4f q = cd.quaternion();
  vn::math::vec3f acc, omega;  // m/s^2, rad/s
  if (uncomp) {
    // These measurements correspond to the calibrated angular rate and acceleration measurements
    // straight from the IMU. The measurements have not been corrected for bias offset by the
    // onboard Kalman filter.
    acc = cd.accelerationUncompensated();
    omega = cd.angularRateUncompensated();
    logger_->trace("Populating uncompensated IMU message");
  } else {
    // This measurement is compensated by the static calibration (individual factory calibration
    // stored in flash), the user compensation, and the dynamic bias compensation from the onboard
    // INS Kalman filter.
    acc = cd.acceleration();
    omega = cd.angularRate();
    logger_->trace("Populating IMU message");
  }

  // Allow for transformation to different frame
  msg.orientation.x = q[0];
  msg.orientation.y = q[1];
  msg.orientation.z = q[2];
  msg.orientation.w = q[3];

  msg.angular_velocity.x = omega[0];
  msg.angular_velocity.y = omega[1];
  msg.angular_velocity.z = omega[2];

  msg.linear_acceleration.x = acc[0];
  msg.linear_acceleration.y = acc[1];
  msg.linear_acceleration.z = acc[2];

  // Add covariance from config
}

void VectorNav::PopulateMagMsg(
  sensor_msgs::MagneticField & msg, vn::sensors::CompositeData & cd, const ros::Time & stamp,
  bool uncomp)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  vn::math::vec3f mag;  // gauss
  if (uncomp) {
    //  This measurement is compensated by the static calibration (individual factory calibration
    //  stored in flash), and the user compensation, however it is not compensated by the onboard
    //  Hard/Soft Iron estimator.
    mag = cd.magneticUncompensated();
    logger_->trace("Populating uncompensated Magnetic Field message");
  } else {
    // This measurement has been corrected for hard/soft iron corrections (if enabled).
    mag = cd.magnetic();
    logger_->trace("Populating Magnetic Field message");
  }
  // Add covariance from config. is it possible to get this from sensor?
}

void VectorNav::PopulateTempMsg(
  sensor_msgs::Temperature & msg, vn::sensors::CompositeData & cd, const ros::Time & stamp)
{
  logger_->trace("Populating Temperature message");
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.temperature = cd.temperature();  // Celsius
  msg.variance = temp_variance_;
}

void VectorNav::PopulatePresMsg(
  sensor_msgs::FluidPressure & msg, vn::sensors::CompositeData & cd, const ros::Time & stamp)
{
  logger_->trace("Populating Pressure message");
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.fluid_pressure = cd.pressure();  // kPa
  msg.variance = pres_variance_;
}
}  // namespace vectornav