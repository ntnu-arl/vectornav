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
    log_directory_ + ros::this_node::getName() + ".log", true);
  logger_file_sink_->set_level(log_level_);

  // Create logger
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(logger_console_sink_);
  sinks.push_back(logger_file_sink_);
  logger_ = std::make_shared<spdlog::logger>("logger", sinks.begin(), sinks.end());
  logger_->set_level(log_level_);
  logger_->flush_on(log_level_);

  // Setup Publishers
  logger_->debug("Setting up publishers");
  pub_imu_ = pnh.advertise<sensor_msgs::Imu>("imu", 1000, false);
  pub_uncomp_imu_ = pnh.advertise<sensor_msgs::Imu>("uncomp_imu", 1000, false);
  pub_mag_ = pnh.advertise<sensor_msgs::MagneticField>("magetic_field", 1000, false);
  pub_uncomp_mag_ = pnh.advertise<sensor_msgs::MagneticField>("uncomp_magetic_field", 1000, false);
  pub_temp_ = pnh.advertise<sensor_msgs::Temperature>("temperature", 1000, false);
  pub_pres_ = pnh.advertise<sensor_msgs::FluidPressure>("pressure", 1000, false);
  pub_sync_out_stamp_ = pnh.advertise<std_msgs::Header>("sync_out_stamp", 1000, false);

  // Setup Services
  logger_->debug("Setting up services");
  srv_reset_ = pnh.advertiseService("reset", &VectorNav::ResetServiceCallback, this);

  receiving_data_ = false;
}

VectorNav::~VectorNav() {}

void VectorNav::ReadParams(ros::NodeHandle & pnh)
{
  // Read parameters
  int i_param;
  pnh.param<int>("sensor_family", i_param, 0);
  sensor_family_ = static_cast<vn::sensors::VnSensor::Family>(i_param);
  pnh.param<std::string>("port", port_, "/dev/ttyUSB0");
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
  pnh.param<bool>("publish_sync_out_count_on_change", publish_sync_out_stamp_on_change_, false);
  sync_out_pulse_width_ = static_cast<uint32_t>(i_param);
  pnh.param<bool>("publish_uncomp_imu", publish_uncomp_imu_, false);
  pnh.param<bool>("publish_uncomp_mag", publish_uncomp_mag_, false);
  pnh.param<std::string>("frame_id", frame_id_, "imu_link");
  pnh.param<float>("temp_variance", temp_variance_, 0.1);
  pnh.param<float>("pres_variance", pres_variance_, 0.1);
  pnh.param<std::string>("log_directory", log_directory_, "/tmp/vectornav/");
  pnh.param<int>("log_level", i_param, 0);
  log_level_ = static_cast<spdlog::level::level_enum>(i_param);
  XmlRpc::XmlRpcValue temp_rpc;
  pnh.getParam("linear_accel_covariance", temp_rpc);
  SetCovarianceMatrix(temp_rpc, linear_accel_covariance_);
  pnh.getParam("angular_vel_covariance", temp_rpc);
  SetCovarianceMatrix(temp_rpc, angular_vel_covariance_);
  pnh.getParam("orientation_covariance", temp_rpc);
  SetCovarianceMatrix(temp_rpc, orientation_covariance_);
  pnh.getParam("magnetic_field_covariance", temp_rpc);
  SetCovarianceMatrix(temp_rpc, mag_covariance_);
  pnh.getParam("mag_ref", temp_rpc);
  SetArray(temp_rpc, mag_ref_);
  pnh.getParam("gravity_ref", temp_rpc);
  SetArray(temp_rpc, gravity_ref_);
  pnh.param<bool>("write_to_flash", write_to_flash_, false);
  pnh.param<bool>("factory_reset_before_start", factory_reset_before_start_, false);
}

void VectorNav::SetCovarianceMatrix(
  const XmlRpc::XmlRpcValue & covariance_list, boost::array<double, 9ul> & covariance_matrix)
{
  assert(covariance_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  assert(covariance_list.size() == 9);
  for (int i = 0; i < covariance_list.size(); i++) {
    // Use string stream here to avoid issues with parsing with decimal points
    std::ostringstream ostr;
    ostr << covariance_list[i];
    std::istringstream istr(ostr.str());
    istr >> covariance_matrix[i];
  }
}

void VectorNav::SetArray(const XmlRpc::XmlRpcValue & values, std::array<float, 3> & arr)
{
  assert(values.getType() == XmlRpc::XmlRpcValue::TypeArray);
  assert(values.size() == 3);
  for (int i = 0; i < values.size(); i++) {
    // Use string stream here to avoid issues with parsing with decimal points
    std::ostringstream ostr;
    ostr << values[i];
    std::istringstream istr(ostr.str());
    istr >> arr[i];
  }
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
  for (size_t i = 0; i < 9; i++) {
    assert(linear_accel_covariance_[i] >= 0);
    assert(angular_vel_covariance_[i] >= 0);
    assert(orientation_covariance_[i] >= 0);
    assert(mag_covariance_[i] >= 0);
  }
}

void VectorNav::ConnectSensor()
{
  // Attempt to connect to the sensor using the specified port and the supported baud rates.
  // This is because the sensor might be at a baud rate different from the one specified in the parameter file.
  std::vector<uint32_t> supported_baud_rates = sensor_.supportedBaudrates();
  // Required baud rate is inserted first to be the first try and speed up initialization
  supported_baud_rates.insert(supported_baud_rates.begin(), baud_rate_);
  sensor_.setResponseTimeoutMs(1000);  // Wait for up to 1s for a response
  sensor_.setRetransmitDelayMs(50);    // Retransmit every 50ms

  // Iterate through the supported baud rates and try to connect to the sensor.
  // Once connected, try to change to the desired baud rate.
  for (const uint32_t & br : supported_baud_rates) {
    // The VN100 family does not support 128000 baud rate
    if (br == 128000 && sensor_family_ == vn::sensors::VnSensor::Family::VnSensor_Family_Vn100)
      continue;

    logger_->debug("Trying baud rate {}", br);
    try {
      sensor_.connect(port_, br);
      sensor_.changeBaudRate(baud_rate_);
      logger_->info("Connected to VectorNav sensor at {} with baud rate {}", port_, br);
      if (baud_rate_ != br) {
        logger_->info("Baud rate changed from {} to {}", br, baud_rate_);
      }
      break;
    } catch (const vn::permission_denied & e) {
      // Rules error
      logger_->critical(
        "Permission denied: {}. Did you restart the system after loading the new rules?", e.what());
      throw;
    } catch (const vn::timeout & e) {
      // Disconnect if any other error occurs
      sensor_.disconnect();
      logger_->debug(
        "Failed to connect to VectorNav sensor at {} with baud rate {}. Retrying with other "
        "supported baud rates",
        port_, br);
      ros::Duration(0.5).sleep();
    }

    // Reached the end of the supported baud rates and failed to connect
    if (&br == &supported_baud_rates.back()) {
      logger_->error(
        "Failed to connect to VectorNav sensor at {} with any of the supported baud rates {}",
        port_, fmt::join(supported_baud_rates, ", "));
      throw std::runtime_error("Failed to connect to VectorNav sensor");
    }
  }

  if (!sensor_.verifySensorConnectivity()) {
    logger_->critical("Sensor connectivity check failed");
    throw std::runtime_error("Sensor connectivity check failed");
  }
  logger_->info("Sensor connectivity check passed");
}

void VectorNav::DisconnectSensor() { sensor_.disconnect(); }

void VectorNav::SetupSensor()
{
  ConnectSensor();

  PrintSensorInfo();

  if (factory_reset_before_start_) {
    logger_->info("Restoring to Factory Settings");
    sensor_.restoreFactorySettings();
    DisconnectSensor();
    ConnectSensor();
  }

  // Stop any sort of data coming from the sensor before writing a config
  logger_->debug("Turning off data streaming");
  sensor_.writeAsyncDataOutputType(VNOFF);

  // Write the reference vectors to the sensor
  logger_->debug("Writing reference vectors");
  try {
    sensor_.writeMagneticAndGravityReferenceVectors(Convert(mag_ref_), Convert(gravity_ref_));
  } catch (const vn::sensors::sensor_error & e) {
    logger_->error(
      "Failed to write reference vectors: {}. Do the mag and gravity ref values make sense?",
      e.what());
    throw;
  }

  // Setup using the binary output registers. This is significantly faster than using ASCII output
  logger_->debug("Setting up binary output registers");
  vn::sensors::BinaryOutputRegister bor(
    async_mode_, async_rate_divisor_,
    COMMONGROUP_TIMESTARTUP | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL |
      COMMONGROUP_MAGPRES |
      (is_triggered_ ? COMMONGROUP_TIMESYNCIN | COMMONGROUP_SYNCINCNT : COMMONGROUP_NONE) |
      (publish_uncomp_imu_ ? COMMONGROUP_IMU : COMMONGROUP_NONE),
    (is_triggering_ ? TIMEGROUP_SYNCOUTCNT : TIMEGROUP_NONE),
    publish_uncomp_mag_ ? IMUGROUP_UNCOMPMAG : IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
    INSGROUP_NONE, GPSGROUP_NONE);

  vn::sensors::BinaryOutputRegister bor_none(
    ASYNCMODE_NONE, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE,
    ATTITUDEGROUP_NONE, INSGROUP_NONE, GPSGROUP_NONE);

  // Use only one binary output register to minimize data usage. The other registers are
  // explicitly disabled so that they don't get used due to a configuration already on the sensor
  sensor_.writeBinaryOutput1(bor);
  sensor_.writeBinaryOutput2(bor_none);
  sensor_.writeBinaryOutput3(bor_none);

  // Setup synchonization
  logger_->debug("Setting up synchronization");
  sensor_.writeSynchronizationControl(
    is_triggered_ ? SYNCINMODE_IMU : SYNCINMODE_COUNT, SYNCINEDGE_RISING, sync_in_skip_factor_,
    is_triggering_ ? SYNCOUTMODE_ITEMSTART : SYNCOUTMODE_NONE, SYNCOUTPOLARITY_POSITIVE,
    sync_out_skip_factor_, sync_out_pulse_width_);

  // Write the changed params to the flash memory
  if (write_to_flash_) sensor_.writeSettings();
}

void VectorNav::ResetSensor() { sensor_.reset(); }

void VectorNav::StopSensor()
{
  // Might need sleeps here to ensure the connection is closed properly - Need to check
  logger_->info("Disconnecting from sensor");
  sensor_.unregisterAsyncPacketReceivedHandler();
  DisconnectSensor();
}

bool VectorNav::ResetServiceCallback(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res)
{
  logger_->info(
    "Resetting VectorNav sensor. Internal Kalman Filter may take a few seconds to converge to "
    "correct accelerometer and gyro bias");
  ResetSensor();
  return true;
}

void VectorNav::SetupAsyncMessageCallback(vn::sensors::VnSensor::AsyncPacketReceivedHandler handler)
{
  // Setup callback
  logger_->debug("Setting up async message callback");
  sensor_.registerAsyncPacketReceivedHandler(this, handler);
  logger_->debug("Async message callback setup");
  logger_->info("Ready to receive data");
}

void VectorNav::BinaryAsyncMessageCallback(Packet & p, size_t index)
{
  const ros::Time arrival_stamp = ros::Time::now();
  if (!receiving_data_) {
    logger_->info("Receiving data");
    receiving_data_ = true;
  }
  logger_->trace("Received async message at timestamp: {}", arrival_stamp.toSec());

  logger_->trace("Parsing binary async message");
  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  logger_->trace("Finished parsing binary async message");

  // The system time since startup measured in nano seconds. The time since startup is based upon the internal
  // TXCO oscillator for the MCU. The accuracy of the internal TXCO is +/- 20ppm (-40C to 85C).
  uint64_t startup_time = cd.timeStartup();

  // The time since the last SyncIn trigger event expressed in nano seconds.
  uint64_t sync_in_time;
  if (cd.hasTimeSyncIn()) sync_in_time = cd.timeSyncIn();

  // Get corrected timestamp
  ros::Time corrected_stamp = CorrectTimestamp(arrival_stamp, startup_time, sync_in_time);

  logger_->trace("Publishing parsed data");
  // Sync Out Stamp
  if (pub_sync_out_stamp_.getNumSubscribers() && cd.hasSyncOutCnt()) {
    std_msgs::Header msg;
    msg.stamp = corrected_stamp;
    msg.frame_id = std::to_string(cd.syncOutCnt());
    if (publish_sync_out_stamp_on_change_) {
      if (cd.syncOutCnt() != sync_out_count_) {
        sync_out_count_ = cd.syncOutCnt();
        pub_sync_out_stamp_.publish(msg);
      }
    } else {
      pub_sync_out_stamp_.publish(msg);
    }
  }

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

const ros::Time VectorNav::CorrectTimestamp(
  const ros::Time & arrival_stamp, uint64_t startup_time, uint64_t sync_in_time)
{
  logger_->trace("Correcting timestamp");
  // TODO: Feature (parameterized)
  // Possible timestamp corrections (which method to use could be a parameter):
  // 1. Do nothing - Host timestamping (inaccurate due to transmission and processing delays)
  // 2. Get the ros time and time since startup from the sensor and use that as reference. This was implemented here (https://github.com/ntnu-arl/vn100_nodelet/blob/ba494935b48134e2ff01fcd2ef4676615c273e92/src/vn100_nodelet.cpp#L294-L306) but the timestamp seemed to drift to become completely unusable
  // 3. Synchronize with an external micro controller that is triggering the sensor and use the sync in time and the known triggers to correct the time
  // 4. Assume that the sensor has an almost constant rate and correct the timestamp based on the timestamp that is expected to arrive next - This is implemented here: https://github.com/dawonn/vectornav/blob/6824e8b668b889a76214636cbc00a21c0b208a29/src/main.cpp#L725-L749
  ros::Time corrected_stamp;
  corrected_stamp = arrival_stamp;
  logger_->trace(
    "Arrival timestamp: {} Corrected timestamp: {}", arrival_stamp.toSec(),
    corrected_stamp.toSec());
  return corrected_stamp;
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

  // TODO: Feature: Allow for transformation to different frame
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

  msg.linear_acceleration_covariance = linear_accel_covariance_;
  msg.angular_velocity_covariance = angular_vel_covariance_;
  msg.orientation_covariance = orientation_covariance_;
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
  msg.magnetic_field_covariance = mag_covariance_;
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

vn::math::vec3f VectorNav::Convert(const std::array<float, 3> & arr)
{
  return vn::math::vec3f(arr[0], arr[1], arr[2]);
}
}  // namespace vectornav