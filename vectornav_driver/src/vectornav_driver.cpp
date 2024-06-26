// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "vectornav_driver/vectornav_driver.hpp"

namespace vectornav_driver
{
void callbackWrapper(void * user_data, Packet & p, size_t index)
{
  static_cast<VectorNavDriver *>(user_data)->binaryAsyncMessageCallback(p, index);
}

VectorNavDriver::VectorNavDriver(ros::NodeHandle & pnh)
{
  // Get parameters
  readParams(pnh);
  verifyParams();

  // Setup logging sinks
  logger_console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  logger_console_sink_->set_level(console_log_level_);
  logger_file_sink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
    log_directory_ + ros::this_node::getName() + ".log", true);
  logger_file_sink_->set_level(file_log_level_);

  // Create logger
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(logger_console_sink_);
  sinks.push_back(logger_file_sink_);
  logger_ = std::make_shared<spdlog::logger>(
    ros::this_node::getName() + "_logger", sinks.begin(), sinks.end());
  logger_->set_level(logger_log_level_);
  logger_->flush_on(logger_flush_log_level_);

  // Setup member messages
  filter_data_msg_.header.frame_id = frame_id_;
  setDiagonal(
    std::pow(linear_acceleration_stddev_, 2), filter_data_msg_.linear_acceleration_covariance);
  setDiagonal(std::pow(angular_velocity_stddev_, 2), filter_data_msg_.angular_velocity_covariance);
  setDiagonal(std::pow(orientation_stddev_, 2), filter_data_msg_.orientation_covariance);

  imu_data_msg_.header.frame_id = frame_id_;
  setDiagonal(
    std::pow(linear_acceleration_stddev_, 2), imu_data_msg_.linear_acceleration_covariance);
  setDiagonal(std::pow(angular_velocity_stddev_, 2), imu_data_msg_.angular_velocity_covariance);

  filter_mag_msg_.header.frame_id = frame_id_;
  setDiagonal(std::pow(magnetic_field_stddev_, 2), filter_mag_msg_.magnetic_field_covariance);

  imu_mag_msg_.header.frame_id = frame_id_;
  setDiagonal(std::pow(magnetic_field_stddev_, 2), imu_mag_msg_.magnetic_field_covariance);

  pressure_msg_.header.frame_id = frame_id_;
  pressure_msg_.variance = std::pow(pressure_stddev_, 2);

  temperature_msg_.header.frame_id = frame_id_;
  temperature_msg_.variance = std::pow(temperature_stddev_, 2);

  // Setup Publishers
  logger_->debug("Setting up publishers");
  pub_filter_data_ = pnh.advertise<sensor_msgs::Imu>("filter/data", 1000, false);
  pub_imu_data_ = pnh.advertise<sensor_msgs::Imu>("imu/data", 1000, false);
  pub_filter_mag_ = pnh.advertise<sensor_msgs::MagneticField>("filter/mag", 1000, false);
  pub_imu_mag_ = pnh.advertise<sensor_msgs::MagneticField>("imu/mag", 1000, false);
  pub_temperature_ = pnh.advertise<sensor_msgs::Temperature>("temperature", 1000, false);
  pub_pressure_ = pnh.advertise<sensor_msgs::FluidPressure>("pressure", 1000, false);
  pub_sync_out_stamp_ = pnh.advertise<std_msgs::Header>("sync_out_stamp", 1000, false);

  // Setup Subscribers
  if (use_sensor_sync_) {
    logger_->debug("Setting up subscribers");
    sub_trigger_stamp_ = pnh.subscribe(
      "/sensor_sync_node/trigger_0", 10, &VectorNavDriver::triggerStampCallback, this,
      ros::TransportHints().tcpNoDelay());
    pub_time_sync_in_ = pnh.advertise<sensor_msgs::TimeReference>("time_sync_in", 1000, false);
  }

  // Setup Services
  logger_->debug("Setting up services");
  srv_reset_ = pnh.advertiseService("reset", &VectorNavDriver::resetServiceCallback, this);
}

VectorNavDriver::~VectorNavDriver()
{
  ros::param::del(ros::this_node::getName() + node_ready_param_name_);
}

void VectorNavDriver::readParams(ros::NodeHandle & pnh)
{
  // Read parameters
  int i_param;
  pnh.param<int>("sensor_family", i_param, 0);
  sensor_family_ = static_cast<vn::sensors::VnSensor::Family>(i_param);
  pnh.param<std::string>("port", port_, "/dev/ttyUSB0");
  pnh.param<int>("baud_rate", i_param, 921600);
  baud_rate_ = static_cast<uint32_t>(i_param);
  pnh.param<int>("async_mode", i_param, 2);
  async_mode_ = static_cast<AsyncMode>(i_param);
  pnh.param<int>("async_rate_divisor", i_param, 4);
  async_rate_divisor_ = static_cast<uint16_t>(i_param);
  pnh.param<bool>("is_triggered", is_triggered_, false);
  pnh.param<int>("sync_in_skip_factor", i_param, 0);
  sync_in_skip_factor_ = static_cast<uint16_t>(i_param);
  pnh.param<bool>("is_triggering", is_triggering_, false);
  pnh.param<int>("sync_out_skip_factor", i_param, 39);
  sync_out_skip_factor_ = static_cast<uint16_t>(i_param);
  pnh.param<int>("sync_out_pulse_width", i_param, 1.0e+7);
  pnh.param<bool>("publish_sync_out_stamp_on_change", publish_sync_out_stamp_on_change_, false);
  sync_out_pulse_width_ = static_cast<uint32_t>(i_param);
  pnh.param<std::string>("frame_id", frame_id_, "imu_link_ned");
  pnh.param<bool>("set_reference_frame", set_reference_frame_, false);
  XmlRpc::XmlRpcValue reference_frame_rpc;
  pnh.getParam("reference_frame", reference_frame_rpc);
  setMatrix(reference_frame_rpc, reference_frame_);
  pnh.param<int>("mag_window_size", i_param, 0);
  std::cout << "mag_window_size: " << i_param << "\n";
  mag_window_size_ = static_cast<uint16_t>(i_param);
  std::cout << "mag_window_size_: " << mag_window_size_ << "\n";
  pnh.param<int>("accel_window_size", i_param, 4);
  accel_window_size_ = static_cast<uint16_t>(i_param);
  pnh.param<int>("gyro_window_size", i_param, 4);
  gyro_window_size_ = static_cast<uint16_t>(i_param);
  pnh.param<int>("temp_window_size", i_param, 4);
  temp_window_size_ = static_cast<uint16_t>(i_param);
  pnh.param<int>("pres_window_size", i_param, 4);
  pres_window_size_ = static_cast<uint16_t>(i_param);
  pnh.param<int>("mag_filter_mode", i_param, 0);
  mag_filter_mode_ = static_cast<vn::protocol::uart::FilterMode>(i_param);
  pnh.param<int>("accel_filter_mode", i_param, 3);
  accel_filter_mode_ = static_cast<vn::protocol::uart::FilterMode>(i_param);
  pnh.param<int>("gyro_filter_mode", i_param, 3);
  gyro_filter_mode_ = static_cast<vn::protocol::uart::FilterMode>(i_param);
  pnh.param<int>("temp_filter_mode", i_param, 3);
  temp_filter_mode_ = static_cast<vn::protocol::uart::FilterMode>(i_param);
  pnh.param<int>("pres_filter_mode", i_param, 0);
  pres_filter_mode_ = static_cast<vn::protocol::uart::FilterMode>(i_param);
  pnh.param<bool>("write_to_flash", write_to_flash_, false);
  pnh.param<bool>("factory_reset_before_start", factory_reset_before_start_, false);
  pnh.param<double>("linear_acceleration_stddev", linear_acceleration_stddev_, 0.0);
  pnh.param<double>("angular_velocity_stddev", angular_velocity_stddev_, 0.0);
  pnh.param<double>("magnetic_field_stddev", magnetic_field_stddev_, 0.0);
  pnh.param<double>("orientation_stddev", orientation_stddev_, 0.0);
  pnh.param<double>("temperature_stddev", temperature_stddev_, 0.0);
  pnh.param<double>("pressure_stddev", pressure_stddev_, 0.0);
  pnh.param<std::string>("log_directory", log_directory_, "/tmp/vectornav_driver/");
  pnh.param<int>("log_level/logger", i_param, 0);
  logger_log_level_ = static_cast<spdlog::level::level_enum>(i_param);
  pnh.param<int>("log_level/flush", i_param, 0);
  logger_flush_log_level_ = static_cast<spdlog::level::level_enum>(i_param);
  pnh.param<int>("log_level/file", i_param, 0);
  file_log_level_ = static_cast<spdlog::level::level_enum>(i_param);
  pnh.param<int>("log_level/console", i_param, 0);
  console_log_level_ = static_cast<spdlog::level::level_enum>(i_param);
  pnh.param<bool>("use_sensor_sync", use_sensor_sync_, false);
}

void VectorNavDriver::verifyParams()
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
  if (use_sensor_sync_) {
    if (!is_triggered_) {
      throw std::runtime_error("Sensor sync is enabled but is_triggered is false");
    }
  }
}

void VectorNavDriver::connectSensor()
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
      logger_->info("Connected to VectorNavDriver sensor at {} with baud rate {}", port_, br);
      if (baud_rate_ != br) {
        logger_->info("Baud rate changed from {} to {}", br, baud_rate_);
      }
      break;
    } catch (const vn::permission_denied & e) {
      // Rules error
      logger_->critical(
        "Permission denied with error message: \"{}\". Did you restart the system after loading "
        "the new rules? Are you in the correct groups to access the device?",
        e.what());
      throw;
    } catch (const vn::timeout & e) {
      // Disconnect if any other error occurs
      sensor_.disconnect();
      logger_->debug(
        "Failed to connect to VectorNavDriver sensor at {} with baud rate {}. Retrying with other "
        "supported baud rates",
        port_, br);
      ros::Duration(0.5).sleep();
    }

    // Reached the end of the supported baud rates and failed to connect
    if (&br == &supported_baud_rates.back()) {
      logger_->critical(
        "Failed to connect to VectorNavDriver sensor at {} with any of the supported baud rates {}",
        port_, fmt::join(supported_baud_rates, ", "));
      throw std::runtime_error("Failed to connect to VectorNavDriver sensor");
    }
  }

  if (!sensor_.verifySensorConnectivity()) {
    logger_->critical("Sensor connectivity check failed");
    throw std::runtime_error("Sensor connectivity check failed");
  }
  logger_->info("Sensor connectivity check passed");

  sensor_.writeSettings();  // Write the baud rate to the sensor
  // Reset the sensor for a clean start
  resetSensor();
}

void VectorNavDriver::setupSensor()
{
  connectSensor();

  logger_->info(
    "Sensor information:\n\tModel Number: {}\n\tSerial Number: {}\n\tFirmware Version: "
    "{}\n\tHardware Revision: {}",
    sensor_.readModelNumber(), sensor_.readSerialNumber(), sensor_.readFirmwareVersion(),
    sensor_.readHardwareRevision());

  if (factory_reset_before_start_) {
    logger_->info("Restoring to Factory Settings");
    sensor_.restoreFactorySettings();
    sensor_.disconnect();
    connectSensor();
  }

  // Stop any sort of data coming from the sensor before writing a config
  logger_->debug("Turning off data streaming");
  sensor_.writeAsyncDataOutputType(VNOFF);

  sensor_.writeImuFilteringConfiguration(
    mag_window_size_, accel_window_size_, gyro_window_size_, temp_window_size_, pres_window_size_,
    mag_filter_mode_, accel_filter_mode_, gyro_filter_mode_, temp_filter_mode_, pres_filter_mode_);

  if (set_reference_frame_) {
    logger_->debug("Writing reference frame");
    try {
      sensor_.writeReferenceFrameRotation(convert(reference_frame_));
    } catch (const vn::sensors::sensor_error & e) {
      logger_->critical(
        "Failed to write reference frame: {}. Is the reference frame an orthonormal and "
        "right-handed matrix? Is the tolerance within 1e-5? Are any parameters outside [-1,1]?",
        e.what());
      throw;
    }
    logger_->debug("Writing settings to flash");
    sensor_.writeSettings();
    resetSensor();
    logger_->debug("Reference frame written");
  }

  // Setup using the binary output registers. This is significantly faster than using ASCII output
  logger_->debug("Setting up binary output registers");
  vn::sensors::BinaryOutputRegister bor(
    async_mode_, (use_sensor_sync_ ? 1 : async_rate_divisor_),
    (is_triggered_ ? COMMONGROUP_TIMESYNCIN | COMMONGROUP_SYNCINCNT : COMMONGROUP_NONE) |
      COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL | COMMONGROUP_IMU,
    (is_triggering_ ? TIMEGROUP_SYNCOUTCNT : TIMEGROUP_NONE),
    IMUGROUP_UNCOMPMAG | IMUGROUP_TEMP | IMUGROUP_PRES, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
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

  // Setup handler for async messages
  logger_->debug("Setting up async message callback");
  sensor_.registerAsyncPacketReceivedHandler(this, callbackWrapper);
  logger_->debug("Async message callback setup");
  logger_->info("Ready to receive data");
  ros::param::set(ros::this_node::getName() + node_ready_param_name_, true);
}

void VectorNavDriver::resetSensor()
{
  sensor_.reset();
  // Wait for the sensor to reset - 1 second is a safe bet
  // Without the sleep, the next command will fail with a timeout
  sleep(1);
}

void VectorNavDriver::stopSensor()
{
  // Might need sleeps here to ensure the connection is closed properly - Need to check
  logger_->info("Disconnecting from sensor");
  trigger_stamp_deque_cv_.notify_all();
  sensor_.unregisterAsyncPacketReceivedHandler();
  sensor_.disconnect();
}

bool VectorNavDriver::resetServiceCallback(
  std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res)
{
  logger_->info(
    "Resetting VectorNavDriver sensor. Internal Kalman Filter may take a few seconds to converge "
    "to correct accelerometer and gyro bias");
  resetSensor();
  return true;
}

void VectorNavDriver::getTriggerBasedStamp(
  const uint64_t time_sync_in, const uint32_t sync_in_cnt, ros::Time & stamp)
{
  logger_->debug("Looking to match count: {}", sync_in_cnt);
  logger_->debug("Time since sync in: {}", time_sync_in);

  {
    std::unique_lock<std::mutex> lock(trigger_stamp_deque_mutex_);
    if (trigger_stamp_deque_.empty()) {
      logger_->warn("Waiting for trigger");

      auto tic = std::chrono::high_resolution_clock::now();

      trigger_stamp_deque_cv_.wait(
        lock, [this] { return !trigger_stamp_deque_.empty() || ros::isShuttingDown(); });
      // Exit if wakeup was due to ROS shutting down
      if (ros::isShuttingDown()) return;

      auto toc = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic);

      logger_->warn("\tDone waiting for trigger, took {}ms", duration.count() / 1e3);
      count_[0]++;
      logger_->info("error count: [{} waits for trigger, {} imu skips]\n", count_[0], count_[1]);
    }

    while (!trigger_stamp_deque_.empty()) {
      std_msgs::Header msg = trigger_stamp_deque_.front();
      uint32_t trigger_count = std::stoul(msg.frame_id);
      logger_->debug("\tchecking trigger count: {}", trigger_count);
      if (trigger_count == sync_in_cnt) {
        stamp = msg.stamp;
        trigger_stamp_deque_.pop_front();
        break;
      } else if (trigger_count < sync_in_cnt) {
        logger_->warn("dropping unused trigger message({}), IMU skip probably", trigger_count);
        trigger_stamp_deque_.pop_front();
        count_[1]++;
        logger_->info("error count: [{} waits for trigger, {} imu skips]\n", count_[0], count_[1]);
      } else {
        logger_->error("trigger count > sync_in_cnt, this should not happen");
        break;
      }
    }
  }

  // Offset the timestamp by the time_sync_in
  stamp += ros::Duration(0, time_sync_in);
}

void VectorNavDriver::binaryAsyncMessageCallback(Packet & p, size_t index)
{
  ros::Time stamp = ros::Time::now();

  static bool first = true;
  if (first) {
    first = false;
    logger_->info("Receiving data");
  }

  logger_->trace("Received async message at timestamp: {}", stamp.toSec());

  logger_->trace("Parsing binary async message");
  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  logger_->trace("Finished parsing binary async message");

  if (use_sensor_sync_) {
    // The time since the last SyncIn trigger event expressed in nano seconds.
    uint64_t time_sync_in = 0;
    if (cd.hasTimeSyncIn()) {
      time_sync_in = cd.timeSyncIn();
    } else {
      logger_->critical("No timeSyncIn in message while using sensor sync");
      return;
    }

    uint32_t sync_in_cnt = 0;
    if (cd.hasSyncInCnt()) {
      sync_in_cnt = cd.syncInCnt();
    } else {
      logger_->error("No syncInCnt in message while using sensor sync");
      return;
    }

    if (sync_in_cnt % async_rate_divisor_ != 0) {
      logger_->debug("Skipping message as sync_in_cnt % async_rate_divisor_ != 0");
      return;
    }

    getTriggerBasedStamp(time_sync_in, sync_in_cnt, stamp);
  }

  logger_->trace("Publishing parsed data");
  // Sync Out Stamp
  if (pub_sync_out_stamp_.getNumSubscribers() && cd.hasSyncOutCnt()) {
    std_msgs::Header msg;
    msg.stamp = stamp;
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
  
  // Time Sync In
  if (pub_time_sync_in_.getNumSubscribers() && cd.hasTimeSyncIn()) {
    sensor_msgs::TimeReference msg;
    msg.header.stamp = stamp;
    msg.time_ref = ros::Time(0, cd.timeSyncIn());
    pub_time_sync_in_.publish(msg);
  }

  // Filtered IMU
  if (pub_filter_data_.getNumSubscribers()) {
    populateImuMsg(cd, stamp, true);
    pub_filter_data_.publish(filter_data_msg_);
  }

  // IMU
  if (pub_imu_data_.getNumSubscribers()) {
    populateImuMsg(cd, stamp, false);
    pub_imu_data_.publish(imu_data_msg_);
  }

  // Magnetic Field
  if (pub_imu_mag_.getNumSubscribers()) {
    populateMagMsg(cd, stamp, false);
    pub_imu_mag_.publish(imu_mag_msg_);
  }

  // Temperature
  if (pub_temperature_.getNumSubscribers()) {
    populateTempMsg(cd, stamp);
    pub_temperature_.publish(temperature_msg_);
  }

  // Pressure
  if (pub_pressure_.getNumSubscribers()) {
    populatePresMsg(cd, stamp);
    pub_pressure_.publish(pressure_msg_);
  }
}

void VectorNavDriver::populateImuMsg(
  vn::sensors::CompositeData & cd, const ros::Time & stamp, bool filter)
{
  sensor_msgs::Imu & msg = filter ? filter_data_msg_ : imu_data_msg_;
  msg.header.stamp = stamp;

  vn::math::vec3f acc, omega;  // m/s^2, rad/s
  if (!filter) {
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

    vn::math::vec4f q = cd.quaternion();
    msg.orientation.x = q[0];
    msg.orientation.y = q[1];
    msg.orientation.z = q[2];
    msg.orientation.w = q[3];

    logger_->trace("Populating IMU message");
  }

  msg.angular_velocity.x = omega[0];
  msg.angular_velocity.y = omega[1];
  msg.angular_velocity.z = omega[2];

  msg.linear_acceleration.x = acc[0];
  msg.linear_acceleration.y = acc[1];
  msg.linear_acceleration.z = acc[2];
}

void VectorNavDriver::populateMagMsg(
  vn::sensors::CompositeData & cd, const ros::Time & stamp, bool filter)
{
  sensor_msgs::MagneticField & msg = filter ? filter_mag_msg_ : imu_mag_msg_;
  msg.header.stamp = stamp;

  vn::math::vec3f mag;  // gauss
  if (!filter) {
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

  // Convert from gauss to Tesla
  mag = mag * 1.0e-4;

  // Fill in message
  msg.magnetic_field.x = mag[0];
  msg.magnetic_field.y = mag[1];
  msg.magnetic_field.z = mag[2];
}

void VectorNavDriver::populateTempMsg(vn::sensors::CompositeData & cd, const ros::Time & stamp)
{
  logger_->trace("Populating Temperature message");
  temperature_msg_.header.stamp = stamp;
  temperature_msg_.temperature = cd.temperature();  // Celsius
}

void VectorNavDriver::populatePresMsg(vn::sensors::CompositeData & cd, const ros::Time & stamp)
{
  logger_->trace("Populating Pressure message");
  pressure_msg_.header.stamp = stamp;
  pressure_msg_.fluid_pressure = cd.pressure() * 1.0e3;  // Convert to Pa
}

void VectorNavDriver::triggerStampCallback(const std_msgs::HeaderConstPtr msg)
{
  std::lock_guard<std::mutex> lock(trigger_stamp_deque_mutex_);
  logger_->debug("Added trigger for count: {}", msg->frame_id);
  trigger_stamp_deque_.push_back(*msg);
  trigger_stamp_deque_cv_.notify_one();
}

}  // namespace vectornav_driver
