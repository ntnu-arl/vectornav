#include "vectornav/vectornav.hpp"

namespace vectornav
{
  VectorNav::VectorNav(const Config& config, ros::NodeHandle& pnh) : config_(config), pnh_(pnh)
  {
    // Print the config
    std::cout << config_.toString() << std::endl;

    // Setup Publishers
    pub_imu_ = pnh_.advertise<sensor_msgs::Imu>("imu", 1000, false);
    pub_uncomp_imu_ = pnh_.advertise<sensor_msgs::Imu>("uncomp_imu", 1000, false);
    pub_mag_ = pnh_.advertise<sensor_msgs::MagneticField>("magetic_field", 1000, false);
    pub_uncomp_mag_ =
        pnh_.advertise<sensor_msgs::MagneticField>("uncomp_magetic_field", 1000, false);
    pub_temp_ = pnh_.advertise<sensor_msgs::Temperature>("temperature", 1000, false);
    pub_pres_ = pnh_.advertise<sensor_msgs::FluidPressure>("pressure", 1000, false);
  }

  VectorNav::~VectorNav() {}

  void VectorNav::SetupSensor()
  {
    // Should try to set the baud rate instead of blindly trusting the baud rate to be available
    // Connect to sensor
    ROS_INFO("VectorNav: Connecting to sensor at %s @ %d", config_.port.c_str(), config_.baud_rate);
    sensor_.connect(config_.port, config_.baud_rate);

    if (!sensor_.verifySensorConnectivity())
    {
      ROS_FATAL("Sensor connectivity test failed");
      return;
    }
    ROS_INFO("VectorNav: Sensor connected");

    PrintSensorInfo();

    // Stop any sort of data coming from the sensor before writing a config
    sensor_.writeAsyncDataOutputType(VNOFF);

    // Setup using the binary output registers. This is significantly faster than using ASCII output
    vn::sensors::BinaryOutputRegister bor(
        ASYNCMODE_PORT2, config_.async_rate_divisor,
        COMMONGROUP_TIMESTARTUP |
            (config_.is_triggered ? COMMONGROUP_TIMESYNCIN | COMMONGROUP_SYNCINCNT
                                  : COMMONGROUP_NONE) |
            COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL |
            COMMONGROUP_MAGPRES,
        TIMEGROUP_NONE,
        config_.get_uncomp_measurements
            ? (IMUGROUP_UNCOMPACCEL | IMUGROUP_UNCOMPGYRO | IMUGROUP_UNCOMPMAG)
            : IMUGROUP_NONE,
        GPSGROUP_NONE, ATTITUDEGROUP_NONE, INSGROUP_NONE, GPSGROUP_NONE);

    vn::sensors::BinaryOutputRegister bor_none(ASYNCMODE_NONE, 1, COMMONGROUP_NONE, TIMEGROUP_NONE,
                                               IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
                                               INSGROUP_NONE, GPSGROUP_NONE);

    // Use only one binary output register to minimize data usage. The other registers are
    // explicitly disabled so that they dont get used due to a configuration already on the sensor
    // F: Factory reset before usage?
    sensor_.writeBinaryOutput1(bor);
    sensor_.writeBinaryOutput2(bor_none);
    sensor_.writeBinaryOutput3(bor_none);

    // Setup synchonization
    sensor_.writeSynchronizationControl(
        config_.is_triggered ? SYNCINMODE_IMU : SYNCINMODE_COUNT, SYNCINEDGE_RISING,
        config_.sync_in_skip_factor,
        config_.is_triggering ? SYNCOUTMODE_ITEMSTART : SYNCOUTMODE_NONE, SYNCOUTPOLARITY_POSITIVE,
        config_.sync_out_skip_factor, config_.sync_out_pulse_width);
  }

  void VectorNav::StopSensor()
  {
    // Might need sleeps here to ensure the connection is closed properly
    sensor_.unregisterAsyncPacketReceivedHandler();
    sensor_.disconnect();
  }

  void VectorNav::SetupAsyncMessageCallback(
      vn::sensors::VnSensor::AsyncPacketReceivedHandler handler)
  {
    // Setup callback
    sensor_.registerAsyncPacketReceivedHandler(this, handler);
  }

  void VectorNav::BinaryAsyncMessageCallback(Packet& p, size_t index)
  {
    const ros::Time ros_time = ros::Time::now();
    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);

    // Get corrected timestamp
    ros::Time corrected_time = ros_time;

    // IMU
    if (pub_imu_.getNumSubscribers())
    {
      sensor_msgs::Imu msg;
      PopulateImuMsg(msg, cd, corrected_time, false);
      pub_imu_.publish(msg);
    }

    // Uncompensated IMU
    if (pub_uncomp_imu_.getNumSubscribers() && config_.get_uncomp_measurements)
    {
      sensor_msgs::Imu msg;
      PopulateImuMsg(msg, cd, corrected_time, true);
      pub_uncomp_imu_.publish(msg);
    }

    // Magnetic Field
    if (pub_mag_.getNumSubscribers())
    {
      sensor_msgs::MagneticField msg;
      PopulateMagMsg(msg, cd, corrected_time, false);
      pub_mag_.publish(msg);
    }

    // Uncompensated Magnetic Field
    if (pub_uncomp_mag_.getNumSubscribers())
    {
      sensor_msgs::MagneticField msg;
      PopulateMagMsg(msg, cd, corrected_time, true);
      pub_uncomp_mag_.publish(msg);
    }

    // Temperature
    if (pub_temp_.getNumSubscribers())
    {
      sensor_msgs::Temperature msg;
      PopulateTempMsg(msg, cd, corrected_time);
      pub_temp_.publish(msg);
    }

    // Pressure
    if (pub_pres_.getNumSubscribers())
    {
      sensor_msgs::FluidPressure msg;
      PopulatePresMsg(msg, cd, corrected_time);
      pub_pres_.publish(msg);
    }
  }

  void VectorNav::PrintSensorInfo()
  {
    ROS_INFO("VectorNav: Model Number: %s", sensor_.readModelNumber().c_str());
    ROS_INFO("VectorNav: Firmware Version: %s", sensor_.readFirmwareVersion().c_str());
    ROS_INFO("VectorNav: Hardware Revision: %d", sensor_.readHardwareRevision());
    ROS_INFO("VectorNav: Serial Number: %d", sensor_.readSerialNumber());
  }

  void VectorNav::PopulateImuMsg(sensor_msgs::Imu& msg, vn::sensors::CompositeData& cd,
                                 const ros::Time& time, bool uncomp)
  {
    msg.header.stamp = time;
    msg.header.frame_id = config_.frame_id;

    vn::math::vec4f q = cd.quaternion();
    vn::math::vec3f acc, omega;  // m/s^2, rad/s
    if (uncomp)
    {
      acc = cd.accelerationUncompensated();
      omega = cd.angularRateUncompensated();
    }
    else
    {
      acc = cd.acceleration();
      omega = cd.angularRate();
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

  void VectorNav::PopulateMagMsg(sensor_msgs::MagneticField& msg, vn::sensors::CompositeData& cd,
                                 const ros::Time& time, bool uncomp)
  {
    msg.header.stamp = time;
    msg.header.frame_id = config_.frame_id;
    vn::math::vec3f mag;  // gauss
    if (config_.get_uncomp_measurements)
    {
      mag = cd.magnetic();
    }
    else
    {
      mag = cd.magneticUncompensated();
    }
    // Add covariance from config. is it possible to get this from sensor?
  }

  void VectorNav::PopulateTempMsg(sensor_msgs::Temperature& msg, vn::sensors::CompositeData& cd,
                                  const ros::Time& time)
  {
    msg.header.stamp = time;
    msg.header.frame_id = config_.frame_id;
    msg.temperature = cd.temperature();  // Celsius
    // Add covariance from config. is it possible to get this from sensor?
  }

  void VectorNav::PopulatePresMsg(sensor_msgs::FluidPressure& msg, vn::sensors::CompositeData& cd,
                                  const ros::Time& time)
  {
    msg.header.stamp = time;
    msg.header.frame_id = config_.frame_id;
    msg.fluid_pressure = cd.pressure();  // kPa
    // Add covariance from config. is it possible to get this from sensor?
  }

}  // namespace vectornav