#include "vectornav/vectornav.hpp"

namespace vectornav
{
  VectorNav::VectorNav(const Config& config, ros::NodeHandle& pnh) : config_(config), pnh_(pnh)
  {
    config_utilities::getConfigFromRos<Config>(pnh_);
    pub_imu_ = pnh_.advertise<sensor_msgs::Imu>("imu", 1000, false);
    pub_imu_ = pnh_.advertise<sensor_msgs::Imu>("uncomp_imu", 1000, false);
    pub_imu_ = pnh_.advertise<sensor_msgs::MagneticField>("magetic_field", 1000, false);
    pub_imu_ = pnh_.advertise<sensor_msgs::Temperature>("temperature", 1000, false);
    pub_imu_ = pnh_.advertise<sensor_msgs::FluidPressure>("pressure", 1000, false);
  }

  VectorNav::~VectorNav() {}

  void VectorNav::StartSensor()
  {
    // Should try to set the baud rate instead of blindly trusting the baud rate to be available
    // Connect to sensor
    sensor_.connect(config_.port, config_.baud_rate);

    if (!sensor_.verifySensorConnectivity())
    {
      ROS_FATAL("Sensor connectivity test failed");
      return;
    }
    ROS_INFO("Sensor connected");

    PrintSensorInfo();

    sensor_.writeAsyncDataOutputType(VNOFF);

    vn::sensors::BinaryOutputRegister bor(
        ASYNCMODE_PORT2, config_.async_rate_divisor,
        COMMONGROUP_TIMESTARTUP |
            (config_.is_triggered ? COMMONGROUP_TIMESYNCIN | COMMONGROUP_SYNCINCNT : COMMONGROUP_NONE) |
            COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL |
            COMMONGROUP_MAGPRES,
        TIMEGROUP_NONE, config_.get_uncomp_measurements ? (IMUGROUP_UNCOMPACCEL | IMUGROUP_UNCOMPGYRO | IMUGROUP_UNCOMPMAG) : IMUGROUP_NONE,
        GPSGROUP_NONE, ATTITUDEGROUP_NONE, INSGROUP_NONE, GPSGROUP_NONE);

    vn::sensors::BinaryOutputRegister bor_none(ASYNCMODE_NONE, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE, INSGROUP_NONE, GPSGROUP_NONE);

    sensor_.writeBinaryOutput1(bor);
    sensor_.writeBinaryOutput2(bor_none);
    sensor_.writeBinaryOutput3(bor_none);

    // Setup synchonization
    sensor_.writeSynchronizationControl(
        config_.is_triggered ? SYNCINMODE_IMU : SYNCINMODE_COUNT, SYNCINEDGE_RISING,
        config_.sync_in_skip_factor,
        config_.is_triggering ? SYNCOUTMODE_ITEMSTART : SYNCOUTMODE_NONE, SYNCOUTPOLARITY_POSITIVE,
        config_.sync_out_skip_factor, config_.sync_out_pulse_width);

    // Setup callback
    sensor_.registerAsyncPacketReceivedHandler(NULL, &BinaryAsyncMessageCallback);
  }

  void VectorNav::StopSensor()
  {
    // Might need sleeps here to ensure the connection is closed properly
    sensor_.unregisterAsyncPacketReceivedHandler();
    sensor_.disconnect();
  }

  void VectorNav::BinaryAsyncMessageCallback(void* user_data, Packet& p, size_t index)
  {
    if (p.type() == Packet::TYPE_BINARY)
    {
      // Verify that the packet contains data that is expected
      if (!p.isCompatible(COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL, TIMEGROUP_NONE,
                          IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE, INSGROUP_NONE,
                          GPSGROUP_NONE))
        return;
    }
  }

  void VectorNav::PrintSensorInfo()
  {
    ROS_INFO("Model Number: %s", sensor_.readModelNumber());
    ROS_INFO("Firmware Version: %s", sensor_.readFirmwareVersion());
    ROS_INFO("Hardware Revision: %d", sensor_.readHardwareRevision());
    ROS_INFO("Serial Number: %d", sensor_.readSerialNumber());
  }
}  // namespace vectornav