#ifndef VECTORNAV_HPP_
#define VECTORNAV_HPP_

// VectorNav Driver
#include "vn/sensors.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>

// Config
#include "vectornav/config.hpp"

using namespace vn::protocol::uart;

namespace vectornav
{
  class VectorNav
  {
   private:
    ros::NodeHandle pnh_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_imu_uncomp_;
    ros::Publisher pub_mag_;
    ros::Publisher pub_pres_;
    ros::Publisher pub_temp_;
    vn::sensors::VnSensor sensor_;
    const Config config_;

   public:
    explicit VectorNav(const Config& config, ros::NodeHandle& pnh);
    ~VectorNav();
    void StartSensor();
    void StopSensor();
    static void BinaryAsyncMessageCallback(void* user_data, Packet& p, size_t index);

    void PrintSensorInfo();
  };
}  // namespace vectornav
#endif  // VECTORNAV_HPP_