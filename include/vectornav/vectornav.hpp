#ifndef VECTORNAV_HPP_
#define VECTORNAV_HPP_

// VectorNav Driver
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/vector.h"

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
    ros::Publisher pub_uncomp_imu_;
    ros::Publisher pub_mag_;
    ros::Publisher pub_uncomp_mag_;
    ros::Publisher pub_pres_;
    ros::Publisher pub_temp_;
    vn::sensors::VnSensor sensor_;
    const Config config_;

   public:
    explicit VectorNav(const Config& config, ros::NodeHandle& pnh);
    ~VectorNav();
    void SetupSensor();
    void StopSensor();
    void SetupAsyncMessageCallback(vn::sensors::VnSensor::AsyncPacketReceivedHandler handler);
    void BinaryAsyncMessageCallback(Packet& p, size_t index);
    void PrintSensorInfo();
    void PopulateImuMsg(sensor_msgs::Imu& msg, vn::sensors::CompositeData& cd,
                        const ros::Time& time, bool uncomp);
    void PopulateMagMsg(sensor_msgs::MagneticField& msg, vn::sensors::CompositeData& cd,
                        const ros::Time& time, bool uncomp);
    void PopulateTempMsg(sensor_msgs::Temperature& msg, vn::sensors::CompositeData& cd,
                         const ros::Time& time);
    void PopulatePresMsg(sensor_msgs::FluidPressure& msg, vn::sensors::CompositeData& cd,
                         const ros::Time& time);
  };
}  // namespace vectornav
#endif  // VECTORNAV_HPP_