// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef VECTORNAV_DRIVER_VECTORNAV_DRIVER_HPP_
#define VECTORNAV_DRIVER_VECTORNAV_DRIVER_HPP_

// C++
#include <chrono>
#include <thread>

// vnproglib
#include "vn/compositedata.h"
#include "vn/sensors.h"

// vectornav_driver
#include "vectornav_driver/utils.hpp"

// ROS version-specific includes
#if ROS_VERSION == 1
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#endif

// spdlog
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace vn::protocol::uart;

namespace vectornav_driver
{
#if ROS_VERSION == 1
using NodeHandle = std::shared_ptr<ros::NodeHandle>;
using Publisher = ros::Publisher;
using Time = ros::Time;
using ImuMsg = sensor_msgs::Imu;
using MagneticFieldMsg = sensor_msgs::MagneticField;
using FluidPressureMsg = sensor_msgs::FluidPressure;
using TemperatureMsg = sensor_msgs::Temperature;
using HeaderMsg = std_msgs::Header;
#else
using NodeHandle = std::shared_ptr<rclcpp::Node>;
using Publisher = rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr;
using Time = rclcpp::Time;
using ImuMsg = sensor_msgs::msg::Imu;
using MagneticFieldMsg = sensor_msgs::msg::MagneticField;
using FluidPressureMsg = sensor_msgs::msg::FluidPressure;
using TemperatureMsg = sensor_msgs::msg::Temperature;
using HeaderMsg = std_msgs::msg::Header;
#endif

class VectorNavDriver
#if ROS_VERSION == 2
  : public std::enable_shared_from_this<VectorNavDriver>
#endif
{
private:
  // Members
  vn::sensors::VnSensor sensor_;
  // Member messages
  ImuMsg filter_data_msg_;
  ImuMsg imu_data_msg_;
  MagneticFieldMsg filter_mag_msg_;
  MagneticFieldMsg imu_mag_msg_;
  FluidPressureMsg pressure_msg_;
  TemperatureMsg temperature_msg_;

  // Node handle (unified for both ROS1 and ROS2)
  NodeHandle node_;
  
  // Publishers (using shared_ptr for both ROS1 and ROS2)
#if ROS_VERSION == 1
  std::shared_ptr<ros::Publisher> pub_filter_data_;
  std::shared_ptr<ros::Publisher> pub_imu_data_;
  std::shared_ptr<ros::Publisher> pub_filter_mag_;
  std::shared_ptr<ros::Publisher> pub_imu_mag_;
  std::shared_ptr<ros::Publisher> pub_pressure_;
  std::shared_ptr<ros::Publisher> pub_temperature_;
  std::shared_ptr<ros::Publisher> pub_sync_out_stamp_;

  // Services
  std::shared_ptr<ros::ServiceServer> srv_reset_;
#else
  rclcpp::Publisher<ImuMsg>::SharedPtr pub_filter_data_;
  rclcpp::Publisher<ImuMsg>::SharedPtr pub_imu_data_;
  rclcpp::Publisher<MagneticFieldMsg>::SharedPtr pub_filter_mag_;
  rclcpp::Publisher<MagneticFieldMsg>::SharedPtr pub_imu_mag_;
  rclcpp::Publisher<FluidPressureMsg>::SharedPtr pub_pressure_;
  rclcpp::Publisher<TemperatureMsg>::SharedPtr pub_temperature_;
  rclcpp::Publisher<HeaderMsg>::SharedPtr pub_sync_out_stamp_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
#endif

  // Parameters
  vn::sensors::VnSensor::Family sensor_family_;
  std::string port_;
  uint32_t baud_rate_;
  uint16_t async_mode_;
  uint16_t async_rate_divisor_;
  // Configuration for the sensor being triggered by an external source
  bool is_triggered_;
  uint16_t sync_in_skip_factor_;
  // Configuration for the sensor triggering external objects
  bool is_triggering_;
  uint16_t sync_out_skip_factor_;
  uint32_t sync_out_pulse_width_;
  bool publish_sync_out_stamp_on_change_;
  uint32_t sync_out_count_;
  // IMU filtering configuration
  uint16_t mag_window_size_;
  uint16_t accel_window_size_;
  uint16_t gyro_window_size_;
  uint16_t temp_window_size_;
  uint16_t pres_window_size_;
  vn::protocol::uart::FilterMode mag_filter_mode_;
  vn::protocol::uart::FilterMode accel_filter_mode_;
  vn::protocol::uart::FilterMode gyro_filter_mode_;
  vn::protocol::uart::FilterMode temp_filter_mode_;
  vn::protocol::uart::FilterMode pres_filter_mode_;
  std::string frame_id_;
  double linear_acceleration_stddev_;
  double angular_velocity_stddev_;
  double magnetic_field_stddev_;
  double orientation_stddev_;
  double temperature_stddev_;
  double pressure_stddev_;
  std::string log_directory_;
  spdlog::level::level_enum logger_log_level_;
  spdlog::level::level_enum logger_flush_log_level_;
  spdlog::level::level_enum file_log_level_;
  spdlog::level::level_enum console_log_level_;
  bool set_reference_frame_;
#if ROS_VERSION == 1
  boost::array<double, 9ul> reference_frame_;
#else
  std::array<double, 9> reference_frame_;
#endif
  bool write_to_flash_;
  bool factory_reset_before_start_;

  // Logging
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_;

public:
  VectorNavDriver(NodeHandle node);
  ~VectorNavDriver();

  void readParams();
  void verifyParams();
  void connectSensor();
  void setupSensor();
  void resetSensor();
  void stopSensor();
  
#if ROS_VERSION == 1
  bool resetServiceCallback(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res);
#else
  void resetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
#endif
  
  void binaryAsyncMessageCallback(Packet & p, size_t index);
  void populateImuMsg(vn::sensors::CompositeData & cd, const Time & time, bool filter);
  void populateMagMsg(vn::sensors::CompositeData & cd, const Time & time, bool filter);
  void populateTempMsg(vn::sensors::CompositeData & cd, const Time & time);
  void populatePresMsg(vn::sensors::CompositeData & cd, const Time & time);
};
}  // namespace vectornav_driver
#endif  // VECTORNAV_DRIVER_VECTORNAV_DRIVER_HPP_
