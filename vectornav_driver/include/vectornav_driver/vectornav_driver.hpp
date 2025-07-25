// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef VECTORNAV_DRIVER_VECTORNAV_DRIVER_HPP_
#define VECTORNAV_DRIVER_VECTORNAV_DRIVER_HPP_

// vnproglib
#include "vn/compositedata.h"
#include "vn/sensors.h"

// vectornav_driver
#include "vectornav_driver/utils.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/empty.hpp>

// spdlog
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace vn::protocol::uart;

namespace vectornav_driver
{
class VectorNavDriver : public rclcpp::Node
{
private:
  // Members
  vn::sensors::VnSensor sensor_;
  // Member messages
  sensor_msgs::msg::Imu filter_data_msg_;
  sensor_msgs::msg::Imu imu_data_msg_;
  sensor_msgs::msg::MagneticField filter_mag_msg_;
  sensor_msgs::msg::MagneticField imu_mag_msg_;
  sensor_msgs::msg::FluidPressure pressure_msg_;
  sensor_msgs::msg::Temperature temperature_msg_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_filter_data_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_data_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_filter_mag_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_imu_mag_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_pressure_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temperature_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_sync_out_stamp_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;

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
  std::array<double, 9> reference_frame_;
  bool write_to_flash_;
  bool factory_reset_before_start_;

  // Logging
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_;

public:
  VectorNavDriver();
  ~VectorNavDriver();
  void readParams();
  void verifyParams();
  void connectSensor();
  void setupSensor();
  void resetSensor();
  void stopSensor();
  void resetServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void binaryAsyncMessageCallback(Packet & p, size_t index);
  void populateImuMsg(vn::sensors::CompositeData & cd, const rclcpp::Time & time, bool filter);
  void populateMagMsg(vn::sensors::CompositeData & cd, const rclcpp::Time & time, bool filter);
  void populateTempMsg(vn::sensors::CompositeData & cd, const rclcpp::Time & time);
  void populatePresMsg(vn::sensors::CompositeData & cd, const rclcpp::Time & time);
};
}  // namespace vectornav_driver
#endif  // VECTORNAV_DRIVER_VECTORNAV_DRIVER_HPP_
