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

// ROS
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
#include <std_srvs/Empty.h>

// spdlog
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

// C++
#include <condition_variable>
#include <deque>
#include <mutex>

using namespace vn::protocol::uart;

namespace vectornav_driver
{
class VectorNavDriver
{
private:
  // Members
  vn::sensors::VnSensor sensor_;
  std::deque<std_msgs::Header> trigger_stamp_deque_;
  std::mutex trigger_stamp_deque_mutex_;
  std::condition_variable trigger_stamp_deque_cv_;
  const std::string node_ready_param_name_{"/ready"};
  std::vector<int> count_{0, 0, 0};

  // Member messages
  sensor_msgs::Imu filter_data_msg_;
  sensor_msgs::Imu imu_data_msg_;
  sensor_msgs::MagneticField filter_mag_msg_;
  sensor_msgs::MagneticField imu_mag_msg_;
  sensor_msgs::FluidPressure pressure_msg_;
  sensor_msgs::Temperature temperature_msg_;

  // Publishers
  ros::Publisher pub_filter_data_;
  ros::Publisher pub_imu_data_;
  ros::Publisher pub_filter_mag_;
  ros::Publisher pub_imu_mag_;
  ros::Publisher pub_pressure_;
  ros::Publisher pub_temperature_;
  ros::Publisher pub_sync_out_stamp_;
  ros::Publisher pub_time_sync_in_;

  // Subscribers
  ros::Subscriber sub_trigger_stamp_;

  // Services
  ros::ServiceServer srv_reset_;

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
  boost::array<double, 9ul> reference_frame_;
  bool write_to_flash_;
  bool factory_reset_before_start_;
  bool use_sensor_sync_;

  // Logging
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_;

public:
  VectorNavDriver(ros::NodeHandle & pnh);
  ~VectorNavDriver();
  void readParams(ros::NodeHandle & pnh);
  void verifyParams();
  void connectSensor();
  void setupSensor();
  void resetSensor();
  void stopSensor();
  bool resetServiceCallback(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res);
  void binaryAsyncMessageCallback(Packet & p, size_t index);
  void populateImuMsg(vn::sensors::CompositeData & cd, const ros::Time & time, bool filter);
  void populateMagMsg(vn::sensors::CompositeData & cd, const ros::Time & time, bool filter);
  void populateTempMsg(vn::sensors::CompositeData & cd, const ros::Time & time);
  void populatePresMsg(vn::sensors::CompositeData & cd, const ros::Time & time);
  void triggerStampCallback(const std_msgs::HeaderConstPtr msg);
  void getTriggerBasedStamp(
    const uint64_t time_sync_in, const uint32_t sync_in_cnt, ros::Time & stamp);
};
}  // namespace vectornav_driver
#endif  // VECTORNAV_DRIVER_VECTORNAV_DRIVER_HPP_
