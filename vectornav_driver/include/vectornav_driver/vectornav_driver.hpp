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
#include "vectornav_driver/ros_interface.hpp"
#include "vectornav_driver/utils.hpp"

// spdlog
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace vn::protocol::uart;

namespace vectornav_driver
{

class VectorNavDriver
{
private:
  // Members
  vn::sensors::VnSensor sensor_;
  // Member messages
  ri::SensorMsgsImu filter_data_msg_;
  ri::SensorMsgsImu imu_data_msg_;
  ri::SensorMsgsMagneticField filter_mag_msg_;
  ri::SensorMsgsMagneticField imu_mag_msg_;
  ri::SensorMsgsFluidPressure pressure_msg_;
  ri::SensorMsgsTemperature temperature_msg_;

  ri::Time ros_start_time_;
  double average_time_difference_ = 0.0;

  // Node handle (unified for both ROS1 and ROS2)
  ri::NodeHandle node_;

  // Publishers (properly typed for each message type)
  ri::Publisher<ri::SensorMsgsImu> pub_filter_data_;
  ri::Publisher<ri::SensorMsgsImu> pub_imu_data_;
  ri::Publisher<ri::SensorMsgsMagneticField> pub_filter_mag_;
  ri::Publisher<ri::SensorMsgsMagneticField> pub_imu_mag_;
  ri::Publisher<ri::SensorMsgsFluidPressure> pub_pressure_;
  ri::Publisher<ri::SensorMsgsTemperature> pub_temperature_;
  ri::Publisher<ri::StdMsgsHeader> pub_sync_out_stamp_;

  // Services
  ri::EmptySrvServer srv_reset_;

  // Parameters
  vn::sensors::VnSensor::Family sensor_family_;
  std::string port_;
  uint32_t baud_rate_;
  uint16_t async_mode_;
  uint16_t async_rate_divisor_;
  bool adjust_timestamp_;
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
  ri::Array<double, 9> reference_frame_;
  bool write_to_flash_;
  bool factory_reset_before_start_;

  // Logging
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_;

public:
  VectorNavDriver(ri::NodeHandle node);
  ~VectorNavDriver();

  void readParams();
  void verifyParams();
  void connectSensor();
  void setupSensor();
  void resetSensor();
  void stopSensor();

#if DETECTED_ROS_VERSION == 1
  bool resetServiceCallback(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res);
#else
  void resetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
#endif

  void binaryAsyncMessageCallback(Packet & p, size_t index);
  ri::Time getTime(vn::sensors::CompositeData & cd, const ri::Time & ros_time);
  void populateImuMsg(vn::sensors::CompositeData & cd, const ri::Time & time, bool filter);
  void populateMagMsg(vn::sensors::CompositeData & cd, const ri::Time & time, bool filter);
  void populateTempMsg(vn::sensors::CompositeData & cd, const ri::Time & time);
  void populatePresMsg(vn::sensors::CompositeData & cd, const ri::Time & time);
};
}  // namespace vectornav_driver
#endif  // VECTORNAV_DRIVER_VECTORNAV_DRIVER_HPP_
