// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef VECTORNAV_HPP_
#define VECTORNAV_HPP_

// VectorNav Driver
#include "vn/compositedata.h"
#include "vn/sensors.h"
#include "vn/vector.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Empty.h>

// spdlog
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace vn::protocol::uart;

namespace vectornav
{
class VectorNav
{
private:
  vn::sensors::VnSensor sensor_;

  // Publishers
  ros::Publisher pub_imu_;
  ros::Publisher pub_uncomp_imu_;
  ros::Publisher pub_mag_;
  ros::Publisher pub_uncomp_mag_;
  ros::Publisher pub_pres_;
  ros::Publisher pub_temp_;
  ros::Publisher pub_sync_out_stamp_;

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
  bool publish_uncomp_imu_;
  bool publish_uncomp_mag_;
  std::string frame_id_;
  float temp_variance_;
  float pres_variance_;
  std::string log_directory_;
  spdlog::level::level_enum log_level_;
  boost::array<double, 9ul> linear_accel_covariance_;
  boost::array<double, 9ul> angular_vel_covariance_;
  boost::array<double, 9ul> orientation_covariance_;
  boost::array<double, 9ul> mag_covariance_;
  std::array<float, 3> mag_ref_;
  std::array<float, 3> gravity_ref_;
  bool write_to_flash_;
  bool factory_reset_before_start_;

  // Logging
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_;

  // Flags
  bool receiving_data_;

public:
  VectorNav(ros::NodeHandle & pnh);
  ~VectorNav();
  void ReadParams(ros::NodeHandle & pnh);
  void SetCovarianceMatrix(
    const XmlRpc::XmlRpcValue & covariance_list, boost::array<double, 9ul> & covariance_matrix);
  void SetArray(const XmlRpc::XmlRpcValue & values, std::array<float, 3> & arr);
  void VerifyParams();
  void ConnectSensor();
  void DisconnectSensor();
  void SetupSensor();
  void ResetSensor();
  void StopSensor();
  bool ResetServiceCallback(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res);
  void SetupAsyncMessageCallback(vn::sensors::VnSensor::AsyncPacketReceivedHandler handler);
  void BinaryAsyncMessageCallback(Packet & p, size_t index);
  const ros::Time CorrectTimestamp(
    const ros::Time & arrival_stamp, uint64_t startup_time, uint64_t sync_in_time);
  void PrintSensorInfo();
  void PopulateImuMsg(
    sensor_msgs::Imu & msg, vn::sensors::CompositeData & cd, const ros::Time & time, bool uncomp);
  void PopulateMagMsg(
    sensor_msgs::MagneticField & msg, vn::sensors::CompositeData & cd, const ros::Time & time,
    bool uncomp);
  void PopulateTempMsg(
    sensor_msgs::Temperature & msg, vn::sensors::CompositeData & cd, const ros::Time & time);
  void PopulatePresMsg(
    sensor_msgs::FluidPressure & msg, vn::sensors::CompositeData & cd, const ros::Time & time);
  vn::math::vec3f Convert(const std::array<float, 3> & arr);
};
}  // namespace vectornav
#endif  // VECTORNAV_HPP_