// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// ROS version-specific includes
#if DETECTED_ROS_VERSION == 1
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <boost/array.hpp>

#else

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>

#endif

namespace ros_interface
{

#if DETECTED_ROS_VERSION == 1
// ROS1 type aliases
using NodeHandle = std::shared_ptr<ros::NodeHandle>;
using Time = ros::Time;
using Duration = ros::Duration;

// Publisher type alias (ROS1 publishers are not templated at the shared_ptr level)
template <typename MessageType>
using Publisher = std::shared_ptr<ros::Publisher>;

// Message type aliases
using StdMsgsHeader = std_msgs::Header;
using SensorMsgsImu = sensor_msgs::Imu;
using SensorMsgsMagneticField = sensor_msgs::MagneticField;
using SensorMsgsTemperature = sensor_msgs::Temperature;
using SensorMsgsFluidPressure = sensor_msgs::FluidPressure;

// Array type
template <typename T, std::size_t N>
using Array = boost::array<T, N>;

// Utility functions
template <typename MessageType>
Publisher<MessageType> create_publisher(
  NodeHandle & nh, const std::string & topic, int queue_size, bool latch = false)
{
  return std::make_shared<ros::Publisher>(nh->advertise<MessageType>(topic, queue_size, latch));
}

inline Time now(NodeHandle = {}) { return ros::Time::now(); }

// Time conversion utilities
inline double to_seconds(const Duration & d) { return d.toSec(); }

inline double to_seconds(const Time & t) { return t.toSec(); }

inline Time time_from_seconds(double secs)
{
  Time t;
  t.fromSec(secs);
  return t;
}

inline Time time_add(const Time & t, double secs) { return t + ros::Duration(secs); }

// Publisher utilities
template <typename T>
inline bool has_subscribers(const std::shared_ptr<T> & publisher)
{
  return publisher->getNumSubscribers() > 0;
}

// Service creation
using EmptySrvServer = std::shared_ptr<ros::ServiceServer>;

template <typename DriverT>
EmptySrvServer create_empty_service(
  NodeHandle & nh, const std::string & name,
  bool (DriverT::*callback)(std_srvs::EmptyRequest &, std_srvs::EmptyResponse &), DriverT * obj)
{
  return std::make_shared<ros::ServiceServer>(nh->advertiseService(name, callback, obj));
}

// Init, spin, shutdown
inline void init(int argc, char ** argv, const std::string & name) { ros::init(argc, argv, name); }

inline NodeHandle create_node_handle(const std::string &)
{
  return std::make_shared<ros::NodeHandle>("~");
}

inline std::string get_node_name(const NodeHandle &) { return ros::this_node::getName(); }

inline void spin(NodeHandle) { ros::spin(); }
inline void shutdown() {}

// Parameter reading helpers
inline void setMatrix(const XmlRpc::XmlRpcValue & value_array, boost::array<double, 9ul> & matrix)
{
  assert(value_array.getType() == XmlRpc::XmlRpcValue::TypeArray);
  assert(value_array.size() == 9);
  for (int i = 0; i < value_array.size(); i++) {
    std::ostringstream ostr;
    ostr << value_array[i];
    std::istringstream istr(ostr.str());
    istr >> matrix[i];
  }
}

#else
// ROS2 type aliases
using NodeHandle = std::shared_ptr<rclcpp::Node>;
using Time = rclcpp::Time;
using Duration = rclcpp::Duration;

// Publisher type alias (ROS2 publishers are templated)
template <typename MessageType>
using Publisher = std::shared_ptr<rclcpp::Publisher<MessageType> >;

// Message type aliases
using StdMsgsHeader = std_msgs::msg::Header;
using SensorMsgsImu = sensor_msgs::msg::Imu;
using SensorMsgsMagneticField = sensor_msgs::msg::MagneticField;
using SensorMsgsTemperature = sensor_msgs::msg::Temperature;
using SensorMsgsFluidPressure = sensor_msgs::msg::FluidPressure;

// Array type
template <typename T, std::size_t N>
using Array = std::array<T, N>;

// Utility functions
template <typename MessageType>
Publisher<MessageType> create_publisher(
  NodeHandle & node, const std::string & topic, int queue_size, bool latch = false)
{
  rclcpp::QoS qos(queue_size);
  if (latch) {
    qos = qos.transient_local();
  }
  const std::string full_topic = (topic.empty() || topic[0] == '/') ? topic : "~/" + topic;
  return node->template create_publisher<MessageType>(full_topic, qos);
}

inline Time now(NodeHandle & node) { return node->now(); }

// Time conversion utilities
inline double to_seconds(const Duration & d) { return d.seconds(); }

inline double to_seconds(const Time & t) { return t.seconds(); }

inline Time time_from_seconds(double secs)
{
  int32_t sec = static_cast<int32_t>(secs);
  uint32_t nanosec = static_cast<uint32_t>((secs - sec) * 1e9);
  return Time(sec, nanosec);
}

inline Time time_add(const Time & t, double secs)
{
  double total = t.seconds() + secs;
  return time_from_seconds(total);
}

// Publisher utilities
template <typename T>
inline bool has_subscribers(const std::shared_ptr<T> & publisher)
{
  return publisher->get_subscription_count() > 0;
}

// Service creation
using EmptySrvServer = rclcpp::Service<std_srvs::srv::Empty>::SharedPtr;

template <typename Callback>
EmptySrvServer create_empty_service(NodeHandle & node, const std::string & name, Callback callback)
{
  const std::string full_name = (name.empty() || name[0] == '/') ? name : "~/" + name;
  return node->create_service<std_srvs::srv::Empty>(full_name, callback);
}

// Init, spin, shutdown
inline void init(int argc, char ** argv, const std::string &) { rclcpp::init(argc, argv); }

inline NodeHandle create_node_handle(const std::string & name)
{
  return std::make_shared<rclcpp::Node>(name);
}

inline std::string get_node_name(const NodeHandle & node) { return node->get_name(); }

inline void spin(NodeHandle node) { rclcpp::spin(node); }
inline void shutdown() { rclcpp::shutdown(); }

// Parameter reading helpers
inline void setMatrix(const std::vector<double> & value_array, std::array<double, 9> & matrix)
{
  assert(value_array.size() == 9);
  for (size_t i = 0; i < value_array.size(); i++) {
    matrix[i] = value_array[i];
  }
}

#endif

}  // namespace ros_interface

namespace ri = ros_interface;
