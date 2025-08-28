// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "vectornav_driver/vectornav_driver.hpp"

int main(int argc, char ** argv)
{
#if ROS_VERSION == 1
  ros::init(argc, argv, "vectornav_driver_node");
  auto node = std::make_shared<ros::NodeHandle>("~");
  auto vnd = std::make_shared<vectornav_driver::VectorNavDriver>(node);
  vnd->setupSensor();
  ros::spin();
  vnd->stopSensor();
#else
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vectornav_driver_node");
  auto vnd = std::make_shared<vectornav_driver::VectorNavDriver>(node);
  vnd->setupSensor();
  rclcpp::spin(node);
  vnd->stopSensor();
  rclcpp::shutdown();
#endif
  return 0;
}
