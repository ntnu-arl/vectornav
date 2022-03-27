// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

#include "vectornav/vectornav.hpp"
#include "vectornav/async_callback_wrapper.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vectornav_node");
  ros::NodeHandle pnh("~");
  vectornav::Config config;
  vectornav::VectorNav vn(config_utilities::getConfigFromRos<vectornav::Config>(pnh), pnh);
  vn.SetupSensor();
  vn.SetupAsyncMessageCallback(vectornav::callback_wrapper);
  ros::spin();
  vn.StopSensor();
  return 0;
}
