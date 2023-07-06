// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "vectornav_driver/vectornav_driver.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "vectornav_driver_node");
  ros::NodeHandle pnh("~");
  vectornav_driver::VectorNavDriver vnd(pnh);
  vnd.setupSensor();
  ros::spin();
  vnd.stopSensor();
  return 0;
}
