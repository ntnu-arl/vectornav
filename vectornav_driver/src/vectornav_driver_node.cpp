// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "vectornav_driver/vectornav_driver.hpp"

int main(int argc, char ** argv)
{
  ri::init(argc, argv, "vectornav_driver_node");
  auto pnh = ri::create_node_handle("vectornav_driver_node");
  vectornav_driver::VectorNavDriver vnd(pnh);
  vnd.setupSensor();
  ri::spin(pnh);
  vnd.stopSensor();
  ri::shutdown();
  return 0;
}
