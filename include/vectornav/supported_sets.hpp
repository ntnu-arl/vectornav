// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

#ifndef SUPPORTED_SETS_HPP_
#define SUPPORTED_SETS_HPP_

#include <set>
#include <stdint.h>

namespace vectornav
{
  static std::set<uint32_t> BAUD_RATES = {9600,   19200,  38400,  57600, 115200,
                                          128000, 230400, 460800, 921600};
}  // namespace vectornav

#endif  // SUPPORTED_SETS_HPP_