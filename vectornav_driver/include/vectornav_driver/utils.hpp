// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef VECTORNAV_DRIVER_UTILS_HPP_
#define VECTORNAV_DRIVER_UTILS_HPP_

// vnproglib
#include "vn/vector.h"

// STL
#include <array>

namespace vectornav_driver
{
inline void setDiagonal(double value, std::array<double, 9> & matrix)
{
  matrix[0] = value;
  matrix[4] = value;
  matrix[8] = value;
}

inline vn::math::vec3f convert(const std::array<float, 3> & arr)
{
  return vn::math::vec3f(arr[0], arr[1], arr[2]);
}

inline vn::math::mat3f convert(std::array<double, 9> & mat_b)
{
  vn::math::mat3f mat;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat(i, j) = static_cast<float>(mat_b[i * 3 + j]);
    }
  }
  return mat;
}

}  // namespace vectornav_driver

#endif  // VECTORNAV_DRIVER_UTILS_HPP_
