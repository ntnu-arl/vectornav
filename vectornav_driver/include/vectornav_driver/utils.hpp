// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef VECTORNAV_DRIVER_UTILS_HPP_
#define VECTORNAV_DRIVER_UTILS_HPP_

// vnproglib
#include "vn/vector.h"

// ROS version-specific includes
#if DETECTED_ROS_VERSION == 1
#include <ros/ros.h>
#include <boost/array.hpp>
#else
#include <rclcpp/rclcpp.hpp>
#include <array>
#endif

namespace vectornav_driver
{
#if DETECTED_ROS_VERSION == 1
inline void setMatrix(const XmlRpc::XmlRpcValue & value_array, boost::array<double, 9ul> & matrix)
{
  assert(value_array.getType() == XmlRpc::XmlRpcValue::TypeArray);
  assert(value_array.size() == 9);
  for (int i = 0; i < value_array.size(); i++) {
    // Use string stream here to avoid issues with parsing with decimal points
    std::ostringstream ostr;
    ostr << value_array[i];
    std::istringstream istr(ostr.str());
    istr >> matrix[i];
  }
}
#else
inline void setMatrix(const std::vector<double> & value_array, std::array<double, 9> & matrix)
{
  assert(value_array.size() == 9);
  for (size_t i = 0; i < value_array.size(); i++) {
    matrix[i] = value_array[i];
  }
}
#endif

#if DETECTED_ROS_VERSION == 1
inline void setDiagonal(double value, boost::array<double, 9ul> & matrix)
#else
inline void setDiagonal(double value, std::array<double, 9> & matrix)
#endif
{
  matrix[0] = value;
  matrix[4] = value;
  matrix[8] = value;
}

inline vn::math::vec3f convert(const std::array<float, 3> & arr)
{
  return vn::math::vec3f(arr[0], arr[1], arr[2]);
}

template <typename T>
inline bool hasSubscribers(const std::shared_ptr<T> & publisher)
{
#if DETECTED_ROS_VERSION == 1
  return publisher->getNumSubscribers() > 0;
#else
  return publisher->get_subscription_count() > 0;
#endif
}

#if DETECTED_ROS_VERSION == 1
inline vn::math::mat3f convert(boost::array<double, 9ul> & mat_b)
#else
inline vn::math::mat3f convert(std::array<double, 9> & mat_b)
#endif
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
