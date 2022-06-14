// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ASYNC_CALLBACK_WRAPPER_HPP_
#define ASYNC_CALLBACK_WRAPPER_HPP_

#include "vectornav.hpp"

namespace vectornav
{
void callback_wrapper(void * user_data, Packet & p, size_t index)
{
  static_cast<VectorNav *>(user_data)->BinaryAsyncMessageCallback(p, index);
}
}  // namespace vectornav

#endif  // ASYNC_CALLBACK_WRAPPER_HPP_
