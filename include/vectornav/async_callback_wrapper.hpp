#ifndef ASYNC_CALLBACK_WRAPPER_HPP_
#define ASYNC_CALLBACK_WRAPPER_HPP_

#include "vectornav.hpp"

namespace vectornav
{
  void callback_wrapper(void* user_data, Packet& p, size_t index)
  {
    static_cast<VectorNav *>(user_data)->BinaryAsyncMessageCallback(p, index);
  }
} // namespace vectornav

#endif // ASYNC_CALLBACK_WRAPPER_HPP_