#ifndef SUPPORTED_SETS_HPP_
#define SUPPORTED_SETS_HPP_

#include <set>
#include <stdint.h>

namespace vectornav
{
  static std::set<uint32_t> BAUD_RATES = {9600,   19200,  38400,  57600, 115200,
                                                    128000, 230400, 460800, 921600};
  static std::set<uint8_t> ASYNC_PORTS = {1, 2};

}  // namespace vectornav

#endif  // SUPPORTED_SETS_HPP_