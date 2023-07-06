cmake_minimum_required(VERSION 3.0.2)
project(vectornav_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_srvs
)

find_package(spdlog REQUIRED)

add_subdirectory(vnproglib/cpp libvncxx)

catkin_package(
 INCLUDE_DIRS include vnproglib/cpp/include
 LIBRARIES vectornav_driver spdlog::spdlog
 CATKIN_DEPENDS roscpp sensor_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  vnproglib/cpp/include
  ${catkin_INCLUDE_DIRS}
)

add_library(vectornav_driver src/vectornav_driver.cpp)

add_executable(vectornav_driver_node src/vectornav_driver_node.cpp)

target_link_libraries(vectornav_driver_node
  vectornav_driver
  ${catkin_LIBRARIES}
  libvncxx
  spdlog::spdlog
)
