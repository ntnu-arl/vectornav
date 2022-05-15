cmake_minimum_required(VERSION 3.0.2)
project(vectornav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(spdlog REQUIRED)

add_subdirectory(vnproglib/cpp libvncxx)

catkin_package(
 INCLUDE_DIRS include vnproglib/cpp/include
 LIBRARIES vectornav spdlog::spdlog
 CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  vnproglib/cpp/include
  ${catkin_INCLUDE_DIRS}
)

add_library(vectornav src/vectornav.cpp)

add_executable(vectornav_node src/vectornav_node.cpp)

target_link_libraries(vectornav_node
  vectornav
  ${catkin_LIBRARIES}
  libvncxx
  spdlog::spdlog
)
