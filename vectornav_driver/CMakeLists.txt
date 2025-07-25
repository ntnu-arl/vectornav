cmake_minimum_required(VERSION 3.8)
project(vectornav_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(spdlog REQUIRED)

add_subdirectory(vnproglib/cpp libvncxx)

include_directories(
  include
  vnproglib/cpp/include
)

add_library(vectornav_driver src/vectornav_driver.cpp)

add_executable(vectornav_driver_node src/vectornav_driver_node.cpp)

ament_target_dependencies(vectornav_driver
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

ament_target_dependencies(vectornav_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

target_link_libraries(vectornav_driver_node
  vectornav_driver
  libvncxx
  spdlog::spdlog
)

install(TARGETS
  vectornav_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
