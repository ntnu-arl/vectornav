cmake_minimum_required(VERSION 3.8)
project(vectornav_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect ROS version
find_package(roscpp QUIET)
if(roscpp_FOUND)
  set(DETECTED_ROS_VERSION 1)
  message(STATUS "Building with ROS1")
else()
  set(DETECTED_ROS_VERSION 2)
  message(STATUS "Building with ROS2")
endif()

add_definitions(-DDETECTED_ROS_VERSION=${DETECTED_ROS_VERSION})

# Find common dependencies
find_package(spdlog REQUIRED)
add_subdirectory(vnproglib/cpp libvncxx)

# ROS-specific dependencies and configuration
if(DETECTED_ROS_VERSION EQUAL 1)
  # ROS1 setup
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_srvs
  )

  catkin_package(
    INCLUDE_DIRS include vnproglib/cpp/include
    LIBRARIES vectornav_driver
    CATKIN_DEPENDS roscpp sensor_msgs std_srvs
  )

  set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
  set(ROS_LIBRARIES ${catkin_LIBRARIES})

else()
  # ROS2 setup
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(std_msgs REQUIRED)

  set(ROS2_DEPENDENCIES
    rclcpp
    sensor_msgs
    std_srvs
    std_msgs
  )
endif()

# Include directories
include_directories(
  include
  vnproglib/cpp/include
  ${ROS_INCLUDE_DIRS}
)

# Build library
add_library(vectornav_driver src/vectornav_driver.cpp)

# Build executable
add_executable(vectornav_driver_node src/vectornav_driver_node.cpp)

# Link libraries and set dependencies
if(DETECTED_ROS_VERSION EQUAL 1)
  target_link_libraries(vectornav_driver_node
    vectornav_driver
    libvncxx
    spdlog::spdlog
    ${ROS_LIBRARIES}
  )
else()
  ament_target_dependencies(vectornav_driver ${ROS2_DEPENDENCIES})
  ament_target_dependencies(vectornav_driver_node ${ROS2_DEPENDENCIES})

  target_link_libraries(vectornav_driver_node
    vectornav_driver
    libvncxx
    spdlog::spdlog
  )

  # Install targets
  install(TARGETS vectornav_driver_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}/
  )

  ament_package()
endif()
