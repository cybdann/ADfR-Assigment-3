cmake_minimum_required(VERSION 3.8)
project(cam_capture)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(asdfr_interfaces REQUIRED) 

include_directories(include)

add_executable(brightness_level_detector src/bld_node.cpp src/brightness_level_detector.cpp)
ament_target_dependencies(brightness_level_detector rclcpp std_msgs sensor_msgs asdfr_interfaces)

add_executable(light_position_indicator src/lpi_node.cpp src/light_position_indicator.cpp)
ament_target_dependencies(light_position_indicator rclcpp std_msgs sensor_msgs OpenCV cv_bridge asdfr_interfaces)

install(TARGETS
  brightness_level_detector
  light_position_indicator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
