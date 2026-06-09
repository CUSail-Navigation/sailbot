cmake_minimum_required(VERSION 3.22.1)
project(cpp_files)
set(CMAKE_CXX_STANDARD 20)

# Find required packages: ROS 2 core ones, and custom messages.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sailboat_interface REQUIRED)

add_executable(cpp_files
        JANK.cpp
)

# Handle the dependencies for all the above packages.
ament_target_dependencies(cpp_files
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        sailboat_interface
)

install(TARGETS cpp_files
  DESTINATION lib/${PROJECT_NAME})

ament_package()