cmake_minimum_required(VERSION 3.22.1)
project(jank_algo)

set(CMAKE_CXX_STANDARD 20)

# Find the ROS 2 core packages.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(jank_algo
        JANK.cpp
)

ament_target_dependencies(jank_algo
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
)

# Link the ROS 2 libraries to the executable.
target_link_libraries(jank_algo
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
)

install(TARGETS jank_algo
  DESTINATION lib/${PROJECT_NAME})

ament_package()