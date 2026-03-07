cmake_minimum_required(VERSION 3.22.1)
project(JANKAlgoDrafting)

set(CMAKE_CXX_STANDARD 20)


# Lines I added: find the ROS 2 core packages.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(JANKAlgoDrafting
        JANK.cpp
        # ROSTests/ROSTest.cpp
        # ROSTests/ROSTest2.cpp
)

ament_target_dependencies(JANKAlgoDrafting
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
)

# Lines I added: link the ROS 2 libraries to the executable.
target_link_libraries(JANKAlgoDrafting
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    # std_msgs::std_msgs__rosidl_typesupport_cpp # For ROSTest2.
)