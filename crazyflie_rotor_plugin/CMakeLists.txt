cmake_minimum_required(VERSION 3.5)
project(crazyflie_rotor_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

link_directories(${gazebo_ros_LIBRARY_DIRS})

add_library(crazyflie_rotor_plugin SHARED src/crazyflie_rotor_plugin.cpp)
ament_target_dependencies(crazyflie_rotor_plugin rclcpp std_msgs geometry_msgs gazebo_ros)

target_link_libraries(crazyflie_rotor_plugin ${gazebo_ros_LIBRARIES})

install(TARGETS crazyflie_rotor_plugin
  LIBRARY DESTINATION lib
)

ament_package() 