cmake_minimum_required(VERSION 3.0.2)
project(antobot_devices_gps)

find_package(catkin REQUIRED COMPONENTS
  rclpy
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

