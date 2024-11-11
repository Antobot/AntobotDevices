cmake_minimum_required(VERSION 3.0.2)
project(control_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  Ready.srv
  Start.srv
  Stop.srv
  Config.srv
  Restart.srv
  Sleep.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS sensor_msgs std_msgs nav_msgs geometry_msgs message_generation rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
