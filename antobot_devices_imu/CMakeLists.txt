cmake_minimum_required(VERSION 3.0.2)
project(antobot_devices_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs 
  tf
  geonav_transform
  tf2
  tf2_ros
  std_msgs
  rospy
)

catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf geonav_transform tf2 tf2_ros std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_euler_cpp src/imu_euler_cpp.cpp)
target_link_libraries(imu_euler_cpp ${catkin_LIBRARIES})
