cmake_minimum_required(VERSION 3.8)
project(antobot_devices_orbbec_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(multi_orbbec_image_saver src/orbbec_image_saver_node.cpp)
target_compile_features(multi_orbbec_image_saver PUBLIC cxx_std_17)
ament_target_dependencies(multi_orbbec_image_saver
  cv_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(multi_orbbec_image_saver
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS multi_orbbec_image_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
