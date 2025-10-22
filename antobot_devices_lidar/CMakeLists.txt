cmake_minimum_required(VERSION 3.8)
project(antobot_devices_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(antobot_devices_lidar_convert src/aviaMsg2pointcloud2.cpp)
ament_target_dependencies(antobot_devices_lidar_convert 
rclcpp  sensor_msgs livox_ros_driver2 pcl_conversions geometry_msgs)

target_link_libraries(antobot_devices_lidar_convert
  ${PCL_LIBRARIES}
        yaml-cpp
)

install(TARGETS
        antobot_devices_lidar_convert
        DESTINATION lib/${PROJECT_NAME}
)

# === Python integration ===
ament_python_install_package(${PROJECT_NAME})  # calls setup.py

install(PROGRAMS
        antobot_devices_lidar/lidar_manager.py
        antobot_devices_lidar/lidarManagerClient.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()


