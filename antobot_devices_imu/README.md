## antobot_devices_imu

The package to use and manage IMU devices on Antobot's robot platform

### config

- Different IMU devices and their relative location to the robot should be configured in the antobot_description package (platform_config.yaml)

### launch
- imu_euler.launch - used to compensate the costmap calculation based on IMU data
- static_lidar_frame.launch - used if you do not want to use the IMU data to compensate the costmap calculation
- static_lidar_frame_two_lidars.launch - same as above, but required if using 2 LiDARs

### src
- imuManager.py - the main script to launch and manage all GPS code
- imu_euler.py - calculates a new angle to calculate the costmap from the LiDAR data based on IMU input 
- imu_euler.cpp - C++ optimised version of imu_euler.py (preferred)









