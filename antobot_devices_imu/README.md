## antobot_devices_imu

The package to use and manage IMU devices on Antobot's robot platform

### config

- Different IMU devices and their relative location to the robot should be configured in the antobot_description package (platform_config.yaml)

### launch
- imu.launch.py - launches the configured IMU driver
- static_lidar_frame.launch.py - used if you do not want to use the IMU data to compensate the costmap calculation
- static_lidar_frame_two_lidars.launch.py - same as above, but required if using 2 LiDARs

### src
- imuManager.py - the main script to launch and manage all IMU code
- imu_compensated.py - orientation estimation with adaptive drift compensation
