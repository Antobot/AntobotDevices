# AntobotDevices

AntobotDevices is a folder which defines the hardware compatible with Antobot systems, and creates a method for configuring multiple types of devices.

There are 4 types of device software included in this repository:
- GPS:
  - The main script to launch and manage all GPS code is gpsManager.py
  - Allows for many different GPS configurations, including:
    - Dual-GPS using an Antobot base station
    - 3 different methods of receiving correction messages (ublox PointPerfect MQTT, and NTRIP, and Antobot's own MQTT-based base station corrections)
- IMU
- LiDAR: 
- Camera: AntobotCamera is a submodule of AntobotDevices. Please read more inside that repository.




