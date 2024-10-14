# antobot_manager_devices

antobot_manager_devices is a folder which defines the hardware compatible with Antobot systems, and creates a method for configuring multiple types of devices.

There are 3 types of devices in the folder:
- Main Device Type: the main device needs to know a variety of information about itself in order to launch and manage its software, and thus each of these need individual configuration files, with examples provided. The 3 types of computing devices can be categorised as:
  - robot: several types of robots can be defined, both based on the Ant platform and otherwise. Can be either simulated or hardware, which will be handled accordingly.
  - tower: a sensor tower, which can be flexibly configured with many types of different sensors in relative poses to one another.
  - scout: a scouting module, sort of a simple sensor tower with cameras as the only potential sensor type

- Sensing:
  - Sensing devices include things like GPS, cameras, IMU, LiDAR, etc.
  - Each type of sensing device should have its own manager, which should consider different models, interfaces, etc. for that sensor type. The manager should bein charge of launching all of the software for that type of sensor, monitoring its data, and managing any low-level changes which occur or need to occur related to that sensor type
  - Individual sensing devices should not be configured in their folders, as these can be considered at the "Computing" level

- Computing: urcu and hmi
  - these devices are computing units so have hardware-specific code that cannot be run if the hardware is not there




