#!/usr/bin/env python3

# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     gpsManager is intended to launch and manage the appropriate GPS scripts and data.

# Contacts: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import yaml
import socket
import rospy

import roslaunch
import rospkg

from antobot_manager_software.launchManager import Node, RoslaunchWrapperObject
# from antobot_manager_msgs.srv import netMonitorLaunch, netMonitorLaunchResponse


class gpsManager():
    def __init__(self):

        self.gps_nodes = []

        # self._launch = roslaunch.scriptapi.ROSLaunch()
        # self._launch.start()

        gps_data,device_type = self.read_gps_config()

        if len(gps_data.items()) == 2: # Dual GPS setting
            for k, v in gps_data.items():
                if k == "urcu":
                    self.createLauncher("gps_movingbase.py", "gps_movingbase")
        else: # One gps antenna 
            for k, v in gps_data.items():
                node_name = "gps"
                if v['rtk_type'] == "ppp":
                    node_name += "_ppp"
                elif v['rtk_type'] == "base_station":
                    node_name += "base_station"
                elif v['rtk_type'] == "rtk_mqtt":
                    node_name += "_rtk_mqtt"

                exec_name = node_name + ".py" 

                print("launching executable {}".format(exec_name))

                # Launch the appropriate GPS script
                self.createLauncher(exec_name, node_name)

        return

    def read_gps_config(self):
        
        gps_data = None
        device_type = None

        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_manager_software')
        path = packagePath + "/config/software_config.yaml"
        
        with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            device_type = data['device_type']

        if device_type == "robot":
            print("Loading robot parameters!")
            rospack = rospkg.RosPack()
            packagePath=rospack.get_path('antobot_platform_robot')
            path = packagePath + "/config/robot_config.yaml"
        elif device_type == "tower":
            print("Loading sensor tower parameters!")
            rospack = rospkg.RosPack()
            packagePath=rospack.get_path('antobot_platform_tower')
            path = packagePath + "/config/tower_config.yaml"

        with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            gps_data = data['gps']
            # rtk_type = ???    # should launch all gps-related scripts from gpsManager instead

        return gps_data,device_type

    def check_gps(self):
        # Checks whether there have been any changes to the robot's network

        try:
            # Check whether any node has died

            # TODO: check RTK status of each GPS node

            # TODO: check frequency of each GPS node

            return
        except KeyboardInterrupt:
            exit()
        return
        
    
    def gps_status(self): 
        # Gets the gps frequency and RTK status of each of the GPS receivers. If any are having issues, the rest of the system should be informed.

        return
    

    def createLauncher(self, exec_name, node_name):
        '''Starts a camera by calling the c16 launch file'''

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['antobot_devices_gps', 'gps_config_launch.launch', 'exec_name:='+exec_name, 'node_name:='+node_name]

        
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        print(roslaunch_file)
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]

        launcher = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files)

        # start the launch file
        launcher.start_node_name(node_name)

        return launcher

def main():
    rospy.init_node ('gpsManager') 
    gpsMgr = gpsManager()
    
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        gpsMgr.check_gps()
        rate.sleep()


if __name__ == '__main__':
    main()
