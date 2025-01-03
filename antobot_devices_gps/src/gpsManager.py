#!/usr/bin/env python3

# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     gpsManager is intended to launch and manage the appropriate GPS scripts and data.

# Contacts: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import yaml
import socket
import serial

import rospy

import roslaunch
import rospkg

from antobot_manager_software.launchManager import Node, RoslaunchWrapperObject
# from antobot_manager_msgs.srv import netMonitorLaunch, netMonitorLaunchResponse

from gps_f9p import F9P_GPS
from gps_movingbase import MovingBase_Ros
from gps_corrections import gpsCorrections


class gpsManager():
    def __init__(self):

        launch_nodes = False
        use_class = True

        self.gps_nodes = []
        self.dual_gps = 'false'

        # Create serial interface placeholder
        self.f9p_usb_port = None
        self.f9p_urcu_serial_port = None

        # Read gps config
        gps_data, device_type = self.read_gps_config()

        # Launch GPS nodes
        for k, v in gps_data.items():
            if launch_nodes:
                # Launch the appropriate GPS script
                exec_name, node_name = self.get_node_name(k, v)
                self.createLauncher(exec_name, node_name)
                print("launching executable {}".format(exec_name))
            elif use_class:
                
                gps_cls_tmp = self.get_gps_class(k, v)
                self.gps_nodes.append(gps_cls_tmp)

        # Launch corrections node
        

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
            packagePath=rospack.get_path('antobot_description')
            path = packagePath + "/config/platform_config.yaml"
        elif device_type == "tower":
            print("Loading sensor tower parameters!")
            rospack = rospkg.RosPack()
            packagePath=rospack.get_path('antobot_description')
            path = packagePath + "/config/platform_config.yaml"

        with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            gps_data = data['gps']
            # rtk_type = ???    # should launch all gps-related scripts from gpsManager instead

        return gps_data, device_type

    def get_node_name(self, k, v):

        node_name = "gps_" + k

        if k == "urcu":
            exec_name = "gps_f9p.py"
        if k == "movingbase":
            exec_name = "gps_movingbase.py"
            # May need to also launch a separate node (movingbase_enu_conversion)
        if k == "f9p_usb" or k == "f9p_usb2":
            exec_name = "gps_f9p.py"

        return exec_name, node_name
    
    def createLauncher(self, exec_name, node_name):

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['antobot_devices_gps', 'gps_config_launch.launch', 'exec_name:='+exec_name, 'node_name:='+node_name, 'dual_gps:='+self.dual_gps]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        print(roslaunch_file)
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]

        launcher = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files)

        # start the launch file
        launcher.start_node_name(node_name)

        return launcher
    
    def get_gps_class(self, k, v):

        if k == "urcu":
            serial = None
            gps_cls = F9P_GPS("urcu")
        if k == "movingbase":
            serial = None
            gps_cls = MovingBase_Ros()
            # May need to also launch a separate node (movingbase_enu_conversion)
        if k == "f9p_usb" or k == "f9p_usb2":
            baud = 460800
            if self.f9p_usb_port == None:
                self.f9p_usb_port = serial.Serial(v['device_port'], baud)
            gps_cls = F9P_GPS("usb", serial_port=self.f9p_usb_port, pub_name="antobot_" + k)

        return gps_cls
    
    def check_gps(self):
        # Checks whether there have been any changes to the robot's network

        for gps_node in self.gps_nodes:
            if gps_node.node_type == "gps_f9p":
                self.check_gps_node(gps_node)

        return
    
    def check_gps_node(self, gps_node):
        # If no longer running, re-launch

        gps_node.get_gps()

        # Publish any changes to fixed status, frequency, etc.

        return
    

def main():
    rospy.init_node ('gpsManager') 
    gpsMgr = gpsManager()

    rate = rospy.Rate(50)  # check at 50hz
    # rate = rospy.Rate(1) # check at 1hz

    while not rospy.is_shutdown():
        gpsMgr.check_gps()
        rate.sleep()


if __name__ == '__main__':
    main()
