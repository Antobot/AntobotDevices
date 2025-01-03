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
import asyncio

import rospy

import roslaunch
import rospkg

from antobot_manager_software.launchManager import Node, RoslaunchWrapperObject
# from antobot_manager_msgs.srv import netMonitorLaunch, netMonitorLaunchResponse

from gps_f9p import F9P_GPS
from gps_movingbase import MovingBase_Ros
from gps_corrections import gpsCorrections

import Jetson.GPIO as GPIO


class gpsManager():
    def __init__(self):

        launch_nodes = False
        use_class = True

        self.gps_nodes = []
        self.dual_gps = 'false'
        self.movingbase = False
        self.urcu_gps_node = None

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
        # # # STILL NEEDS TO BE DONE!!!

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
            # Define serial port for the F9P inside of the uRCU (for movingbase or otherwise)
            if self.f9p_urcu_serial_port == None:
                self.f9p_urcu_serial_port = serial.Serial(v['device_port'], baud)

            # Define the class object
            gps_cls = F9P_GPS("urcu")
            self.urcu_gps_node = gps_cls

        if k == "movingbase":
            # Define serial port for the movingbase F9P (connected via USB)
            if self.f9p_usb_port == None:
                self.f9p_usb_port = serial.Serial(v['device_port'], baud)

            # Define the class object
            gps_cls = MovingBase_Ros(self.f9p_urcu_serial_port, self.f9p_usb_port, None)
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
    
    async def check_gps_async(self):

        MB = await self.create_MovingBase()
        while not rospy.is_shutdown():
            try:

                self.check_gps_node(self.urcu_gps_node)

                headFrame = await MB.get_RELPOSNEDframe()
                self.pub_head(headFrame)

            except:
                rospy.logerr(f"MovingBase: Close the MovingBase node")
                break
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

    if gpsMgr.movingbase:
        loop = asyncio.get_event_loop()
        try:
            loop.run_until_complete(gpsMgr.check_gps_async)
        except Exception as e:     
            GPIO.cleanup()
            loop.close()


if __name__ == '__main__':
    main()
