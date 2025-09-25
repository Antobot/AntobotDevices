#! /usr/bin/env python3

# Copyright (c) 2022, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     This code manages the different lidars on Antobot's robots. It has a service (/anto_manager/lidar)
#                           that can be called for turning on/off the lidar and enable/distable the lidar input in the costmap.

# Contacts: soyoung.kim@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from pathlib import Path
from datetime import datetime
import os
from launch import LaunchService
from launch_ros import get_default_launch_description
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

import roslaunch # Using this until we develop our own camera manager solution
import rospkg

from antobot_manager_software.launchManager import ProcessListener, RoslaunchWrapperObject

from antobot_devices_msgs.srv import lidarManager, lidarManagerResponse
from antobot_devices_msgs.srv import costmapToggleObservation, costmapToggleObservationRequest
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


###################################################################################################################################################

###################################################################################################################################################
class lidar(Node): # lidar pare
    '''Stores lidar specific information'''
    def __init__(self,ip, location="front"):

        # Save arguments
        self.ip = ip
        self.active = False # Lidar turn on/off 
        self.enabled_in_costmap = True # default True
        self.launch = None
        self.location = location
        self.type = ""

        self.launch_file_path = os.path.join(get_package_share_directory('antobot_devices_lidar'),'launch',self.type+"_launch.py")

        
        ## Client for controlling lidar input to the costmap node
        self.costmapToggleClient = self.create_client(costmapToggleObservation,'/costmap_node/costmap/toggle_observation')
           


#################################################
    def run(self):
        ##############################################################################
        ## Check for the /ros_gz_bridge node to check whether we need to launch lidar driver node    
        ##############################################################################
        node_names = self.get_node_names()
        if '/ros_gz_bridge' in node_names:
            self.sim = True
        else:
            self.sim = False

        if self.sim:
            self.get_logger().info('SW2320: Lidar Manager: Simulation - robot lidar launched with Gazebo')
            self.active = True
        else:
            # By default, turn on all the available lidar 
            self.createLauncher()
            self.start()

#################################################

    def createLauncher(self):
        '''Starts a camera by calling the c16 launch file'''

        # pass # To be implemented in the child class
        
#################################################

    def start(self):
        if self.active==False:
            self.active=True
            self.launch.start()

#################################################

    def shutdown(self):
        if self.active==True:
            self.active=False
            self.launch.stop()
            self.get_logger().info("SW2320: Lidar Manager: Lidar {} shutdown".format(self.location))

        else:
            self.get_logger().info("SW2320: Lidar Manager: Shutdown requested but no active lidar")

#################################################
    
    def disableInCostmap(self):
        # disable this input from the costmap
        return_msg = lidarManagerResponse()
        if self.active and self.enabled_in_costmap:
            topic_name = "/lidar_"+self.location+"/lslidar_point_cloud"
            req = costmapToggleObservationRequest(observation_source = topic_name,command = False)
            response = self.costmapToggleClient.call(req) # Always return true 
            return_msg.responseCode = True
            return_msg.responseString = topic_name + " disabled in the costmap"
            self.enabled_in_costmap = False
        elif not self.active:
            return_msg.responseCode = False
            return_msg.responseString = "lidar not turned off - no disabling the topic in the costmap"
        elif not self.enabled_in_costmap:
            return_msg.responseCode = True # True since it's already disabled
            return_msg.responseString = "topic already disabled in the costmap"
        else:
            # This Shouldn't happen
            self.get_logger().info("SW2320: Lidar Manager: Lidar topic is neither active nor enabled in costmap - failed to disable the topic")
            return_msg.responseCode = False
            return_msg.responseString = "Failed to disable the topic"
        
        return return_msg
    
#################################################

    def enableInCostmap(self):
        return_msg = lidarManagerResponse()
        if self.active and not self.enabled_in_costmap:
            topic_name = "/lidar_"+self.location+"/lslidar_point_cloud"
            req = costmapToggleObservationRequest(observation_source = topic_name,command = True)
            response = self.costmapToggleClient.call(req) # Always return true 
            return_msg.responseCode = True
            return_msg.responseString = topic_name + " enabled in the costmap"
            self.enabled_in_costmap = True
        elif not self.active:
            return_msg.responseCode = False
            return_msg.responseString = "lidar not turned off - no enabling the topic in the costmap"
        elif self.enabled_in_costmap:
            return_msg.responseCode = True # True since it's already enabled
            return_msg.responseString = "topic already enabled in the costmap"
        else:
            self.get_logger().info("SW2320: Lidar Manager: Lidar topic is alrady active and enabled in costmap")
            return_msg.responseCode = False
            return_msg.responseString = "Failed to enable the topic"

        return return_msg

class lidar_c16(lidar):
    '''Stores lidar specific information'''
    def __init__(self,name ="c16",ip="",m_port="",d_port="",frame_id="",location="front"):
        super().__init__(ip, frame_id, location)
        self.type = 'cx' # Can support C16, C32
        # Save arguments
        self.m_port = str(m_port)
        self.d_port = str(d_port)
        self.frame_id = frame_id
        self.name_space = name

        super().run() # run the lidar (create launcher and start if not simulation)
#################################################

    def createLauncher(self):
        '''Starts a camera by calling the c16 launch file'''

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        

        self.get_logger().info("SW2320: Lidar Manager: Creating launcher for front lidar")
        cli_args = ['antobot_devices_lidar', 'lslidar_config_launch_updated.launch', 'name_space:='+self.name_space, 'frame_id:='+self.frame_id,  'device_ip:='+self.ip, 'msop_port:='+self.m_port, 'difop_port:='+self.d_port]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        #print(roslaunch_file)
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]

        self.launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files)
    

###################################################################################################################################################


class lidar_mid360(lidar):
    '''Stores lidar specific information'''
    def __init__(self,name ="mid360",ip="",frame_id="",location="front"):
        super().__init__(ip, location)
        self.type = 'mid360'

        # Save arguments
        # TODO: mid360 port settings
        self.frame_id = frame_id
        self.name_space = name

        super().run() # run the lidar (create launcher and start if not simulation)
#################################################

    def createLauncher(self):
        '''Starts a camera by calling the c16 launch file'''

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['antobot_devices_lidar', 'mid360_config_launch.launch', 'frame_id:='+self.frame_id]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]

        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]

        self.launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files)
    

###################################################################################################################################################


class lidarManagerClass(Node):
    # # # The lidarManager class reads the intended configuration for a particular device, then launches
    # # # and monitors the software for each individual lidar.

    def __init__(self):

        # Read robot_config file
        self.for_navigation = False
        self.read_config_file()

        ##############################################################################
        ## Check for the simulation parameter which should be set by am_sim     
        ##############################################################################
        node_names = self.get_node_names()
        if '/ros_gz_bridge' in node_names:
            self.sim = True
        else:
            self.sim = False

        ##############################################################################
        ## Run either real or simulated manager    
        ##############################################################################

        if self.sim:
            self.get_logger().info("SW2320: Lidar Manager: Running in simulation mode")
        else:
            self.get_logger().info("SW2320: Lidar Manager: This is NOT a simulation - using real Lidar")

        # Create a service to allow other nodes to start/stop lidars
        self.srvlidarMgr = self.create_service(lidarManager,"/antobot/lidarManager",self._serviceCallbackLidarMgr)


#################################################
    
    def read_config_file(self):
        rospack = rospkg.RosPack()

        try:
            path = rospack.get_path('antobot_description')
            with open(path+'/config/platform_config.yaml','r') as file:
                params = yaml.safe_load(file)

            # Check if imu compensated frame is used - default true
            if "imu_compensated_frame" in params:
                self.imu_frame = params["imu_compensated_frame"]
            else:
                self.imu_frame = True
            print('test')
            print(f'imu_frame: {self.imu_frame}')
            self.lidars = {}
            
            for lidar_type in params["lidar"]:
                if lidar_type == 'mid360':
                    ip = params["lidar"][lidar_type]["device_ip"]
                    m_port = params["lidar"][lidar_type]["msop_port"]
                    d_port = params["lidar"][lidar_type]["difop_port"]

                    if params["lidar"][lidar_type]["mode"] == "navigation":
                        frame_id = "laser_link_" + params["lidar"][lidar_type]["location"]
                        self.for_navigation = True
                        print('navigation')
                    else:
                        frame_id = "laser_link_" + params["lidar"][lidar_type]["location"] + "_static"
                    print(frame_id)
                    self.lidars[lidar_type] = lidar_mid360('lidar_'+params["lidar"][lidar_type]["location"], ip, frame_id, location=params["lidar"][lidar_type]["location"])

                else: # c16
                    ip = params["lidar"][lidar_type]["device_ip"]
                    m_port = params["lidar"][lidar_type]["msop_port"]
                    d_port = params["lidar"][lidar_type]["difop_port"]

                    if params["lidar"][lidar_type]["mode"] == "navigation":
                        frame_id = "laser_link_" + params["lidar"][lidar_type]["location"]
                        self.for_navigation = True
                        print('navigation')
                    else:
                        frame_id = "laser_link_" + params["lidar"][lidar_type]["location"] + "_static"

                    self.lidars[lidar_type] = lidar_c16('lidar_'+params["lidar"][lidar_type]["location"], ip, m_port, d_port, frame_id, location=params["lidar"][lidar_type]["location"])
        except Exception as e:
            self.get_logger().err(f"SW2320: Lidar Manager: Failed to read robot config file. Error: {e}")




    ############################################################################################
    ## RosService callback - This is the main method of interaction (also works with simulation)
    ############################################################################################

    def _serviceCallbackLidarMgr(self, request):
       
        ## ROS service input:
        #bool frontPowerOn			# Turn on/off 
        #bool frontCostmapEnable    # Enable/disable in costmap
        #bool rearPowerOn			# Turn on/off 
        #bool rearCostmapEnable     # Enable/disable in costmap
        #---
        #int8 responseCode		    # True - success, False - failure
        #string responseString      # Additional info


        # Create the return message
        return_msg = lidarManagerResponse()

        frontLidarAvailable=False
        rearLidarAvailable=False

        return_msg.responseCode = False 


        for lidar_i in self.lidars.values():

            #################################################################################

            if lidar_i.location == 'front':

                frontLidarAvailable=True
                return_msg.responseCode = True  # True when lidar is found
                return_msg.responseString = 'Lidar found - applying settings'


                ## Front Lidar Power ---------------------------------------------------------

                if request.frontPowerOn:
                    # Check if the lidar is already active
                    if lidar_i.active:
                        self.get_logger().info("SW2320: Lidar Manager: Front Lidar already active. ")
                    else:
                        # Simulated Lidar just impersonates real lidar
                        if self.sim:
                            self.get_logger().info("SW2320: Lidar Manager: Simulated front lidar should be turned on (logic check)")
                            lidar_i.active = True

                        else: # For the real lidar
                            self.get_logger().info("SW2320: Lidar Manager: Turning on front Lidar")
                            lidar_i.createLauncher()
                            lidar_i.start()
                
                else:
                    # Check if the lidar is already inactive
                    if not lidar_i.active:
                        self.get_logger().info("SW2320: Lidar Manager: Front Lidar already inactive")
                    else:
                        # Simulated Lidar just impersonates real lidar
                        if self.sim:
                            self.get_logger().info("SW2320: Lidar Manager: Simulated front lidar should be turned off (logic check)")
                            lidar_i.active = False

                        else: # For the real lidar
                            self.get_logger().info("SW2320: Lidar Manager: Turning off front Lidar")
                            lidar_i.shutdown()

            
            #######################################################################################
            
            elif lidar_i.location == 'rear':

                rearLidarAvailable=True
                return_msg.responseCode = True  # True when lidar is found
                return_msg.responseString = 'Lidar found - applying settings'


                ## Rear Lidar Power ---------------------------------------------------------

                if request.rearPowerOn:
                    # Check if the lidar is already active
                    if lidar_i.active:
                        self.get_logger().info("SW2320: Lidar Manager: Rear Lidar already active. ")
                    else:
                        # Simulated Lidar just impersonates real lidar
                        if self.sim:
                            self.get_logger().info("SW2320: Lidar Manager: Simulated rear lidar should be turned on (logic check)")
                            lidar_i.active = True

                        else: # For the real lidar
                            self.get_logger().info("SW2320: Lidar Manager: Turning on rear Lidar")
                            lidar_i.createLauncher()
                            lidar_i.start()
                
                else:
                    # Check if the lidar is already inactive
                    if not lidar_i.active:
                        self.get_logger().info("SW2320: Lidar Manager: Rear Lidar already inactive")
                    else:
                        # Simulated Lidar just impersonates real lidar
                        if self.sim:
                            self.get_logger().info("SW2320: Lidar Manager: Simulated rear lidar should be turned off (logic check)")
                            lidar_i.active = False

                        else: # For the real lidar
                            self.get_logger().info("SW2320: Lidar Manager: Turning off rear Lidar")
                            lidar_i.shutdown()


                  
        # If neither front nor rear lidar were found
        if (not rearLidarAvailable) and (not frontLidarAvailable):
            return_msg.responseCode = False 
            self.get_logger().info("SW2320: Lidar Manager: Unable to find any configurable lidar")
            return_msg.responseString = "Unable to find any configurable lidar. "


        return return_msg
    


    

######################################################################################################
## Main
######################################################################################################

def main(args):
    
    rclpy.init()
    try:
        lm = lidarManagerClass()
        rclpy.spin(lm)
    except KeyboardInterrupt:
        sys.exit(1)
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        lm.destroy_node()

if __name__ == '__main__':
    main(sys.argv)

