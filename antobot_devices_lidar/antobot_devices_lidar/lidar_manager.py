#! /usr/bin/env python3

# Copyright (c) 2022, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     This code manages the different lidars on Antobot's robots. It has a service (/anto_manager/lidar)
#                           that can be called for turning on/off the lidar and enable/distable the lidar input in the costmap.

# Contacts: soyoung.kim@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import os, signal
import sys
import yaml
import multiprocessing
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

from antobot_devices_msgs.srv import LidarManager
from antobot_devices_msgs.srv import CostmapToggleObservation

###################################################################################################################################################

###################################################################################################################################################
class lidar(Node): # lidar pare
    '''Stores lidar specific information'''
    def __init__(self,id=200,sim=False, type=""):
        
        super().__init__(f"{type}_{id}_object", namespace='internal') # Initialise Node class first
        # Save arguments
        self.active = False # Lidar turn on/off 
        self.enabled_in_costmap = True # default True
        self.launch = None
        self.id = id
        self.sim = sim
        self.p = None
        
        ## Client for controlling lidar input to the costmap node
        self.costmapToggleClient = self.create_client(CostmapToggleObservation,'/costmap_node/costmap/toggle_observation')
           

#################################################
    def run(self):
        if self.sim:
            
            self.active = True
        else:
            # By default, turn on all the available lidar 
            print("multiprocessing start")
            self.p = multiprocessing.Process(target=self.createLauncher)
            self.start()

#################################################

    def createLauncher(self):
        pass
        # To be implemented in the child class
        
#################################################

    def start(self):
        if not self.active:
            print("start function")
            self.active = True
            # If process was already used, recreate it
            if self.p is None or not self.p.is_alive():
                def launcher_wrapper():
                    # Create a new process group for this subprocess
                    #os.setpgrp()
                    self.createLauncher()

                self.p = multiprocessing.Process(target=launcher_wrapper)
            self.p.start()
        else:
            print(f"[{self.id}] already active.")

#################################################

    def shutdown(self):
        """Stop the lidar process safely."""
        print("Shutdown called")
        if self.active:
            self.active = False
            if self.p and self.p.is_alive():
                print(f"[{self.id}] terminating process PID={self.p.pid}")
                try:
                    # Send SIGTERM to the whole process group
                    os.killpg(os.getpgid(self.p.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass

                self.p.join(timeout=5.0)
                if self.p.is_alive():
                    print(f"[{self.id}] forcing kill...")
                    os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
                else:
                    print(f"[{self.id}] process group terminated.")
            else:
                print(f"[{self.id}] no running process.")
            self.p = None
        else:
            print(f"[{self.id}] shutdown requested but lidar not active.")

#################################################
    
    def disableInCostmap(self):
        # disable this input from the costmap
        return_msg = LidarManager.Response()
        if self.active and self.enabled_in_costmap:
            topic_name = "/lidar_"+self.id+"/pointcloud"
            req = CostmapToggleObservation.Request()
            req.observation_source = topic_name
            req.command = False
        
            self.costmapToggleClient.call(req) # Always return true 
            return_msg.response_code = True
            return_msg.response_string = topic_name + " disabled in the costmap"
            self.enabled_in_costmap = False
        elif not self.active:
            return_msg.response_code = False
            return_msg.response_string = "lidar not turned off - no disabling the topic in the costmap"
        elif not self.enabled_in_costmap:
            return_msg.response_code = True # True since it's already disabled
            return_msg.response_string = "topic already disabled in the costmap"
        else:
            # This Shouldn't happen
            self.get_logger().info("SW2320: Lidar Manager: Lidar topic is neither active nor enabled in costmap - failed to disable the topic")
            return_msg.response_code = False
            return_msg.response_string = "Failed to disable the topic"
        
        return return_msg
    
#################################################

    def enableInCostmap(self):
        return_msg = LidarManager.Response()
        if self.active and not self.enabled_in_costmap:
            topic_name = "/lidar_"+self.id+"/pointcloud"
            req = CostmapToggleObservation.Request()
            req.observation_source = topic_name
            req.command = True
            self.costmapToggleClient.call(req) # Always return true 
            return_msg.response_code = True
            return_msg.response_string = topic_name + " enabled in the costmap"
            self.enabled_in_costmap = True
        elif not self.active:
            return_msg.response_code = False
            return_msg.response_string = "lidar not turned off - no enabling the topic in the costmap"
        elif self.enabled_in_costmap:
            return_msg.response_code = True # True since it's already enabled
            return_msg.response_string = "topic already enabled in the costmap"
        else:
            self.get_logger().info("SW2320: Lidar Manager: Lidar topic is alrady active and enabled in costmap")
            return_msg.response_code = False
            return_msg.response_string = "Failed to enable the topic"

        return return_msg

class lidar_cx(lidar):
    '''Stores lidar specific information'''
    def __init__(self, ip="",m_port="",d_port="",frame_id="",id=200,sim=False):

        super().__init__(id, sim, type="cx")
        self.type = 'cx' # Can support any Cx lidar (C16, C32, etc)
        # Save arguments
        self.m_port = str(m_port)
        self.d_port = str(d_port)
        self.frame_id = frame_id
        self.name_space = f'lidar_{id}'
        self.device_ip = ip
        self.device_id = id

        super().run() # run the lidar (create launcher and start if not simulation)
#################################################

    def createLauncher(self):
        '''Starts cx launch file'''

        self.launch_file_path = os.path.join(get_package_share_directory('antobot_devices_lidar'),'launch',self.type+"_launch.py")

        included_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(self.launch_file_path),
            launch_arguments={
                'name_space': self.name_space,
                'frame_id': self.frame_id,
                'device_ip': self.device_ip,
                'msop_port': self.m_port,
                'difop_port': self.d_port
            }.items()
        )

        ld = LaunchDescription()
        ld.add_action(included_launch)

        ls = LaunchService()
        ls.include_launch_description(ld)

        # === Handle termination signals gracefully ===
        def shutdown_handler(signum, frame):
            print(f"[{self.type}] Immediate kill requested (signal {signum})")
            try:
                # kill this whole process group: launch_ros + lidar_driver_node
                os.killpg(os.getpgid(os.getpid()), signal.SIGKILL)
            except ProcessLookupError:
                pass
            sys.exit(0)

        # Register signal handlers
        signal.signal(signal.SIGINT, shutdown_handler)
        signal.signal(signal.SIGTERM, shutdown_handler)

        print(f"[{self.type}] Launching lidar driver...")
        try:
            ls.run()
        except KeyboardInterrupt:
            shutdown_handler(signal.SIGINT, None)
        finally:
            print(f"[{self.type}] Launch process exited cleanly.")

###################################################################################################################################################


class lidar_mid360(lidar):
    '''Stores lidar specific information'''
    def __init__(self, ip="",frame_id="",num=1,id=200,sim=False):

        super().__init__(id, sim,type="mid360")
        self.type = 'mid360'
        # Save arguments
        # TODO: mid360 port settings
        self.frame_id = frame_id
        self.name_space = f'lidar_{id}'
        self.ip = ip
        self.device_id = id
        self.num = num


        super().run() # run the lidar (create launcher and start if not simulation)
#################################################

    def createLauncher(self):
        '''Starts mid360 launch file'''

        if(self.num == 1):
            self.launch_file_path = os.path.join(get_package_share_directory('antobot_devices_lidar'),'launch',self.type+"_launch.py")
        elif(self.num == 2):
            self.launch_file_path = os.path.join(get_package_share_directory('antobot_devices_lidar'),'launch',"multi_"+self.type+"_launch.py")
        included_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(self.launch_file_path),
            launch_arguments={
                'name_space': self.name_space,
                'frame_id': self.frame_id,
                'id': str(self.device_id)
            }.items()
        )

        ld = LaunchDescription()
        ld.add_action(included_launch)

        ls = LaunchService()
        ls.include_launch_description(ld)
        print("create Launcher in mid360 New")

        # === Handle termination signals gracefully ===
        def shutdown_handler(signum, frame):
            print(f"[{self.type}] Immediate kill requested (signal {signum})")
            try:
                # kill this whole process group: launch_ros + lidar_driver_node
                os.killpg(os.getpgid(os.getpid()), signal.SIGKILL)
            except ProcessLookupError:
                pass
            sys.exit(0)

        # Register signal handlers
        signal.signal(signal.SIGINT, shutdown_handler)
        signal.signal(signal.SIGTERM, shutdown_handler)

        print(f"[{self.type}] Launching lidar driver...")
        try:
            ls.run()
        except KeyboardInterrupt:
            shutdown_handler(signal.SIGINT, None)
        finally:
            print(f"[{self.type}] Launch process exited cleanly.")
    

###################################################################################################################################################


class lidarManagerClass(Node):
    # # # The lidarManager class reads the intended configuration for a particular device, then launches
    # # # and monitors the software for each individual lidar.

    def __init__(self):
        super().__init__("lidarManager") # Initialise Node class first
        # Read robot_config file
        self.sim = False
        self.for_navigation = False
        self.read_config_file()

        ##############################################################################
        ## Run either real or simulated manager    
        ##############################################################################

        if self.sim:
            self.get_logger().info("SW2320: Lidar Manager: Running in simulation mode")
        else:
            self.get_logger().info("SW2320: Lidar Manager: This is NOT a simulation - using real Lidar")

        # Create a service to allow other nodes to start/stop lidars
        self.srvlidarMgr = self.create_service(LidarManager,"/antobot/lidarManager",self._serviceCallbackLidarMgr)


#################################################
    
    def read_config_file(self):
        try:
            package_path = get_package_share_directory('antobot_description')
            config_file = os.path.join(package_path, 'config', 'platform_config.yaml')

            with open(config_file, 'r') as file:
                params = yaml.safe_load(file)

            robot_hardware = params["robot_hardware"]
            if robot_hardware:
                self.sim = False
            else:
                self.sim = True

            self.lidars = {}

            mid360_count = sum(1 for cfg in params["lidar"].values() if cfg["type"] == "mid360")
            print("SW2320: Lidar Manager: Found {} mid360 lidars".format(mid360_count))

            for lidar_id, lidar_cfg in params["lidar"].items():
                lidar_type = lidar_cfg["type"]
                mode = lidar_cfg.get("mode", "navigation")
                ip = lidar_cfg.get("device_ip", None)
                id = ip.split('.')[-1]
                if mode == "navigation":
                    frame_id = f"lidar_{id}_frame"
                    self.for_navigation = True
                else:
                    frame_id = f"lidar_{id}_link"

                # Instantiate based on type
                if lidar_type == "mid360":
                    # Example: lidar_mid360(ip, frame_id, id, sim)
                    self.lidars[f"mid360{lidar_id}"] = lidar_mid360(
                        ip,
                        frame_id,
                        mid360_count,
                        id=int(id),
                        sim=self.sim
                    )
                    print(f"Started mid360 lidar {lidar_id} at {ip} ({id})")

                elif lidar_type == "cx":
                    m_port = lidar_cfg.get("msop_port", 2368)
                    d_port = lidar_cfg.get("difop_port", 2369)
                    # Example: lidar_cx(ip, m_port, d_port, frame_id, id, sim)
                    self.lidars[f"cx{lidar_id}"] = lidar_cx(
                        ip,
                        m_port,
                        d_port,
                        frame_id,
                        id=int(id),
                        sim=self.sim
                    )
                    print(f"Started cx lidar {lidar_id} at {ip}:{m_port}/{d_port} ({id})")

                else:
                    print(f"Unknown lidar type '{lidar_type}' for ID {lidar_id} — skipping.")
                    
        except Exception as e:
            self.get_logger().error(f"SW2320: Lidar Manager: Failed to read robot config file. Error: {e}")


    def terminate(self):
        self.get_logger().info("SW2320: Lidar Manager: Terminating lidar")
        for lidar_i in self.lidars.values():
            print(lidar_i.type)
            #################################################################################
            if lidar_i.active:
                lidar_i.shutdown()

    ############################################################################################
    ## RosService callback - This is the main method of interaction (also works with simulation)
    ############################################################################################
    
    def _serviceCallbackLidarMgr(self, request, response):
       
        ## ROS service input:
        #bool front_power_on			# Turn on/off 
        #bool front_costmap_enable    # Enable/disable in costmap
        #bool rear_power_on			# Turn on/off 
        #bool rear_costmap_enable     # Enable/disable in costmap
        #---
        #int8 response_code		    # True - success, False - failure
        #string response_string      # Additional info




        frontLidarAvailable=False
        rearLidarAvailable=False

        response.response_code = False 
        print(self.lidars.values())

        for lidar_i in self.lidars.values():
            print(lidar_i.type, lidar_i.device_id)
            #################################################################################

            if lidar_i.device_id == 200:

                frontLidarAvailable=True
                response.response_code = True  # True when lidar is found
                response.response_string = 'Lidar found - applying settings'


                ## Front Lidar Power ---------------------------------------------------------

                if request.front_power_on:
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
                            #lidar_i.createLauncher()
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
            
            elif lidar_i.device_id == 201:

                rearLidarAvailable=True
                response.response_code = True  # True when lidar is found
                response.response_string = 'Lidar found - applying settings'


                ## Rear Lidar Power ---------------------------------------------------------

                if request.rear_power_on:
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
                            #lidar_i.createLauncher()
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
            response.response_code = False 
            self.get_logger().info("SW2320: Lidar Manager: Unable to find any configurable lidar")
            response.response_string = "Unable to find any configurable lidar. "


        return response
    


    

######################################################################################################
## Main
######################################################################################################

def main(args=None):
    
    rclpy.init()
    try:
        lm = lidarManagerClass()
        rclpy.spin(lm)
    except KeyboardInterrupt:
        lm.terminate()
        sys.exit(1)
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        lm.destroy_node()

if __name__ == '__main__':
    main()

