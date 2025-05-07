#! /usr/bin/env python3

# Copyright (c) 2022, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     This code manages the different lidars on Antobot's robots. It has a service (/anto_manager/lidar)
#                           that can be called for turning on/off the lidar and enable/distable the lidar input in the costmap.

# Contacts: soyoung.kim@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# TODO 
# handle simulation - what to launch when the service is called in the simulation?



import sys
import yaml
import rospy
from pathlib import Path
from datetime import datetime

import roslaunch # Using this until we develop our own camera manager solution
import rospkg

from antobot_manager_software.launchManager import ProcessListener, RoslaunchWrapperObject

from antobot_devices_msgs.srv import lidarManager, lidarManagerResponse
from antobot_devices_msgs.srv import costmapToggleObservation, costmapToggleObservationRequest
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


###################################################################################################################################################

class costmapManager:
    def __init__(self,imu_frame,lidar_count):
        self.lidar_count = lidar_count
        self.imu_frame = imu_frame
        self.costmapLaunchWrapper = None
        self.imuLaunchWrapper = None

        self.costmapManagerService = rospy.Service('/costmap_manager/node_died_costmap',Empty,self.restart_costmap)
        self.costmapManagerService = rospy.Service('/costmap_manager/node_died_imu',Empty,self.restart_imu)

        # Set the costmap and imu to relaunch by default
        self.costMapTargetState=True
        self.imuTargetState=True

        # launch imu euler 
        self.launch_imu_euler()

        # launch costmap
        self.launch_costmap()

#################################################

    def restart_costmap(self,req):

        if self.costMapTargetState:

            rospy.loginfo('SW2320: Lidar Manager: Costmap node relaunching')
            self.launch_costmap()
        else:
            rospy.loginfo('SW2320: Lidar Manager: Costmap disabled - not relaunching')

        return EmptyResponse()

#################################################

    def restart_imu(self,req):

        if self.imuTargetState:

            rospy.loginfo('SW2320: Lidar Manager: IMU node relaunching')
            self.launch_imu_euler()
        else:
            rospy.loginfo('SW2320: Lidar Manager: IMU disabled - not relaunching')

        return EmptyResponse()

#################################################   

    def launch_costmap(self):
        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['antobot_move_costmap_2d', 'ant_costmap_only.launch'] # default costmap launch file (applied for both V2 and V3)

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        #print(roslaunch_file)

        launch_files = [roslaunch_file]

        self.costmapLaunchWrapper = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files,process_listeners=[ProcessListener()])
        # start the launch file
        self.costmapLaunchWrapper.start_node_name("costmap")
        
        # Set the target State to True
        self.costMapTargetState=True
    
#################################################

    def launch_imu_euler(self):
        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        if self.imu_frame: # use imu roll angle to generate lidar frame (imu compensated frame)  
            rospy.loginfo("SW2320: Lidar Manager: use IMU compensated lidar frames")
            cli_args = ['antobot_devices_imu', 'imu_euler.launch']
        else:
            if self.lidar_count == 1:
                rospy.loginfo("SW2320: Lidar Manager: Using static lidar frame for the front lidar")
                cli_args = ['antobot_devices_imu', 'static_lidar_frame.launch']
            else: # dual lidar
                rospy.loginfo("SW2320: Lidar Manager: Using static lidar frames for both lidars")
                cli_args = ['antobot_devices_imu', 'static_lidar_frame_two_lidars.launch']

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        #print(roslaunch_file)

        launch_files = [roslaunch_file]

        self.imuLaunchWrapper = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files,process_listeners=[ProcessListener()])
        # start the launch file
        self.imuLaunchWrapper.start_node_name("imu_euler")


#################################################

    def stop_costmap(self):
        if self.costMapTargetState==True:
            self.costMapTargetState=False
            self.costmapLaunchWrapper.stop()
            rospy.loginfo("SW2320: Lidar Manager: Shutting down costmap node")
        else:
            rospy.loginfo("SW2320: Lidar Manager: Shutdown requested but no active costmap node")

#################################################

    def stop_imu_euler(self):
        if self.imuTargetState==True:
            self.imuTargetState=False
            self.imuLaunchWrapper.stop()
            rospy.loginfo("SW2320: Lidar Manager: Shutting down imu node")
        else:
            rospy.loginfo("SW2320: Lidar Manager: Shutdown requested but no active imu node")


###################################################################################################################################################

class lidar: # Currently for C16 lidar
    '''Stores lidar specific information'''
    def __init__(self,name ="c16",ip="",m_port="",d_port="",frame_id="",location="front"):

        # Save arguments
        self.ip = ip
        self.m_port = str(m_port)
        self.d_port = str(d_port)
        self.name_space = name
        self.frame_id = frame_id
        self.active = False # Lidar turn on/off 
        self.enabled_in_costmap = True # default True
        self.launch = None
        self.location = location

        
        ## Client for controlling lidar input to the costmap node
        self.costmapToggleClient = rospy.ServiceProxy('/costmap_node/costmap/toggle_observation',costmapToggleObservation)

        ##############################################################################
        ## Check for the simulation parameter which should be set by am_sim     
        ##############################################################################
           
        try:
            self.sim = rospy.get_param("/simulation")
        except:
            self.sim=False # If the simulation parameter has not been assigned, assume not a simulation

        if self.sim:
            rospy.loginfo('SW2320: Lidar Manager: Simulation - robot lidar launched with Gazebo')
            self.active = True
        else:
            # By default, turn on all the available lidar 
            self.createLauncher()
            self.start() # TODO: do we want logerr for lidar node?

#################################################

    def createLauncher(self):
        '''Starts a camera by calling the c16 launch file'''

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        #cli_args = ['antobot_move_cartograph', 'antoCartographer.launch']
        cli_args = ['antobot_devices_lidar', 'lslidar_config_launch_updated.launch', 'name_space:='+self.name_space, 'frame_id:='+self.frame_id,  'device_ip:='+self.ip, 'msop_port:='+self.m_port, 'difop_port:='+self.d_port]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        #print(roslaunch_file)
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]

        self.launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files)
        
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
            rospy.loginfo("SW2320: Lidar Manager: Lidar {} shutdown".format(self.location))

        else:
            rospy.loginfo("SW2320: Lidar Manager: Shutdown requested but no active lidar")

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
            rospy.loginfo("SW2320: Lidar Manager: Lidar topic is neither active nor enabled in costmap - failed to disable the topic")
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
            rospy.loginfo("SW2320: Lidar Manager: Lidar topic is alrady active and enabled in costmap")
            return_msg.responseCode = False
            return_msg.responseString = "Failed to enable the topic"

        return return_msg

#################################################

    def checkStatus(self):
        pass # data check / or error message / turned on/off





###################################################################################################################################################

class lidarManagerClass:
    # # # The lidarManager class reads the intended configuration for a particular device, then launches
    # # # and monitors the software for each individual lidar.

    def __init__(self):

        # Read robot_config file
        self.for_navigation = False
        self.read_config_file()

        ##############################################################################
        ## Check for the simulation parameter which should be set by am_sim     
        ##############################################################################
           
        try:
            self.sim = rospy.get_param("/simulation")

        except:
            self.sim=False # If the simulation parameter has not been assigned, assume not a simulation


        ##############################################################################
        ## Run either real or simulated manager    
        ##############################################################################

        if self.sim:
            rospy.loginfo("SW2320: Lidar Manager: Running in simulation mode")
        else:
            rospy.loginfo("SW2320: Lidar Manager: This is NOT a simulation - using real Lidar")

        # Create a service to allow other nodes to start/stop lidars
        self.srvLidarMgr = rospy.Service("/antobot/lidarManager", lidarManager, self._serviceCallbackLidarMgr)

        ##############################################################################
        ## Launch the costmap manager  
        ##############################################################################


        # launch costmap node if this lidar is being used for navigation
        if self.for_navigation:
            rospy.sleep(1.0) # Wait for the lidars to be ready
            self.costmapMgr = costmapManager(self.imu_frame, len(self.lidars)) # imu_compensated frame flag, number of lidars 
            rospy.loginfo("SW2320: Lidar Manager: Costmap Manager started")




#################################################
    
    def read_config_file(self):
        rospack = rospkg.RosPack()

        try:
            device_type = rospy.get_param("/device_type",'robot') #Add default value - used when not launching with softwareManager
            path = rospack.get_path('antobot_description')
            with open(path+'/config/platform_config.yaml','r') as file:
                params = yaml.safe_load(file)

            # Check if imu compensated frame is used - default true
            if "imu_compensated_frame" in params:
                self.imu_frame = params["imu_compensated_frame"]
            else:
                self.imu_frame = True
            print('test')
            self.lidars = {}
            
            for lidar_type in params["lidar"]:
                ip = params["lidar"][lidar_type]["device_ip"]
                m_port = params["lidar"][lidar_type]["msop_port"]
                d_port = params["lidar"][lidar_type]["difop_port"]

                if params["lidar"][lidar_type]["mode"] == "navigation":
                    frame_id = "laser_link_" + params["lidar"][lidar_type]["location"]
                    self.for_navigation = True
                    print('navigation')
                else:
                    frame_id = "laser_link_" + params["lidar"][lidar_type]["location"] + "_static"

                self.lidars[lidar_type] = lidar('lidar_'+params["lidar"][lidar_type]["location"], ip, m_port, d_port, frame_id,location=params["lidar"][lidar_type]["location"])
        except:
            rospy.logerr("SW2320: Lidar Manager: Failed to read robot config file")




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
                        rospy.loginfo("SW2320: Lidar Manager: Front Lidar already active. ")
                    else:
                        # Simulated Lidar just impersonates real lidar
                        if self.sim:
                            rospy.loginfo("SW2320: Lidar Manager: Simulated front lidar should be turned on (logic check)")
                            lidar_i.active = True

                        else: # For the real lidar
                            rospy.loginfo("SW2320: Lidar Manager: Turning on front Lidar")
                            lidar_i.createLauncher()
                            lidar_i.start()
                
                else:
                    # Check if the lidar is already inactive
                    if not lidar_i.active:
                        rospy.loginfo("SW2320: Lidar Manager: Front Lidar already inactive")
                    else:
                        # Simulated Lidar just impersonates real lidar
                        if self.sim:
                            rospy.loginfo("SW2320: Lidar Manager: Simulated front lidar should be turned off (logic check)")
                            lidar_i.active = False

                        else: # For the real lidar
                            rospy.loginfo("SW2320: Lidar Manager: Turning off front Lidar")
                            lidar_i.shutdown()

                
                ## Front Lidar Costmap ---------------------------------------------------------

                # Add the output of this lidar to the costmap
                if request.frontCostmapEnable:
                    return_msg1 = lidar_i.enableInCostmap()
                else:
                    return_msg1 = lidar_i.disableInCostmap()
            
            
            #######################################################################################
            
            elif lidar_i.location == 'rear':

                rearLidarAvailable=True
                return_msg.responseCode = True  # True when lidar is found
                return_msg.responseString = 'Lidar found - applying settings'


                ## Rear Lidar Power ---------------------------------------------------------

                if request.rearPowerOn:
                    # Check if the lidar is already active
                    if lidar_i.active:
                        rospy.loginfo("SW2320: Lidar Manager: Rear Lidar already active. ")
                    else:
                        # Simulated Lidar just impersonates real lidar
                        if self.sim:
                            rospy.loginfo("SW2320: Lidar Manager: Simulated rear lidar should be turned on (logic check)")
                            lidar_i.active = True

                        else: # For the real lidar
                            rospy.loginfo("SW2320: Lidar Manager: Turning on rear Lidar")
                            lidar_i.createLauncher()
                            lidar_i.start()
                
                else:
                    # Check if the lidar is already inactive
                    if not lidar_i.active:
                        rospy.loginfo("SW2320: Lidar Manager: Rear Lidar already inactive")
                    else:
                        # Simulated Lidar just impersonates real lidar
                        if self.sim:
                            rospy.loginfo("SW2320: Lidar Manager: Simulated rear lidar should be turned off (logic check)")
                            lidar_i.active = False

                        else: # For the real lidar
                            rospy.loginfo("SW2320: Lidar Manager: Turning off rear Lidar")
                            lidar_i.shutdown()

                
                ## Rear Lidar Costmap ---------------------------------------------------------

                # Add the output of this lidar to the costmap
                if request.rearCostmapEnable:
                    return_msg1 = lidar_i.enableInCostmap()
                else:
                    return_msg1 = lidar_i.disableInCostmap()
            
                  
        # If neither front nor rear lidar were found
        if (not rearLidarAvailable) and (not frontLidarAvailable):
            return_msg.responseCode = False 
            rospy.loginfo("SW2320: Lidar Manager: Unable to find any configurable lidar")
            return_msg.responseString = "Unable to find any configurable lidar. "






        #################################################################################
        
        # The costmap node is toggled based on lidar status
    
        if self.for_navigation: # If these lidars are used for navigation

            # If there are NO active lidars
            if (not request.frontPowerOn) and (not request.rearPowerOn):

                rospy.loginfo("SW2320: Lidar Manager: No active lidar - costmap should be stopped")

                # If the costmap is still active
                if self.costmapMgr.costMapTargetState:

                    rospy.loginfo("SW2320: Lidar Manager: No Lidar active:  Stopping costmap")
                    self.costmapMgr.stop_costmap()

            else: # If there are active lidars

                rospy.loginfo("SW2320: Lidar Manager: At least 1 lidar active - costmap required")

                # If the costmap is not active
                if not self.costmapMgr.costMapTargetState:

                    rospy.loginfo("SW2320: Lidar Manager: Lidar active - starting costmap")
                    self.costmapMgr.launch_costmap()


        return return_msg
    


    

######################################################################################################
## Main
######################################################################################################

def main(args):
    
    rospy.init_node('lidarManager', anonymous=False)
    # Turn lidars on
    lidarMgr = lidarManagerClass()
    rospy.loginfo("SW2320: Lidar Manager: Lidar Manager started")

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

