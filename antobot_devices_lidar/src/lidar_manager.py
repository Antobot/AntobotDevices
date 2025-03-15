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

from sensor_msgs.msg import Temperature, CameraInfo
from antobot_devices_msgs.srv import lidarManager, lidarManagerResponse
from sensor_msgs.msg import LaserScan
from antobot_devices_msgs.srv import costmapToggleObservation, costmapToggleObservationRequest
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class costmapManager:
    def __init__(self):
        self.launch = None
        self.costmapManagerService = rospy.Service('/costmap_manager/node_died_costmap',Empty,self.restart_costmap)

        self.costmapManagerService = rospy.Service('/costmap_manager/node_died_imu',Empty,self.restart_imu)

        # launch imu euler 
        self.launch_imu_euler()

        # launch costmap
        self.launch_costmap()

    def restart_costmap(self,req):
        print("restart the costmap node")
        self.launch_costmap()
        return EmptyResponse()
    
    def restart_imu(self,req):
        print("restart the imu node")
        self.launch_imu_euler()
        return EmptyResponse()
        

    def launch_costmap(self):
        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['antobot_move_costmap_2d', 'ant_costmap_only.launch'] # default costmap launch file (applied for both V2 and V3)

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        print(roslaunch_file)

        launch_files = [roslaunch_file]

        self.launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files,process_listeners=[ProcessListener()])
        # start the launch file
        self.launch.start_node_name("costmap")
    
    def launch_imu_euler(self):
        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['antobot_devices_imu', 'imu_euler.launch']

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        print(roslaunch_file)

        launch_files = [roslaunch_file]

        self.launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files,process_listeners=[ProcessListener()])
        # start the launch file
        self.launch.start_node_name("imu_euler")



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
            print("Simulation - robot lidar launched with Gazebo")
            self.active = True
        else:
            # By default, turn on all the available lidar 
            self.createLauncher()
            self.start() # TODO: do we want logerr for lidar node?



    def createLauncher(self):
        '''Starts a camera by calling the c16 launch file'''

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        #cli_args = ['antobot_move_cartograph', 'antoCartographer.launch']
        cli_args = ['antobot_devices_lidar', 'lslidar_config_launch_updated.launch', 'name_space:='+self.name_space, 'frame_id:='+self.frame_id,  'device_ip:='+self.ip, 'msop_port:='+self.m_port, 'difop_port:='+self.d_port]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        print(roslaunch_file)
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]

        self.launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files)
        

    def start(self):
        if self.active==False:
            self.active=True
            self.launch.start()

    def shutdown(self):
        if self.active==True:
            self.active=False
            self.launch.stop()
            print("Lidar {} shutdown".format(self.location))
        else:
            print("Lidar not running - no shutdown")

    
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
            print("Shouldn't happen")
            return_msg.responseCode = False
            return_msg.responseString = "Failed to disable the topic"
        
        return return_msg

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
            print("Shouldn't happen")
            return_msg.responseCode = False
            return_msg.responseString = "Failed to enable the topic"

        return return_msg

    def checkStatus(self):
        pass # data check / or error message / turned on/off




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
            print('Lidar Manager: This is a simulation')
        else:
            print('Lidar Manager: This is NOT a simulation - using real Lidar')

        # Create a service to allow other nodes to start/stop lidars
        self.srvLidarMgr = rospy.Service("/antobot/lidarManager", lidarManager, self._serviceCallbackLidarMgr)


    
    def read_config_file(self):
        rospack = rospkg.RosPack()

        try:
            device_type = rospy.get_param("/device_type",'robot') #Add default value - used when not launching with softwareManager
            path = rospack.get_path('antobot_description')
            with open(path+'/config/platform_config.yaml','r') as file:
                params = yaml.safe_load(file)

            # Check if imu compensated frame is used - default true
            if "imu_compensated_frame" in params["lidar"]:
                self.imu_frame = params["lidar"]["imu_compensated_frame"]
            else:
                self.imu_frame = True

            self.lidars = {}
            
            for lidar_type in params["lidar"]:
                ip = params["lidar"][lidar_type]["device_ip"]
                m_port = params["lidar"][lidar_type]["msop_port"]
                d_port = params["lidar"][lidar_type]["difop_port"]

                if params["lidar"][lidar_type]["mode"] == "navigation":
                    frame_id = "laser_link_" + params["lidar"][lidar_type]["location"]
                    self.for_navigation = True
                else:
                    frame_id = "laser_link_" + params["lidar"][lidar_type]["location"] + "_static"

                self.lidars[lidar_type] = lidar('lidar_'+params["lidar"][lidar_type]["location"], ip, m_port, d_port, frame_id,location=params["lidar"][lidar_type]["location"])
        except:
            print("Failed to read robot config file ")


    ############################################################################################
    ## RosService callback - This is the main method of interaction (also works with simulation)
    ############################################################################################

    def _serviceCallbackLidarMgr(self, request):
       
        ## ROS service input:
        # string lidarName		    # 1 - front, 2 - rear 
        # int8 command			# 0 - turn off, 1 - turn on, 2 - disable in costmap, 3 - enable in costmap
        # ---
        # int8 responseCode		# True - success, False - failure/not needed command
        # string responseString   # Additional info

        # Create the return message
        return_msg = lidarManagerResponse()

        if request.command == 0: # Turn off lidar
            lidar_found = False
            for lidar_i in self.lidars.values():
                if lidar_i.location == request.lidarName:
                    lidar_found = True
                    if lidar_i.active:
                        if not self.sim:
                            lidar_i.shutdown()
                        else:
                            print("Simulation - lidar turned off (just for logic check)")
                            lidar_i.active = False
                        return_msg.responseCode = True
                        return_msg.responseString = request.lidarName + " lidar turn off"
                    elif not lidar_i.active:
                        return_msg.responseCode = True # True since it's already turned off
                        return_msg.responseString = request.lidarName + " lidar already turned off"
                    break
            if not lidar_found:
                return_msg.responseCode = False
                return_msg.responseString = request.lidarName + "lidar not available"


        elif request.command == 1: # Turn on lidar
            lidar_found = False
            for lidar_i in self.lidars.values():
                if lidar_i.location == request.lidarName:
                    lidar_found = True
                    if lidar_i.active:
                        return_msg.responseCode = True # True since it's already active
                        return_msg.responseString = request.lidarName + " lidar was already on"
                    elif not lidar_i.active:
                        if not self.sim :
                            lidar_i.createLauncher()
                            lidar_i.start()
                        else:
                            print("Simulation - lidar turned on (just for logic check)")
                            lidar_i.active = True
                        return_msg.responseCode = True
                        return_msg.responseString = request.lidarName + " lidar turn on"
                    break
            if not lidar_found:
                return_msg.responseCode = False
                return_msg.responseString = "lidar Name not available"

        elif request.command == 2: # Disable in costmap
            for lidar_i in self.lidars.values():
                if lidar_i.location == request.lidarName:
                    return_msg.responseCode = True
                    return_msg = lidar_i.disableInCostmap()
                    break
                else:
                    return_msg.responseCode = False
                    return_msg.responseString = request.lidarName + " lidar not available"

        elif request.command == 3: # Enable in costmap
            for lidar_i in self.lidars.values():
                if lidar_i.location == request.lidarName:
                    return_msg.responseCode = True
                    return_msg = lidar_i.enableInCostmap()
                    break
                else:
                    return_msg.responseCode = False
                    return_msg.responseString = request.lidarName + "lidar not available"

        else: # Wrong command
            return_msg.responseCode = False
            return_msg.responseString = request.command + " command not available"

        return return_msg
    


    

######################################################################################################
## Main
######################################################################################################

def main(args):
    
    rospy.init_node('lidarManager', anonymous=False)
    # Turn lidars on
    lidarMgr = lidarManagerClass()
    print('Lidar Manager started')
    rospy.sleep(1.0)

    # launch costmap node
    if lidarMgr.for_navigation:
        costmapMgr = costmapManager()
        print('Costmap Manager started')

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

