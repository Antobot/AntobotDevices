#!/usr/bin/env python3

import rospy
import rosservice
from antobot_devices_msgs.srv import lidarManager, lidarManagerResponse

class lidarManagerClient():
    '''Client for controllig lidar'''

    def __init__(self):    
        
        self.serviceName = '/antobot/lidarManager'
        self.lidarManagerClient = rospy.ServiceProxy(self.serviceName, lidarManager)

        # Message contents
        self.frontPowerOn=False			  # Turn on/off 
        self.frontCostmapEnable=False     # Enable/disable in costmap
        self.rearPowerOn=False			  # Turn on/off 
        self.rearCostmapEnable=False      # Enable/disable in costmap

       
    def checkForService(self):
        service_list = rosservice.get_service_list()
        if self.serviceName in service_list:
            return True
        else:
            return False

    def updateLidar(self):
        
        lmResponse = self.lidarManagerClient(self.frontPowerOn,self.frontCostmapEnable,self.rearPowerOn,self.rearCostmapEnable)

        print('Lidar Manager Client: '+ lmResponse.responseString)
        return lmResponse






