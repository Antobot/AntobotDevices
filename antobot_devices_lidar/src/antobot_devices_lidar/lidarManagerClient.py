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
        self.lidarName='front'  # front, rear
        self.command=0			# 0 - turn off, 1 - turn on, 2 - disable in costmap, 3 - enable in costmap


    def checkForService(self):
        service_list = rosservice.get_service_list()
        if self.serviceName in service_list:
            return True
        else:
            return False

    def updateLidar(self):
        
        moveToDropoffResponse = self.lidarManagerClient(self.lidarName,self.command)

        print('Lodar Manager Client: '+ moveToDropoffResponse.responseString)
        return moveToDropoffResponse






