#!/usr/bin/env python3

from antobot_devices_msgs.srv import LidarManager

class lidarManagerClient():
    '''Client for controllig lidar'''

    def __init__(self, node_in=None):    
        
        self.serviceName = '/antobot/lidarManager'
        self.lidarManagerClient = node_in.create_client(LidarManager, self.serviceName)

        # Message contents
        self.frontPowerOn=False			  # Turn on/off 
        self.frontCostmapEnable=False     # Enable/disable in costmap
        self.rearPowerOn=False			  # Turn on/off 
        self.rearCostmapEnable=False      # Enable/disable in costmap

        self.req = LidarManager.Request()

       
    def checkForService(self):
        return self.lidarManagerClient.service_is_ready()

    def updateLidar(self):
        
        self.req.front_power_on = self.frontPowerOn
        self.req.front_costmap_enable = self.frontCostmapEnable
        self.req.rear_power_on = self.rearPowerOn
        self.req.rear_costmap_enable = self.rearCostmapEnable

        lmResponse = self.lidarManagerClient.call_async(self.req)

        print('Lidar Manager Client: '+ lmResponse.responseString)
        return lmResponse


