#!/usr/bin/env python3

# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:  compute the distance between raw gps and filter gps.

# Contacts: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import time
import datetime
import math
import os
from sensor_msgs.msg import NavSatFix

import yaml
import socket
import serial
import asyncio

import rospy



class gpsCompare():
    def __init__(self):

        self.gps_raw_lat = None
        self.gps_raw_lon = None
        self.gps_raw_time = time.time()

        self.gps_filter_lat = None
        self.gps_filter_lon = None   
        self.gps_filter_time = time.time()     

        self.r = 6371

        self.fileNmae = 'gps_compare.txts'

        self.gps_raw_sub=rospy.Subscriber('/antobot_gps', NavSatFix, self.getGPSRawCallback)
        self.gps_filter_sub=rospy.Subscriber('/gps/filtered', NavSatFix, self.getGPSFilterCallback)


    def getGPSRawCallback(self,data):
        self.gps_raw_lat=data.latitude
        self.gps_raw_lon=data.longitude

        self.gps_raw_time = time.time()

    
    def getGPSFilterCallback(self,data):
        self.gps_filter_lat=data.latitude
        self.gps_filter_lon=data.longitude

        self.gps_filter_time = time.time()   


    def save_data(self, data):
        with open(self.fileNmae, "a") as file:
            file.write(data)

    def haversine(self, envent=None):
        
        if self.gps_raw_lat and self.gps_filter_lat and abs(self.gps_raw_time - self.gps_filter_time) < 0.001:
            lat1, lon1, lat2, lon2 = map(math.radians, [self.gps_raw_lat, self.gps_raw_lon, self.gps_filter_lat, self.gps_filter_lon])
            
            dlat = lat2 - lat1
            dlon = lon2 - lon1
            a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
            c = 2 * math.asin(math.sqrt(a))
            
            dis = c * self.r * 1000
            self.save_data(f"time:{time.time()}; distance: {dis}\n")
            print(f"dis is {dis}")
            return dis
    
def main():
    rospy.init_node ('gpsCompare') 
    gpsCp = gpsCompare()

    rospy.Timer(rospy.Duration(0.02), gpsCp.haversine)  # Runs periodically without blocking
    rospy.spin() 


if __name__ == '__main__':
    main()
