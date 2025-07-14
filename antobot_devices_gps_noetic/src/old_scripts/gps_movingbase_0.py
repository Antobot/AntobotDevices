#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # Code Description: This code is used to connect to ublox Things stream server to subscribe the GNSS augmentation data and implement PPP IP mode in ublox f9p chip. 
# This code should only be used in the Ublox chip : "ZED-F9P-04B-01" (>uRCUv1.2c) with firmware upgraded to HPG1.32.
# The chip was connected to the Xavier via SPI. To run the script at 8Hz, the following modifications should be made in the f9p chip. (run gps_config_movingbase.py first)
 #   CFG-RATE-MEAS = 125 sec
 #   VALSET: CFG-MSGOUT-NMEA-ID_GSV_UART1 = 0
 #   VALSET: CFG-MSGOUT-NMEA-ID_GSA_UART1 = 0
 #   VALSET: CFG-MSGOUT-NMEA-ID_GLL_UART1 = 0

#This script reports the following on GPS frequency status:
# GPS Frequency status : Critical ; when the GPS frequency <2Hz
# GPS Frequency status : Warning ; when the 2>= GPS frequency <6Hz 
# GPS Frequency status : Good ; when the GPS frequency >= 6Hz

#This script reports the following on Heading status:
# Heading status : Invalid ; Heading status = 0
# Heading status : Valid ; Heading status = 1

#This script reports the following on Heading frequency status:
# Heading Frequency status : Critical ; when the Heading frequency <2Hz
# Heading Frequency status : Warning ; when the 2>= Heading frequency <6Hz 
# Heading Frequency status : Good ; when the Heading frequency >= 6Hz

# Contact: Aswathi Muralidharan
# email:  aswathi.muralidharan@antobot.ai
########################################################################################################################################################

import threading
import asyncio
import time

import rclpy,rostopic
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8
from std_msgs.msg import Float64

import Jetson.GPIO as GPIO

from antobot_devices_gps.movingbase import MovingBase

class MovingBase_Ros:
    def __init__(self, base_port_uart, rover_port, base_port_spi, mode):
        
        self.base_port_uart = base_port_uart
        self.rover_port = rover_port
        self.base_port_spi = base_port_spi
        self.mode = mode
        
        self.device_connect = False
        self.publish_first = True
        self.freq_low = 2
        self.freq_high = 4

        self.time_buf_len = 10

        # Create the message for ROS
        self.gps_pub = rclpy.Publisher('antobot_gps', NavSatFix, queue_size=10)
        self.gpsfix = NavSatFix()
        self.gpsfix.header.stamp = rclpy.Time.now()
        self.gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
        self.gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED 
        self.gps_hz = 0 
        self.gps_freq_status = None
        self.gps_time_buf = []
        self.gps_fix_status = 0
        self.gps_fix_status_str = None
        self.h_acc = 500

        self.heading_pub = rclpy.Publisher('am_heading_urcu', Float64, queue_size=10)
        self.heading_pub_enu = rclpy.Publisher('am_heading_enu', Float64, queue_size=10)
        self.heading_hz = 0
        self.heading_freq_status = None
        self.heading_time_buf = []
        self.heading_status = False
        self.heading_status_str = None

    def pub_pvt(self, frame):

        if frame:
            self.gps_time_buf.append(time.time())

            self.gpsfix.latitude = frame.lat
            self.gpsfix.longitude = frame.lon 
            self.gpsfix.altitude = frame.height

            # Get GPS fix status
            self.gpsfix.status.status = frame.status_fix
            self.gps_fix_status = frame.status_fix

            # Assumptions made on covariance
            self.gpsfix.position_covariance[0] = (frame.hAcc*0.001)**2 
            self.gpsfix.position_covariance[4] = (frame.hAcc*0.001)**2 
            self.gpsfix.position_covariance[8] = (4*frame.hAcc*0.001)**2 
        
            if frame.hAcc < self.h_acc:
                self.gps_pub.publish(self.gpsfix)
        
        elif len(self.gps_time_buf) > 0:
            self.gps_time_buf.pop(0)
        elif len(self.gps_time_buf) == 0:
            self.gps_fix_status = 0
       

        if self.gps_fix_status == 0 and self.gps_fix_status_str != "Critical":
            rclpy.logerr("SN4010: GPS Fix Status: Crtitial")
            self.gps_fix_status_str = "Critical"
        elif self.gps_fix_status == 1 and self.gps_fix_status_str != "Warning":
            self.gps_fix_status_str = "Warning"
            rclpy.logwarn("SN4010: GPS Fix Status: Float Mode")
        elif self.gps_fix_status == 3 and self.gps_fix_status_str != "Good":
            self.gps_fix_status_str = "Good"
            rclpy.loginfo("SN4010: GPS Fix Status: Fixed Mode")

        if len(self.gps_time_buf) > self.time_buf_len:
            self.gps_time_buf.pop(0)

        if len(self.gps_time_buf)>2:
            self.gps_hz = (len(self.gps_time_buf)-1) / (time.time() - self.gps_time_buf[0])  
        else:
            self.gps_hz = 0

        if self.gps_hz < self.freq_low and self.gps_freq_status != "Critical":
            rclpy.logerr(f"SN4012: GPS Frequency status: Critical (<{self.freq_low} hz)")
            self.gps_freq_status = "Critical"
        elif self.gps_hz >= self.freq_low and self.gps_hz < self.freq_high and self.gps_freq_status != "Warning":
            rclpy.logwarn(f"SN4012: GPS Frequency status: Warning (<{self.freq_high} hz)")
            self.gps_freq_status = "Warning"
        elif self.gps_hz >= self.freq_high and self.gps_freq_status != "Good":
            rclpy.loginfo(f"SN4012: GPS Frequency status: Good (>{self.freq_high} hz) ")
            self.gps_freq_status = "Good"   

    def pub_head(self, frame):

        if frame:
            self.heading_time_buf.append(time.time())
            heading = frame.relPosHeading/100000
            self.heading_status = frame.relPosHeadingValid

            if self.heading_status:
                self.heading_pub.publish(heading) # True North heading - keep it for debugging

                # Convert the value to ENU (East-North-Up)
                enu_heading = (450.0 - heading) % 360.0

                self.heading_pub_enu.publish(enu_heading)
                
        elif len(self.heading_time_buf) > 0:
            self.heading_time_buf.pop(0)
        elif len(self.heading_time_buf) == 0:
            self.heading_status = False

        if self.heading_status is False and self.heading_status_str != "Invalid":
            rclpy.logerr("SN4020: Heading Status: Invalid")
            self.heading_status_str  = "Invalid"
        elif self.heading_status and self.heading_status_str != "Valid":
            rclpy.loginfo("SN4020: Heading Status: Valid")
            self.heading_status_str = "Valid"

        if len(self.heading_time_buf) > self.time_buf_len:
            self.heading_time_buf.pop(0)

        if len(self.heading_time_buf)>2:
            self.heading_hz = (len(self.heading_time_buf)-1) / (time.time() - self.heading_time_buf[0])  
        else:
            self.heading_hz = 0

        if self.heading_hz < self.freq_low and self.heading_freq_status != "Critical":
            rclpy.logerr(f"SN4022: Heading Frequency status: Critical (<{self.freq_low} hz)")
            self.heading_freq_status = "Critical"
        elif self.heading_hz >= self.freq_low and self.heading_hz < self.freq_high and self.heading_freq_status != "Warning":
            rclpy.logwarn(f"SN4022: Heading Frequency status: Warning (<{self.freq_high} hz)")
            self.heading_freq_status = "Warning"
        elif self.heading_hz >= self.freq_high and self.heading_freq_status != "Good":
            rclpy.loginfo(f"SN4022: Heading Frequency status: Good (>{self.freq_high} hz)")
            self.heading_freq_status = "Good"
    

    async def create_MovingBase(self):
        while not self.device_connect:
            try:
                MB = await MovingBase.create(self.base_port_uart, self.rover_port, self.base_port_spi, self.mode)
                self.device_connect = True
                return MB
            except Exception as e:
                if self.publish_first:
                    rclpy.logerr('MovingBase inilization failed: %s:', str(e))
                    self.publish_first = False
                time.sleep(1)
        
    async def main(self):
        MB = await self.create_MovingBase()
        while True:
            try:

                pvtFrame = MB.get_PVTframe()
                self.pub_pvt(pvtFrame)

                headFrame = await MB.get_RELPOSNEDframe()
                self.pub_head(headFrame)

            except:
                rclpy.logerr(f"nRTK: Close the nRTK Node")
                break

if __name__ == '__main__':
    
    rclpy.init_node("nRTK", disable_signals=True)
    
   
    # GPIO
    gpioID = 29
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(gpioID, GPIO.OUT)

    
    base_port_uart = rclpy.get_param("/gps/urcu/device_port","/dev/ttyTHS0")
    rover_port = rclpy.get_param("/gps/ublox_rover/device_port","/dev/AntoF9P")
    base_port_spi = None
    
    rtk_type = rclpy.get_param("/gps/urcu/rtk_type","ppp") # or "base_station"
    if rtk_type == "ppp":
        mode = 2
    else:
        mode = 1 #1: RTK base station, 2: PPP-IP

    movebase = MovingBase_Ros(base_port_uart, rover_port, base_port_spi, mode)
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(movebase.main())
    except Exception as e:     
        GPIO.cleanup()
        loop.close()
