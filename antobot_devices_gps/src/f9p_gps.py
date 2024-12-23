#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     The purpose of this code is to process the GPS data received via SPI from the Ublox F9P chip
# # #                       and publish a GPS message as a rostopic using this data.
# # #                       This script is used when base station is used to get the 433MHz correction messages

#This script reports the following on GPS status:
# GPS status : Critical ; GPS status = 0
# GPS status : Warning ;GPS status = 1 - Float
# GPS status : Good ; GPS status = 3 - Fix

# Contact: Aswathi Muralidharan 
# email: aswathi.muralidharan@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import rospy
import rospkg
import spidev
import sys
import time
import pynmea2
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8, Float32
from antobot_devices_gps.ublox_gps import UbloxGps

class F9P_GPS:

    def __init__(self):
        #GPS class initialisation 
        #self.baud = 460800# 38400
        self.port = spidev.SpiDev()
        self.gps_spi = UbloxGps(self.port)
        self.geo = None
        self.fix_status = 0
        self.gps_status = "Critical"
        self.gps_freq_status = "Critical"
        self.gps_time_buf = []

        self.h_acc_thresh = 75  # 


    def uart2_config(self,baud):
        #set the baud rate of uart2 to 38400
        self.gps_spi.ubx_set_val(0x40530001,baud)
        #set the uart2 enable true
        self.gps_spi.ubx_set_val(0x10530005,0x01) #cfg-uart2-enable

        
    def get_gps(self, streamed_data):
        # Function to check whether the streamed data matches the desired 
                        
        if isinstance(streamed_data,str) and streamed_data.startswith("$GNGGA"):
            self.geo = pynmea2.parse(streamed_data)
            return True

        return False


    def get_fix_status(self):

        if isinstance(self.geo,str) and self.geo.startswith("$GNGGA"):
            if self.geo.gps_qual == 4 and self.gps_status != 'Good':
                rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
                self.gps_status = 'Good'
                self.fix_status = 3
            elif self.geo.gps_qual == 2 or 5:
                if self.hAcc < self.h_acc_thresh:
                    gpsfix.status.status = 3
                    if gps_status != 'Good':
                        rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
                        gps_status = 'Good'
                else:   
                    gpsfix.status.status = 1
                    if gps_status != 'Warning':
                        rospy.logwarn("SN4010: GPS Fix Status: Float Mode")
                        gps_status = 'Warning'
            else:
                self.fix_status = 0 #no fix
                if self.gps_status != 'Critical':
                    rospy.logerr("SN4010: GPS Fix Status: Critical")
                    self.gps_status = 'Critical'
        
        return self.fix_status

    def create_gps_msg(self, gpsfix):

        gpsfix = NavSatFix()
        gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
        gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        gpsfix.latitude = self.geo.latitude if gps_f9p.geo.lat_dir == 'N' else - self.geo.latitude
        gpsfix.longitude = self.geo.longitude if gps_f9p.geo.lon_dir == 'E' else - self.geo.longitude
        gpsfix.altitude = float(self.geo.altitude)
        
        # Get GPS fix status
        gpsfix.status.status = gps_f9p.get_fix_status()

        # Assumptions made on covariance
        gpsfix.position_covariance[0] = (float(gps_f9p.geo.horizontal_dil)*0.1*0.001)**2 
        gpsfix.position_covariance[4] = (float(gps_f9p.geo.horizontal_dil)*0.1*0.001)**2 
        gpsfix.position_covariance[8] = (4*gps_f9p.geo.horizontal_dil*0.1*0.001)**2 

        # Update the navsatfix messsage
        current_time = rospy.Time.now()
        gps_time_i=(current_time.to_sec()-gpsfix.header.stamp.to_sec())
        gpsfix.header.stamp = current_time

        return gpsfix

    def get_gps_freq(self):
        # # # Gets the frequency of the published GPS message and sends a message if there has been a significant change

        # Create a buffer to find the average frequency
        time_buf_len = 10
        self.gps_time_buf.append(gps_time_i)
        if len(self.gps_time_buf) > time_buf_len:
            self.gps_time_buf.pop(0)

        # Inverted average time to calculate hertz
        gps_hz = len(self.gps_time_buf) / sum(self.gps_time_buf)

        #rospy.loginfo(f'GPS Frequency: {self.gps_hz} Hz')
        if gps_hz < 2 and self.gps_freq_status != "Critical":
            rospy.logerr("SN4012: GPS Frequency status: Critical (<2 hz)")
            self.gps_freq_status = "Critical"
        elif gps_hz >=2 and gps_hz < 6 and self.gps_freq_status != "Warning":
            rospy.logwarn("SN4012: GPS Frequency status: Warning (<6 hz)")
            self.gps_freq_status = "Warning"
        elif gps_hz >= 6 and self.gps_freq_status != "Good":
            rospy.loginfo("SN4012: GPS Frequency status: Good (>6 hz)")
            self.gps_freq_status = "Good" 

    def get_gps_quality(self, streamed_data):

        if isinstance(streamed_data,str) and streamed_data.startswith("$GNGST"):
            gst_parse = pynmea2.parse(streamed_data)
            self.hAcc=((gst_parse.std_dev_latitude)**2+(gst_parse.std_dev_longitude)**2)**0.5


        return


## Example function

def main(args):
    gps_freq_status = "Critical"

    # init node
    rospy.init_node('rtk', anonymous=True)
    rate = rospy.Rate(50)  # 8hz
    gps_f9p = F9P_GPS()

    baudrate_rtk = 38400
    gps_f9p.uart2_config(baudrate_rtk)

    gps_pub = rospy.Publisher('antobot_gps', NavSatFix, queue_size=10)

    mode = 1 # 1: RTK base station; 2: PPP-IP; 3: LBand
    while not rospy.is_shutdown():
        # Get the data from the F9P
        #self.geo = self.gps_spi.geo_coords() #poll method
        streamed_data = self.gps_spi.stream_nmea() #stream method

        gps_f9p.get_gps_quality(streamed_data)

        # Check the new data is viable and update message
        if gps_f9p.get_gps(streamed_data):                
            
            gpsfix = gps_f9p.create_gps_msg()
            gps_f9p.get_gps_freq()   

            if gps_f9p.geo.horizontal_dil < 1:
                gps_pub.publish(gpsfix)

        

        rate.sleep()



if __name__ == '__main__':   
    main(sys.argv)

