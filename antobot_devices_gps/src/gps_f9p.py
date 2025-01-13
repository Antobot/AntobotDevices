#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     The purpose of this code is to process the GPS data received via SPI from the Ublox F9P chip
# # #                       and publish a GPS message as a rostopic using this data.

#This script reports the following on GPS status:
# GPS status : Critical ; GPS status = 0
# GPS status : Warning ;GPS status = 1 - Float
# GPS status : Good ; GPS status = 3 - Fix

# Contact: Daniel Freer 
# email: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import rospy
import rospkg
import spidev
import sys
import time
import pynmea2

import serial

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8, Float32
from antobot_devices_gps.ublox_gps import UbloxGps

class F9P_GPS:

    def __init__(self, dev_type, serial_port=None, pub_name="antobot_gps"):
        # # # GPS class initialisation
        #     Inputs: dev_type - the device type of the F9P chip. 
        #           "urcu" - if using the F9P inside of the URCU
        #           "usb" - if using an external F9P conncected via USB

        self.node_type = "gps_f9p"

        self.gpsfix = NavSatFix()
        self.gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
        
        self.dev_type = dev_type
        if self.dev_type == "urcu":
            self.port = spidev.SpiDev()
        elif self.dev_type == "usb":
            if serial_port == None:
                self.baud = 460800  # 38400?? Need to resolve baudrate difference with baudrate_rtk below
                self.port = serial.Serial("/dev/USB_0", self.baud)
            else:
                self.port = serial_port
        self.gps_dev = UbloxGps(self.port)
        self.geo = None
        self.fix_status = 0
        self.gps_status = "Critical"
        self.gps_freq_status = "Critical"
        self.gps_time_buf = []
        self.hAcc = 500
        self.h_acc_thresh = 75  # 

        current_time = rospy.Time.now()
        self.gps_time_i=(current_time.to_sec()-self.gpsfix.header.stamp.to_sec())

        self.gps_pub = rospy.Publisher(pub_name, NavSatFix, queue_size=10)

        return


    def uart2_config(self,baud):
        #set the baud rate of uart2 to appropriate value (38400?)
        self.gps_dev.ubx_set_val(0x40530001,baud)
        #set the uart2 enable true
        self.gps_dev.ubx_set_val(0x10530005,0x01) #cfg-uart2-enable

        
    def get_gps(self):
        # Get the data from the F9P
        #self.geo = self.gps_dev.geo_coords() #poll method
        streamed_data = self.gps_dev.stream_nmea() #stream method

        self.get_gps_quality(streamed_data)

        # Check the new data is viable and update message
        if self.correct_gps_format(streamed_data):                
            
            self.create_gps_msg()
            self.get_gps_freq()   

            # if self.hAcc < 1:
            #     self.gps_pub.publish(self.gpsfix)
            
            #     if self.mqtt_publish:
            #         print("Publishing GPS data to MQTT")    # TODO: Add this functionality
    

    def correct_gps_format(self, streamed_data):
        # Function to check whether the streamed data matches the desired 
                        
        if isinstance(streamed_data,str) and streamed_data.startswith("$GNGGA"):
            self.geo = pynmea2.parse(streamed_data)
            print(self.geo)
            return True

        return False

    def get_fix_status(self):

        if self.geo.gps_qual == 4 and self.gps_status != 'Good':
            rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
            self.gps_status = 'Good'
            self.fix_status = 3
        elif self.geo.gps_qual == 2 or 5:
            if self.hAcc < self.h_acc_thresh:
                self.gpsfix.status.status = 3
                if self.gps_status != 'Good':
                    rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
                    self.gps_status = 'Good'
            else:   
                self.gpsfix.status.status = 1
                if self.gps_status != 'Warning':
                    rospy.logwarn("SN4010: GPS Fix Status: Float Mode")
                    self.gps_status = 'Warning'
        else:
            self.fix_status = 0 #no fix
            if self.gps_status != 'Critical':
                rospy.logerr("SN4010: GPS Fix Status: Critical")
                self.gps_status = 'Critical'
        
        return self.fix_status

    def create_gps_msg(self):

        self.gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.gpsfix.altitude = 0

        self.gpsfix.latitude = self.geo.latitude if self.geo.lat_dir == 'N' else - self.geo.latitude
        self.gpsfix.longitude = self.geo.longitude if self.geo.lon_dir == 'E' else - self.geo.longitude
        self.gpsfix.altitude = self.geo.altitude
        
        # Get GPS fix status
        self.gpsfix.status.status = self.get_fix_status()

        # Assumptions made on covariance
        self.gpsfix.position_covariance[0] = (self.hAcc)**2 
        self.gpsfix.position_covariance[4] = (self.hAcc)**2 
        self.gpsfix.position_covariance[8] = (4*self.hAcc)**2 

        # Update the navsatfix messsage
        current_time = rospy.Time.now()
        self.gps_time_i=(current_time.to_sec()-self.gpsfix.header.stamp.to_sec())
        self.gpsfix.header.stamp = current_time
        self.gpsfix.header.stamp = self.geo.timestamp

        return

    def get_gps_freq(self):
        # # # Gets the frequency of the published GPS message and sends a message if there has been a significant change

        # Create a buffer to find the average frequency
        time_buf_len = 10
        self.gps_time_buf.append(self.gps_time_i)
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

        if isinstance(streamed_data,str):
            if streamed_data.startswith("$GNGST"):
                gst_parse = pynmea2.parse(streamed_data)
                self.hAcc=((gst_parse.std_dev_latitude)**2+(gst_parse.std_dev_longitude)**2)**0.5
            if streamed_data.startswith("$GNGGA"):
                gga_parse = pynmea2.parse(streamed_data)
                self.gga_gps_qual = int(gga_parse.gps_qual)
                self.num_sats = int(gga_parse.num_sats)         # Number of satellites
                self.hor_dil = float(gga_parse.horizontal_dil)  # Horizontal dilution of precision (HDOP)
                self.geo_sep = float(gga_parse.geo_sep)         # Geoid separation
            if streamed_data.startswith("$GNGSA"):      # Full satellite information
                gsa_parse = pynmea2.parse(streamed_data)
                # Add parser here (?)
            if streamed_data.startswith("$GNVTG"):      # Velocity
                vtg_parse = pynmea2.parse(streamed_data)
                self.cogt = vtg_parse.cogt          # Course over ground (true)
                self.cogm = vtg_parse.cogm          # Course over ground (magnetic)
                self.sogn = vtg_parse.sogn          # Speed over ground (knots)
                self.sogk = vtg_parse.sogk          # Speed over ground (km/h)

                # Calculate ENU velocity from cogt or cogm and 
            
        return


## Example function

def main(args):
    mqtt_publish = False

    # init node
    rospy.init_node('rtk', anonymous=True)
    rate = rospy.Rate(50)  # 8hz
    gps_f9p = F9P_GPS("urcu")

    baudrate_rtk = 38400            # Need to resolve baudrate
    gps_f9p.uart2_config(baudrate_rtk)

    mode = 1 # 1: RTK base station; 2: PPP-IP; 3: LBand
    while not rospy.is_shutdown():
        gps_f9p.get_gps()

        rate.sleep()



if __name__ == '__main__':   
    main(sys.argv)

