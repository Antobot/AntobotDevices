#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # Code Description: This code is used to connect to ublox Things stream server to subscribe the GNSS augmentation data and implement PPP IP mode in ublox f9p chip. 
# This code should only be used in the Ublox chip : "ZED-F9P-04B-01" (>uRCUv1.2c) with firmware upgraded to HPG1.32.
#  The chip was connected to the Xavier via SPI. To run the script at 8Hz, the following modifications should be made in the f9p chip. (run gps_config.py first)
 #   CFG-RATE-MEAS = 125 sec
 #   VALSET: CFG-MSGOUT-NMEA-ID_GSV_UART1 = 0
 #   VALSET: CFG-MSGOUT-NMEA-ID_GSA_UART1 = 0
 #   VALSET: CFG-MSGOUT-NMEA-ID_GLL_UART1 = 0

#This script reports the following on GPS frequency status:
# GPS Frequency status : Critical ; when the GPS frequency <2Hz
# GPS Frequency status : Warning ; when the 2>= GPS frequency <6Hz 
# GPS Frequency status : Good ; when the GPS frequency >= 6Hz

# Contact: Aswathi Muralidharan
# email:  aswathi.muralidharan@antobot.ai
########################################################################################################################################################

#basic Python libraries
import ssl
import time
from datetime import datetime
import sys
import serial
import spidev

import yaml
import os

#MQTT dependencies
import paho.mqtt.client as mqtt
import json
#ros dependencies
import rclpy, rostopic, rospkg
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8
from gps_base_station import F9P_GPS
from antobot_devices_gps.ublox_gps import UbloxGps,sfeSpiWrapper

import Jetson.GPIO as GPIO

#import gpiod
#dependency:
#sudo apt install python3-libgpiod
#sudo chmod a+rw /dev/gpiochip0


parent_directory = os.path.dirname(os.path.abspath(__file__))
yaml_file_path = os.path.join(parent_directory, "../config/mqtt_config.yaml")

with open(yaml_file_path, 'r') as file:
    config = yaml.safe_load(file)
    device_ID = config['device_ID']
    base_ID = config['base_ID']
    mqtt_Broker = config['mqtt_Broker']
    mqtt_Port = config['mqtt_Port']
    mqtt_keepalive = config['mqtt_keepalive']
    mqtt_UserName = config['mqtt_UserName']
    mqtt_PassWord = config['mqtt_PassWord']


class nRTK:
    
    def __init__(self, baud, dev_port, device_id, base_id, mqtt_broker, mqtt_port, mqtt_keepalive, mqtt_username, mqtt_password):
        self.echo = True
        self.latitude = 0.0
        self.longitude = 0.0
        self.fix_quality = 0
        self.satellites = 0
        self.horizontal_dilution = 0
        self.altitude_m = 0
        self.height_geoid = 0
        #self.gpio_value = 2
        #self.baudrate = 460800

        self.client_id = device_id
        self.broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_keepalive = mqtt_keepalive
        self.connect = False
        
        #self.gnss = serial.Serial(port='/dev/ttyUSB0', baudrate=460800, timeout=0.1) #baudrate=38400
        #making the SPI configuration:
        
        # Hardware configurations
        #/dev/ttyTHS1: UART0
        #/dev/ttyTHS0: UART1
        #/dev/ttyTCU0: UART2

        #UART1 with GPIO01 high = module ZED-F9P gets PPP IP correction from xavier
        self.nRTK_correction = serial.Serial(port=dev_port, baudrate=baud)  #38400
        self.spiport = sfeSpiWrapper()
        self.gpio01 = 29
        self.GPIO = GPIO
        self.GPIO.setmode(GPIO.BOARD)
        self.GPIO.setup(self.gpio01, GPIO.OUT)
        #self.GPIO.output(self.gpio01, GPIO.HIGH) # if need 20240424

        # Topic names and QoS
        self.mqtt_topic_sub = base_id
        self.mqtt_topic_pub = "Antobot_robot_gps"
        self.client = mqtt.Client(self.client_id)
        self.client.username_pw_set(mqtt_username, mqtt_password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.connect_broker()

        #To check the frequency of the GPS topic
        #self.timer = rclpy.Timer(rclpy.Duration(1.0), self.frequency_check)
        self.gps_hz = 0.0
        self.gps_hz_status = "None"
        self.msg_count = 0
        self.first_message = True
        self.last_time = rclpy.get_time()

        self.gps_time_buf = []

    #call back when the broker is connected
    def connect_broker(self):
        try:
            self.client.connect(self.broker, self.mqtt_port, self.mqtt_keepalive)
        except:
            print("Trying to connect ...")
        time.sleep(2)
        
    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self,client, userdata, flags, rc):
        if rc == 0:
            self.client.subscribe(self.mqtt_topic_sub)
            self.connect = True
        else:
            print("Connection failed!", rc)

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self,client,userdata, msg):
        # write the corrections via UART2
        #print(msg.payload)
        data = self.nRTK_correction.write(msg.payload)
        #data = self.spartn_key(msg.payload) #to test if the GPS is working by writting the corrections via SPI

    def spartn_key(self, key):
        # # # DEPRECATED # # #
        # # # This function writes the key it subscribed from the MQTT broker to the ZED-F9P via SPI
        # # # This was used initially, but during the reconnection of 4G, writing the SPARTN key via SPI blocks entire SPI communication.
        # # # So this function is not used currently

        #print("Key received: ",len(key),key,type(key))
        data = self.spiport.write(key) 
        #print("Status of data write",data)
        received_bytes = self.spiport.read()
        #print("Received bytes from set_baudrate",ubx_str)
        #self.check_ubx_uart(received_bytes)
        #print("key passed",received_bytes)
    def send_gps_data(self, gps_data):
        self.client.publish(topic=self.mqtt_topic_pub, payload=gps_data) 


def main(args):
    
    rclpy.init_node ('nRTK')

    dev_port = "/dev/ttyTHS0"
    
    device_id = "Antobot_device_" + device_ID
    base_id = "Anto_MQTT_F9P_" + base_ID

    mqtt_broker = mqtt_Broker
    mqtt_port = mqtt_Port
    mqtt_keepalive = mqtt_keepalive
    mqtt_username = "antobot_device"
    mqtt_password = "SvHCWJFNP98h"     


    rate = rclpy.Rate(8)
    
    baudrate = 460800

    #Publisher to publish nRTK
    gps_pub = rclpy.Publisher('antobot_gps', NavSatFix, queue_size=10) # am_nRTK_urcu
    gps_f9p = F9P_GPS()
    gps_f9p.uart2_config(baudrate)

    nRTK_node = nRTK(baudrate, dev_port, device_id, base_id, mqtt_broker, mqtt_port, mqtt_keepalive, mqtt_username, mqtt_password)
    nRTK_node.client.loop_start() # MQTT broker for nRTK messages from PPP

    mode = 2 #1: RTK base station, 2: PPP-IP, 3: LBand

    # Create the message for ROS
    gpsfix = NavSatFix()
    gpsfix.header.stamp = rclpy.Time.now()
    gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
    gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED        

    # Begin without a specific GPS status'
    gps_status = 'None'
    gps_freq_status = "None"

    # Set the horizontal accuracy limit (in mm)
    h_acc = 75 

    # Pulish the gps data
    b_gps_data_pub = True
    count_gps_data_pub = 0
    gps_data = {
        'time': None,
        'lat': None,
        'lon': None,
        'alt': None,
        'status': None,
        'frequency': None
    }

    while not rclpy.is_shutdown():

        #GPIO.output(nRTK_node.gpio01,nRTK_node.GPIO.HIGH)

        # Get the GPS data from the F9P
        gps_f9p.get_gps()
  
        # Check the new data is viable and update message
        if gps_f9p.geo.lat is not None and gps_f9p.geo.lat != 0:                 
            gpsfix.latitude = gps_f9p.geo.lat 
            gpsfix.longitude = gps_f9p.geo.lon 
            gpsfix.altitude = gps_f9p.geo.height
            
            # Get GPS fix status
            gpsfix.status.status = gps_f9p.get_fix_status()

            # Assumptions made on covariance
            gpsfix.position_covariance[0] = (gps_f9p.geo.hAcc*0.001)**2 
            gpsfix.position_covariance[4] = (gps_f9p.geo.hAcc*0.001)**2 
            gpsfix.position_covariance[8] = (4*gps_f9p.geo.hAcc*0.001)**2 

            # Update the navsatfix messsage
            current_time = rclpy.Time.now()
            gps_time_i=(current_time.to_sec()-gpsfix.header.stamp.to_sec())
            gpsfix.header.stamp = current_time

            # Create a buffer to find the average frequency
            time_buf_len = 10
            nRTK_node.gps_time_buf.append(gps_time_i)
            if len(nRTK_node.gps_time_buf) > time_buf_len:
                nRTK_node.gps_time_buf.pop(0)

            # Inverted average time to calculate hertz
            gps_hz = len(nRTK_node.gps_time_buf) / sum(nRTK_node.gps_time_buf)        

            if gps_f9p.geo.hAcc < 500:
                gps_pub.publish(gpsfix)

            #rclpy.loginfo(f'GPS Frequency: {self.gps_hz} Hz')
            if gps_hz < 2 and gps_freq_status != "Critical":
                rclpy.logerr("SN4012: GPS Frequency status: Critical (<2 hz)")
                gps_freq_status = "Critical"
            elif gps_hz >=2 and gps_hz < 6 and gps_freq_status != "Warning":
                rclpy.logwarn("SN4012: GPS Frequency status: Warning (<6 hz)")
                gps_freq_status = "Warning"
            elif gps_hz >= 6 and gps_freq_status != "Good":
                rclpy.loginfo("SN4012: GPS Frequency status: Good (>6 hz)")
                gps_freq_status = "Good"

            
            
            if b_gps_data_pub:
                count_gps_data_pub += 1
                
                current_time = datetime.now()
                gps_data['time'] = current_time.strftime("%Y-%m-%d %H:%M:%S")
                gps_data['lat'] = "{:.7f}".format(gps_f9p.geo.lat)
                gps_data['lon'] = "{:.7f}".format(gps_f9p.geo.lon)
                gps_data['alt'] = gps_f9p.geo.height
                gps_data['status'] = gps_f9p.get_fix_status()
                gps_data['frequency'] = "{:.2f}".format(gps_hz)
                if gps_hz < 2:
                    count_gps_data_pub = 0
                    nRTK_node.send_gps_data(json.dumps(gps_data))
                elif gps_hz >=2 and gps_hz < 6 and count_gps_data_pub % 3 == 0:
                    count_gps_data_pub = 0
                    nRTK_node.send_gps_data(json.dumps(gps_data))
                elif gps_hz >= 6 and count_gps_data_pub % 6 == 0:
                    count_gps_data_pub = 0
                    nRTK_node.send_gps_data(json.dumps(gps_data))
                    
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
