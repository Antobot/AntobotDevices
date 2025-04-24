#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     This code receives GPS correction messages from a variety of sources (usually MQTT) and writes 
# # #                       the corrections to the F9P chip.

# Contact: Daniel Freer 
# email: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


import os
import time
import yaml
import paho.mqtt.client as mqtt
import rospy, rospkg
import importlib
import serial
import socket
import base64
from std_msgs.msg import  String
import threading


class gpsCorrections():
    def __init__(self, dev_type=None, corr_type="ppp", serial_port=None):
        # # # Initialisation of GPS corrections class
        #     Inputs: dev_type - the type of device that this script is running on
        #           "urcu" - the URCU; Jetson-based packages should be used
        #           "rasPi" - antoScout or another rasPi-based system; rasPi-compatible packages should be used


        self.sock = None
        self.corr_type = corr_type
        self.gga_interval=10
        self.latest_gga = None
        self.sub_gga = rospy.Subscriber("/antobot_gps/gga",String,self.gga_callback)
        self.count=0
        self.running = True
        # Reading configuration file
        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_description')
        path = packagePath + "/config/platform_config.yaml"

        with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            dev_type = data['gps'].keys()
        print(dev_type)
        if "urcu" in dev_type :
            print("in urcu type")
            GPIO = importlib.import_module("Jetson.GPIO") 
            dev_port = "/dev/ttyTHS0"
            baud = 460800
            self.serial_port = serial.Serial(port=dev_port, baudrate=baud)  #38400
        elif "f9p_usb" in dev_type:
            self.serial_port = serial.Serial(port="/dev/ttyUSB0", baudrate=38400)  #460800

            
        packagePath=rospack.get_path('antobot_devices_gps')
        print("packagePath: {}".format(packagePath))
        yaml_file_path = packagePath + "/config/corrections_config.yaml"
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)
            self.ntrip_server = config['ntrip']['address']
            self.ntrip_port = config['ntrip']['port']
            self.ntrip_username = config['ntrip']['username']
            self.ntrip_password = config['ntrip']['password']
            self.ntrip_mountpoint = config['ntrip']['mountpoint']

        # Configuring method-specific parameters (MQTT)
        if self.corr_type == "ppp":
            dev_port = rospy.get_param("/gps/urcu/device_port","/dev/ttyUSB0")
            self.mqtt_topics = [(f"/pp/ip/eu", 0), ("/pp/ubx/mga", 0), ("/pp/ubx/0236/ip", 0)]
            self.userdata = { 'gnss': self.serial_port, 'topics': self.mqtt_topics } 
        elif self.corr_type == "ant_mqtt":
            self.connect = False
        

        # If the uRCU is being used, the appropriate GPIO pin must be set to "high" to enable corrections from Xavier 
        if "urcu" in dev_type:
            # Set the GPIO pin of the URCU high
            self.gpio01 = 29
            self.GPIO = GPIO
            self.GPIO.setmode(GPIO.BOARD)
            self.GPIO.setup(self.gpio01, GPIO.OUT)
            #self.GPIO.output(self.gpio01, GPIO.HIGH) # if need 20240424

    def make_ntrip_request(self):
        auth = base64.b64encode(f"{self.ntrip_username}:{self.ntrip_password}".encode()).decode()
        request = (
            f"GET /{self.ntrip_mountpoint} HTTP/1.1\r\n"
            f"Host: {self.ntrip_server}:{self.ntrip_port}\r\n"
            "User-Agent: NTRIP u-blox client\r\n"
            f"Authorization: Basic {auth}\r\n"
            "Connection: close\r\n"
            "\r\n"
        )
        return request.encode()

    def connect_ntrip(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ntrip_server, self.ntrip_port))
        self.sock.send(self.make_ntrip_request())
        print("[INFO] Connected to NTRIP server")
   
    def stream_corrections(self,event=None):
        print("[INFO] Streaming corrections...")
        data = self.sock.recv(1024)
        if not data:
            print("[WARN] NTRIP server closed connection")
            self.connect_ntrip()
        #print(data)
        self.serial_port.write(data)
    
    def gga_callback(self,data):
        self.latest_gga=data.data
        return

    def send_gga(self,event=None):
        
        print("in send gga")
        """ Periodically send latest GGA to NTRIP server """ 
        if self.latest_gga:
            try:

                self.sock.send(self.latest_gga.encode())
                print(f"[INFO] Sent GGA: {self.latest_gga.strip()}")
            except Exception as e:
                print(f"[WARN] Failed to send GGA: {e}")
                #self.running=False
                self.connect_ntrip()

    def close(self):
        self.running=False
        if self.serial_port:
            self.serial_port.close()
        if self.sock:
            self.sock.close()
        print("[INFO] Closed connections")


def main():
    rospy.init_node ('gps_correction')
    gps_corr = gpsCorrections()
    rate=rospy.Rate(0.1)
    try:
        gps_corr.connect_ntrip()
        rospy.Timer(rospy.Duration(0.1), gps_corr.stream_corrections)
        rospy.Timer(rospy.Duration(10), gps_corr.send_gga)
        rospy.spin() 

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        gps_corr.close()
        pass

if __name__ == '__main__':
    main()
    
