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


class gpsCorrections():
    def __init__(self, dev_type, corr_type, serial_port=None):
        # # # Initialisation of GPS corrections class
        #     Inputs: dev_type - the type of device that this script is running on
        #           "urcu" - the URCU; Jetson-based packages should be used
        #           "rasPi" - antoScout or another rasPi-based system; rasPi-compatible packages should be used

        # Importing device-specific packages
        if dev_type == "urcu":
            import serial
            import Jetson.GPIO as GPIO
        elif dev_type == "rasPi":
            print("importing rasPi specific packages!")
        
        self.corr_type = corr_type

        # Reading configuration file
        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_devices_gps')
        print("packagePath: {}".format(packagePath))
        yaml_file_path = packagePath + "/config/corrections_config.yaml"
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)
            self.ppp_client_id = config['ppp']['device_ID']
            self.ppp_server = 'pp.services.u-blox.com'

            self.ant_client_id = "anto_rtk_" + config['ant_mqtt']['robot_ID']
            self.ant_mqtt_topic_sub = "AntoCom/02/" + config['ant_mqtt']['base_ID'] + "/00"
            self.ant_broker = config['ant_mqtt']['mqtt_Broker']
            self.ant_mqtt_port = config['ant_mqtt']['mqtt_Port']
            self.mqtt_keepalive = config['ant_mqtt']['mqtt_keepalive']
            mqtt_username = config['mqtt_UserName']
            mqtt_password = config['mqtt_PassWord']

        # Setting up hardware ports
        if serial_port == None:
            dev_port = "/dev/ttyTHS0"
            baud = 460800
            self.serial_port = serial.Serial(port=dev_port, baudrate=baud)  #38400
            # May need a different option for antoScout
        else:
            self.serial_port = serial_port

        # Configuring method-specific parameters (MQTT)
        if self.corr_type == "ppp":
            dev_port = rospy.get_param("/gps/urcu/device_port","/dev/ttyTHS0")
            self.mqtt_topics = [(f"/pp/ip/eu", 0), ("/pp/ubx/mga", 0), ("/pp/ubx/0236/ip", 0)]
            self.userdata = { 'gnss': self.serial_port, 'topics': self.mqtt_topics } 
        elif self.corr_type == "ant_mqtt":
            self.connect = False

        # If the uRCU is being used, the appropriate GPIO pin must be set to "high" to enable corrections from Xavier 
        if dev_type == "urcu":
            # Set the GPIO pin of the URCU high
            self.gpio01 = 29
            self.GPIO = GPIO
            self.GPIO.setmode(GPIO.BOARD)
            self.GPIO.setup(self.gpio01, GPIO.OUT)
            #self.GPIO.output(self.gpio01, GPIO.HIGH) # if need 20240424

        # Setting up the MQTT Client
        if self.corr_type == "ppp":
            self.client = mqtt.Client(client_id=self.ppp_client_id, userdata=self.userdata)
            self.certfile=os.path.join(packagePath,"config/")+f'device-{self.ppp_client_id}-pp-cert.crt'
            self.keyfile=os.path.join(packagePath,"config/")+f'device-{self.ppp_client_id}-pp-key.pem'
            self.client.tls_set(certfile=self.certfile,keyfile=self.keyfile)
        elif self.corr_type == "ant_mqtt":
            self.client = mqtt.Client(self.ant_client_id)
            self.client.username_pw_set(mqtt_username, mqtt_password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.connect_broker()
            


    # Attempts to connect to the broker
    def connect_broker(self):
        try:
            if self.corr_type == "ppp":
                self.client.connect(self.ppp_server, port=8883)
            elif self.corr_type == "ant_mqtt":
                self.client.connect(self.ant_broker, self.ant_mqtt_port, self.mqtt_keepalive)
        except:
            print("Trying to connect ...")

        time.sleep(2)
        
    # The callback for when the client receives a CONNECT response from the server.
    def on_connect(self,client, userdata, flags, rc):
        
        if rc == 0:
            self.connect = True
            if self.corr_type == "ppp":
                self.client.subscribe(userdata['topics'])
            elif self.corr_type == "ant_mqtt":
                self.client.subscribe(self.ant_mqtt_topic_sub)
        else: 
            print("Connection failed!", rc)

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self,client,userdata, msg):
        # write the corrections via UART2
        if self.corr_type == "ppp":
            data = userdata['gnss'].write(msg.payload)
        elif self.corr_type == "ant_mqtt":
            data = self.serial_port.write(msg.payload)


def main():
    
    gps_corr = gpsCorrections("urcu", "ppp") # "ppp" or "anto_mqtt"

    gps_corr.client.loop_start()
    while(True):
        time.sleep(1)
        #print("Running")


if __name__ == '__main__':
    main()
    
