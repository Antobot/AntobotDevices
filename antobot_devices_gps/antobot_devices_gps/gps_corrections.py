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
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import importlib
import socket
import base64
import threading
import serial
from std_msgs.msg import  String
from antobot_devices_msgs.msg import GpsQual


class gpsCorrections(Node):
    def __init__(self):
        # # # Initialisation of GPS corrections class
        #     Inputs: dev_type - the type of device that this script is running on
        #           "urcu" - the URCU; Jetson-based packages should be used
        #           "rasPi" - antoScout or another rasPi-based system; rasPi-compatible packages should be used

        super().__init__('gps_corrections')
        self.gga_interval=10
        self.latest_gga = None
        self.sub_gga = self.create_subscription(String,"/antobot_gps/gga",self.gga_callback,10)
        self.sub_gga
        self.time_offset = self.create_subscription(GpsQual,"/antobot_gps/quality", self.time_offset_callback,10)
        self.time_offset
        self.count=0
        self.running = True
        self.sent_time = self.get_clock().now()
        # Reading configuration file
        packagePath = get_package_share_directory('antobot_description')
        path = packagePath + "/config/platform_config.yaml"

        with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            dev_type = data['gps'].keys()
            for key, value in data['gps'].items():
                #print(key)
                self.corr_type=value['rtk_type']
                dev_port = value['device_port']
        # Importing device-specific packages
        print(dev_type)
        if "urcu" in dev_type :
            print("in urcu type")
            GPIO = importlib.import_module("Jetson.GPIO") 
            dev_port = "/dev/ttyTHS1"
            baud = 460800
            self.serial_port = serial.Serial(port=dev_port, baudrate=baud)  #38400 460800
        elif "f9p_usb" in dev_type:
            self.serial_port = serial.Serial(port="/dev/ttyUSB4", baudrate=460800)
        elif "rasPi" in dev_type:
            print("import rasPi specific packages!")

            
        packagePath = get_package_share_directory('antobot_devices_gps')
        print("packagePath: {}".format(packagePath))
        yaml_file_path = packagePath + "/config/corrections_config.yaml"
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)
            if self.corr_type == "ppp":
                self.ppp_client_id = config['ppp']['device_ID']
                self.ppp_server = 'pp.services.u-blox.com'
            elif self.corr_type == "ant_mqtt":
                self.ant_client_id = "anto_rtk_" + config['ant_mqtt']['robot_ID']
                self.ant_mqtt_topic_sub = "AntoCom/02/" + config['ant_mqtt']['baseStation_ID'] + "/00"
                self.ant_broker = config['ant_mqtt']['mqtt_Broker']
                self.ant_mqtt_port = config['ant_mqtt']['mqtt_Port']
                self.mqtt_keepalive = config['ant_mqtt']['mqtt_keepalive']
                mqtt_username = config['ant_mqtt']['mqtt_UserName']
                mqtt_password = config['ant_mqtt']['mqtt_PassWord']
            elif self.corr_type == "ntrip":
                self.ntrip_server = config['ntrip']['address']
                self.ntrip_port = config['ntrip']['port']
                self.ntrip_username = config['ntrip']['username']
                self.ntrip_password = config['ntrip']['password']
                self.ntrip_mountpoint = config['ntrip']['mountpoint']




        # Configuring method-specific parameters (MQTT)
        if self.corr_type == "ant_mqtt":
            self.connect = False
        elif self.corr_type == "ppp":
            dev_port = self.get_parameter_or("/gps/urcu/device_port",Parameter("/gps/urcu/device_port",value="/dev/ttyUSB0")).value
            self.mqtt_topics = [(f"/pp/ip/eu", 0), ("/pp/ubx/mga", 0), ("/pp/ubx/0236/ip", 0)]
            self.userdata = { 'gnss': self.serial_port, 'topics': self.mqtt_topics } 
        

        # If the uRCU is being used, the appropriate GPIO pin must be set to "high" to enable corrections from Xavier 
        #if "urcu" in dev_type:
        #    # Set the GPIO pin of the URCU high
        #    self.gpio01 = 29
        #    self.GPIO = GPIO
        #    self.GPIO.setmode(GPIO.BOARD)
        #    self.GPIO.setup(self.gpio01, GPIO.OUT)
        #    #self.GPIO.output(self.gpio01, GPIO.HIGH) # if need 20240424


###added for check

 # Setting up the MQTT Client
        if self.corr_type != "ntrip":
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

            self.last_receive_time = time.time()
            self.timer = self.create_timer(30.0, self.check_RTCM_timeout) 
            


    # Attempts to connect to the broker
    def connect_broker(self):
        try:
            if self.corr_type == "ppp":
                self.client.connect(self.ppp_server, port=8883)
            elif self.corr_type == "ant_mqtt":
                self.client.connect(self.ant_broker, self.ant_mqtt_port, self.mqtt_keepalive)
            # rospy.loginfo("SN4500: Connected to MQTT broker successfully.")
        except Exception as e:
            self.get_logger().error("SN4500: Connected to MQTT broker failed. ({e})")

        time.sleep(2)
        
    # The callback for when the client receives a CONNECT response from the server.
    def on_connect(self,client, userdata, flags, rc):
        
        if rc == 0:
            self.connect = True
            if self.corr_type == "ppp":
                self.client.subscribe(userdata['topics'])
            elif self.corr_type == "ant_mqtt":
                self.client.subscribe(self.ant_mqtt_topic_sub)
            self.get_logger().info("SN4500: Connected to broker successfully")
        else: 
            self.get_logger().error("SN4500: Connected to broker failed. (rc = {rc})")

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self,client,userdata, msg):
        # write the corrections via UART2
        try:
            if self.corr_type == "ppp":
                data = userdata['gnss'].write(msg.payload)
            elif self.corr_type == "ant_mqtt":
                data = self.serial_port.write(msg.payload)
            self.last_receive_time = time.time()
        except Exception as e:
            self.get_logger().error(f"SN4500: Write the corrections failed. ({e})")
            dev_port = "/dev/ttyTHS1"
            baud = 460800
            self.serial_port = serial.Serial(port=dev_port, baudrate=baud)  #38400


    def check_RTCM_timeout(self, event):
        if time.time() - self.last_receive_time > 30: # 30s
            self.get_logger().error("SN4500: The RTCM message update timeout ({} > 30s).".format(time.time() - self.last_receive_time))
            #self.last_receive_time = time.time()
        #print(time.time() - self.last_receive_time) # test
###end

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
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ntrip_server, self.ntrip_port))
            self.sock.send(self.make_ntrip_request())
            print("[INFO] Connected to NTRIP server") 
        except:
            print("[ERROR] can't connect to NTRIP server")
    def stream_corrections(self,event=None):
        try:
            print("[INFO] Streaming corrections...")
            data = self.sock.recv(1024)
            if not data:
                print("[WARN] NTRIP server closed connection")
                self.connect_ntrip()
            print(data)
            self.serial_port.write(data)
            print(self.serial_port)
        except:
            print("[ERROR] NTRIP server break, reconnecting")
            self.connect_ntrip()
    def gga_callback(self,data):
        self.latest_gga=data.data
        return

    def time_offset_callback(self,data):
        self.time_offset = data.t_offset
        return

    def send_gga(self,event=None):
        
        print("in send gga")
        """ Periodically send latest GGA to NTRIP server """ 
        if self.latest_gga and self.time_offset<0.5:
            try:
                self.sock.send(self.latest_gga.encode())
                print(f"[INFO] Sent GGA: {self.latest_gga.strip()}")
                self.sent_time = self.get_clock().now()
            except Exception as e:
                print(f"[WARN] Failed to send GGA: {e}")
                #self.running=False
                self.connect_ntrip()
        if ((self.get_clock().now()-self.sent_time).nanoseconds/1e9 )> 35:
            self.connect_ntrip()


    def close(self):
        self.running=False
        if self.serial_port:
            self.serial_port.close()
        if self.sock:
            self.sock.close()
        print("[INFO] Closed connections")



def main(args=None):
    rclpy.init(args=args)
    gps_corr = gpsCorrections()
    
    #gps_corr.client.loop_start()
    
    try:
        if gps_corr.corr_type == "ntrip":
            gps_corr.connect_ntrip()
            gps_corr.create_timer(5, gps_corr.send_gga)
            gps_corr.create_timer(0.1, gps_corr.stream_corrections) 
            rclpy.spin(gps_corr) 
        if gps_corr.corr_type == "ppp" or "ant_mqtt":
            gps_corr.client.loop_start()
            gps_corr.create_timer(1,lambda:None)
            rclpy.spin(gps_corr)
            #rate = rospy.Rate(1) # 1hz
            #while not rospy.is_shutdown():
                #print("gpsCorrections is running") # for test
            #    rate.sleep()
            #gps_corr.client.loop_stop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if gps_corr.corr_type == "ntrip":
            gps_corr.close()
        rclpy.shutdown()





if __name__ == '__main__':
    main()
    
