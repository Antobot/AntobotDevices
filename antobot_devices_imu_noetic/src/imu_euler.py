#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: 	The primary purpose of this code is to create a new frame called laser_imu using imu's roll and pitch
#               data. By creating this new laser_imu frame that will be the origin frame of the 3D lidar, we can compnesate
#               the roll and pitch angle of the robot. This will avoid including the floor to the costmap when the robot 
#               is looking downwards. Currently this is only useful for bumps or irregular floor condition in the flat farm.  
# Subscribes to: imu topic (/imu/data_corrected) - after calibration 
# Publishes : laser_imu tf frame - which is then used as the frame for 3D lidar (when roll pitch angle compensation is enabled)
# Contacts:     soyoung.kim@antobot.ai
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import rclpy
import time
import math
import sys, signal
import tf

from sensor_msgs.msg import Imu

def signal_handler(signal, frame):
    print("\nimu TF node killed")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class imuTf:
    def __init__(self):
        # imu
        self.q_imu = []
        self.angles = None

        # Subscribers
        self.sub_imu = rclpy.Subscriber('imu/data', Imu, self.imuCallback) # Check frequency is 50Hz 
        #self.sub_imu = rclpy.Subscriber('/imu/data_corrected', Imu, self.imuCallback) # Check frequency is 10hz

###################################################################################################       
### Callback functions 

    def imuCallback(self,data):  
        # Description: IMU callback function (IMU sensor raw data)
        self.q_imu = data.orientation
        self.angles = tf.transformations.euler_from_quaternion([self.q_imu.x, self.q_imu.y, self.q_imu.z, self.q_imu.w])
        #print('angle roll {} pitch {} yaw {}'.format(self.angles[0],self.angles[1], self.angles[2]))

     
###################################################################################################       

### Main loop

if __name__ == '__main__':
    # init node
    rosnode = rclpy.init_node('imu_euler', anonymous=True)
    
    # Define imuTf calibration
    imuTf = imuTf()
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    # Front lidar
    listener.waitForTransform("base_link", "laser_link_front", rclpy.Time.now(), rclpy.Duration(3.0))
    (trans,rot) = listener.lookupTransform ("base_link", "laser_link_front", rclpy.Time(0))
    rpy = tf.transformations.euler_from_quaternion(rot) 

    # Rear lidar
    rear_lidar_available = True # only available in v3 platform
    try:
        listener.waitForTransform("base_link", "laser_link_rear", rclpy.Time.now(), rclpy.Duration(3.0))
        (trans_2,rot_2) = listener.lookupTransform ("base_link", "laser_link_rear", rclpy.Time.now())
        rpy_2 = tf.transformations.euler_from_quaternion(rot_2) 
    except:
        print("Rear lidar frame not available")
        rear_lidar_available = False


    

    # loop rate
    rate = rclpy.Rate(50)  # 10hz for imu publishing
    
    while not rclpy.is_shutdown():
        if (imuTf.angles is not None):
            # Front lidar frame
            q1 = tf.transformations.quaternion_from_euler(imuTf.angles[0], 0.0, 0.0) # RPY
            q2 = tf.transformations.quaternion_from_euler(0.0,rpy[1],rpy[2])
            q3 = tf.transformations.quaternion_multiply(q1, q2)
            br.sendTransform((trans[0], trans[1], 0),
                       q3,
                        rclpy.Time.now(),
                        "laser_tmp_front",
                        "base_link")
            br.sendTransform((0, 0, trans[2]),
                       tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                        rclpy.Time.now(),
                        "laser_link_front",
                        "laser_tmp_front")
            
            if (rear_lidar_available):
                # Rear lidar frame
                q2 = tf.transformations.quaternion_from_euler(0.0,rpy_2[1],rpy_2[2])
                q3 = tf.transformations.quaternion_multiply(q1, q2)
                br.sendTransform((trans_2[0], trans_2[1], 0),
                        q3,
                            rclpy.Time.now(),
                            "laser_tmp_rear",
                            "base_link")
                br.sendTransform((0, 0, trans_2[2]),
                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                            rclpy.Time.now(),
                            "laser_link_rear",
                            "laser_tmp_rear")
        rate.sleep()
