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

import rospy
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
        self.sub_imu = rospy.Subscriber('imu/data', Imu, self.imuCallback) # Check frequency is 50Hz 
        #self.sub_imu = rospy.Subscriber('/imu/data_corrected', Imu, self.imuCallback) # Check frequency is 10hz

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # Front lidar
        self.listener.waitForTransform("base_link", "laser_link_front", rospy.Time.now(), rospy.Duration(3.0))
        (self.trans,self.rot) = self.listener.lookupTransform ("base_link", "laser_link_front", rospy.Time(0))
        self.rpy = tf.transformations.euler_from_quaternion(self.rot) 

        # Rear lidar
        rear_lidar_available = True # only available in v3 platform
        try:
            self.listener.waitForTransform("base_link", "laser_link_rear", rospy.Time.now(), rospy.Duration(3.0))
            (self.trans_2,self.rot_2) = self.listener.lookupTransform ("base_link", "laser_link_rear", rospy.Time.now())
            self.rpy_2 = tf.transformations.euler_from_quaternion(self.rot_2) 
        except:
            print("Rear lidar frame not available")
            self.rear_lidar_available = False


###################################################################################################       
### Callback functions 

    def imuCallback(self,data):  
        # Description: IMU callback function (IMU sensor raw data)
        self.q_imu = data.orientation
        self.angles = tf.transformations.euler_from_quaternion([self.q_imu.x, self.q_imu.y, self.q_imu.z, self.q_imu.w])
        #print('angle roll {} pitch {} yaw {}'.format(self.angles[0],self.angles[1], self.angles[2]))

###################################################################################################

    def main(self, event=None):    
        if (self.angles is not None):
            # Front lidar frame
            q1 = tf.transformations.quaternion_from_euler(self.angles[0], 0.0, 0.0) # RPY
            q2 = tf.transformations.quaternion_from_euler(0.0,self.rpy[1],self.rpy[2])
            q3 = tf.transformations.quaternion_multiply(q1, q2)
            self.br.sendTransform((self.trans[0], self.trans[1], 0),
                       q3,
                        rospy.Time.now(),
                        "laser_tmp_front",
                        "base_link")
            self.br.sendTransform((0, 0, self.trans[2]),
                       tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                        rospy.Time.now(),
                        "laser_link_front",
                        "laser_tmp_front")
            
            if (self.rear_lidar_available):
                # Rear lidar frame
                q2 = tf.transformations.quaternion_from_euler(0.0,self.rpy_2[1],self.rpy_2[2])
                q3 = tf.transformations.quaternion_multiply(q1, q2)
                self.br.sendTransform((self.trans_2[0], self.trans_2[1], 0),
                        q3,
                            rospy.Time.now(),
                            "laser_tmp_rear",
                            "base_link")
                self.br.sendTransform((0, 0, self.trans_2[2]),
                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                            rospy.Time.now(),
                            "laser_link_rear",
                            "laser_tmp_rear")


     
###################################################################################################       

### Main loop

if __name__ == '__main__':
    # init node
    rosnode = rospy.init_node('imu_euler', anonymous=True)
    
    # Define imuTf calibration
    imutf = imuTf()

    rospy.Timer(rospy.Duration(0.02), imutf.main)  # Runs periodically without blocking
    rospy.spin() 







