#!/usr/bin/env python3

# Copyright (c) 2025, ANTOBOT LTD.
# All rights reserved.
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     The purpose of this code is to read the sbus data and then convert it into a Joy message.

# Contact: Huaide Wang 
# email: huaide.wang@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import time
import rclpy
from rclpy.node import Node

# Install the ELRS_Serial package located in acExternal with the command "pip install -e ."
import ELRS_Serial as rx

from sensor_msgs.msg import Joy


class Joystick_ELRS(Node):
    def __init__(self, frequency=30):

        super().__init__('joystick_ELRS')
        self.declare_parameter('device_port', '/dev/ttyUSB0')

        self.device_port = self.get_parameter('device_port').get_parameter_value().string_value

        self.timer_period = 1/frequency 

        self.device_connect = False
        self.publish_first = True
        
        
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # [A, B, X, Y, LB, RB, BACK, START, LOGITECH, LS, RS]
        self.axes_pre = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.buttons_pre = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.joy_pub_active = True
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = self.device_port

        self.channel5_pre = None
        self.channel6_pre = None
        
        self.buttons_reset = False
        
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        
        self.LB = 0
        self.RB = 0
        self.BACK = 0

        self.RT = 0 
        
        self.debug_ = False
        
        self.SE_cnt = 0

        self.timer = self.create_timer(self.timer_period, self.update)

    def translate_buttons(self, num):
        if num < 300:
            return 0.0
        elif num > 900 and num < 1100:
            return 1
        elif num > 1700:
            return 2.0
        
    def translate_axes(self, num, coef = 1): 

        num = round((num - 1500) / 500, 2)
        if abs(num) < 0.05:
            num = 0.0
        
        return num * coef

    def set_buttos_zero(self):
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
                
        self.LB = 0
        self.RB = 0
        self.BACK = 0 
        self.RT = 0
        
    def create_joy_msg(self, frame):
    
        left_rocker_LR = frame[4]
        left_rocker_FB = frame[3]
        right_rocker_LR = frame[1] 
        right_rocker_FB = frame[2]
        
        SE = frame[9]
        # SE = 1 if frame[9] > 1500 else 0

        axes = [self.translate_axes(left_rocker_LR), self.translate_axes(left_rocker_FB), self.translate_axes(right_rocker_LR, -1), self.translate_axes(right_rocker_FB)]

        if SE > 1500:
            self.SE_cnt += 1
        else:
            self.SE_cnt = 0

        if self.SE_cnt > 10: # trigger, 10 times for security
            self.LB = 1
            self.RB = 1
            self.SE_cnt = 0
            # print('Manual')


        self.axes[1] = axes[1]
        self.axes[3] = axes[2]
        self.axes[4] = axes[3]
        self.axes[7] = axes[0]

        # self.axes = [0.0, axes[1], 0.0, axes[2], axes[3], self.RT, 0.0, axes[0]]
        # self.buttons = [self.A, self.B, self.X, self.Y, self.LB, self.RB, self.BACK, 0, 0, 0, 0]
        self.buttons[4] = self.LB
        self.buttons[5] = self.RB
        # print(self.axes)
        # print(self.axes != self.axes_pre)
        # if self.axes != self.axes_pre or self.buttons != self.buttons_pre or abs(self.axes[1]) > 0.9 or abs(self.axes[3]) > 0.9 or (self.LB == 1 and self.RB == 1):

        self.joy_msg.axes = self.axes
        self.joy_msg.buttons = self.buttons   
        self.joy_msg.header.stamp = self.get_clock().now().to_msg() 
        self.joy_pub.publish(self.joy_msg)

        
        self.axes_pre = self.axes
        self.buttons_pre = self.buttons
        self.set_buttos_zero()

    def update(self):
        
        if not self.device_connect:
            try:
                self.receiver = rx.CRSFReceiver(port=self.device_port, baudrate=420000)
                self.device_connect = True
            except:
                if self.publish_first:
                    self.get_logger().warn(f'JoyELRS could not open {self.device_port}')
                    self.publish_first = False
                time.sleep(1)
        else:
            try:
                self.receiver.update()
                frame = self.receiver.get_channels()
                # print(frame)
                # print(type(frame))
                self.create_joy_msg(frame)
            except:
                self.device_connect = False


def main(args=None):
    rclpy.init(args=args)

    JE = Joystick_ELRS()

    rclpy.spin(JE)

    JE.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    
    main()
