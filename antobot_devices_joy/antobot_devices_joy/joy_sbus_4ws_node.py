#!/usr/bin/env python3
import asyncio
import time
import rclpy
from collections import deque
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from std_msgs.msg import Int32 

from .sbus_received import SBUSReceiver

class JoystickSbus(Node):
    def __init__(self):
        super().__init__('joy_sbus_4ws_node')

        self.declare_parameter('dev', '/dev/anto_joy')
        self.device_port = self.get_parameter('dev').get_parameter_value().string_value
        print(f"device_port:{self.device_port}" )
        self.indoor_demo_pub = self.create_publisher(Empty, '/indoor_demo', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = self.device_port

        # 4WS mode publisher
        self.mode_pub = self.create_publisher(Int32, '/antobot/control/wheelsteer/mode', 10)
        self.current_mode = 3

        self.device_connect = False
        self.publish_first = True
        self.debug_ = True

        self.axes = [0.0] * 8
        self.buttons = [0] * 11
        self.axes_pre = [0.0] * 8
        self.buttons_pre = [0] * 11

        self.channel5_pre = None
        self.channel6_pre = None
        self.buttons_reset = True 

        # Buttons
        self.A = self.B = self.X = self.Y = 0
        self.LB = self.RB = self.BACK =  0
        self.RT = 0.0

        self.buffer_size = 30
        self.data_buffer = deque(maxlen=self.buffer_size)
        self.flag = 0         
        self.stable_value = None

        self.blockOut = 0      

        self.get_logger().info(f"Joystick SBUS 7C node started on port {self.device_port}")

    def translate_buttons(self, num):
        """三挡开关数值 0 / 1 / 2"""
        if num < 300:
            return 0
        elif 900 < num < 1100:
            return 1
        elif num > 1700:
            return 2
        return 1

    def normalize_axis(self, val):
        """ 200~1800 映射 [-1.0, 1.0]"""
        if val < 100 or val > 1900:
            return 0.0
        norm = (val - 1000) / 800.0
        norm = max(min(norm, 1.0), -1.0)
        if abs(norm) < 0.05:
            norm = 0.0
        return round(norm, 3)

    def set_buttons_zero(self):
        self.A = self.B = self.X = self.Y = 0
        self.LB = self.RB = self.BACK = 0
        self.RT = 0.0

    def filterStable(self, val):
        self.data_buffer.append(val)
        if len(self.data_buffer) < self.buffer_size:
            self.flag = 0
            return
        # 使用集合检查所有元素是否相同
        unique_values = set(self.data_buffer)

        if len(unique_values) == 1:
            self.flag = 1
            self.stable_value = self.data_buffer[0]
        else:
            self.flag = 0
            self.stable_value = None

    def create_joy_msg(self, SbusFrame):
        ch = SbusFrame.sbusChannels

        right_rocker_LR = ch[0] #  右摇杆左右
        print(f"channel1 {ch[0]}")
        right_rocker_FB = ch[2] #  右摇杆上下 只有右遥感前后能设置教练锁死
        print(f"channel3 {ch[2]}")
        left_rocker_FB = ch[1] #  左摇杆前后
        print(f"channel2 {ch[1]}")
        left_rocker_LR = ch[3] #  左摇杆左右
        print(f"channel4 {ch[3]}")
        channel5 = ch[4] # CH5/CH6: 三挡开关 (SWA)
        channel6 = ch[5] # CH5/CH6: 三挡开关 (SWB)
        knob1 = ch[6] # CH7/CH8: 两个旋钮（200~1800）
        knob2 = ch[7]

        # Update stability using left_rocker_FB
        self.filterStable(ch[1])

        left_LR = self.normalize_axis(left_rocker_LR)
        left_FB = self.normalize_axis(left_rocker_FB)
        right_LR = self.normalize_axis(right_rocker_LR)
        right_FB = self.normalize_axis(right_rocker_FB)
        knob1_norm = self.normalize_axis(knob1)
        knob2_norm = self.normalize_axis(knob2)


        if right_rocker_FB < 50:
            self.axes = [0.0] * 8
            self.buttons = [0] * 11
            self.blockOut = 1
            return
        else:
            self.blockOut = 0

        ch5_val = self.translate_buttons(channel5)
        ch6_val = self.translate_buttons(channel6)
        print("ch5_val: ", ch5_val)
        print("ch6_val: ", ch6_val)
        print("right_FB: ", right_FB)

        # ========= 4WS steering mode selection =========
        # NOTE:
        #   - flag == 0 → mode CANNOT change (ignore ch5/ch6)
        #   - flag == 1 → allow mode change according to switches
        #
        # Rule when flag == 1:
        #   - if ch6_val != 1 → LOCK (3)
        #   - if ch6_val == 1:
        #         ch5_val == 0 → CRAB (0)
        #         ch5_val == 1 → COUNTERPHASE (1)
        #         ch5_val == 2 → SPOTTURN (2)
        if self.flag == 1 and self.blockOut == 0:
            if ch6_val == 1:
                if ch5_val == 0:
                    mode = 0  # CRAB
                elif ch5_val == 1:
                    mode = 1  # COUNTERPHASE
                elif ch5_val == 2:
                    mode = 2  # SPOTTURN
                else:
                    mode = 3  # fallback to LOCK
            else:
                mode = 3      # SWB not middle → LOCK

            # Only publish when mode changes
            if mode != self.current_mode:
                self.current_mode = mode
                mode_msg = Int32()
                mode_msg.data = mode
                self.mode_pub.publish(mode_msg)
                mode_name = {
                    0: "CRAB",
                    1: "COUNTERPHASE",
                    2: "SPOTTURN",
                    3: "LOCK"
                }.get(mode, "UNKNOWN")
                self.get_logger().info(
                    f"[4WS] Steering mode changed to {mode} ({mode_name}), flag={self.flag}"
                )
        else:
            # flag == 0 or blockOut == 1: do not modify current_mode
            # self.get_logger().debug(f"[4WS] Mode change blocked (flag={self.flag}, blockOut={self.blockOut})")
            pass
        # ============================

        # ========= Deactivate buttons (LB / RB) based on stability =========
        # If the signal is stable and not blocked, treat LB/RB as pressed.
        if self.flag == 1 and self.blockOut == 0:
            self.LB = 1
            self.RB = 1
        else:
            self.LB = 0
            self.RB = 0
        # ============================

        # 映射到 Joy 消息,axes共8通道, 无旋钮消息
        self.axes = [
            0.0, left_FB, 0.0, right_LR, right_FB, self.RT, 0.0, left_LR
        ]
        self.buttons = [self.A, self.B, self.X, self.Y,
                        self.LB, self.RB, self.BACK, 0, 0, 0, self.flag]

        print(f'axes: {self.axes}')
        print(f'buttons: {self.buttons}')

        # 数据变化时发布
        if (self.axes != self.axes_pre or self.buttons != self.buttons_pre
                or abs(self.axes[1]) > 0.9 or abs(self.axes[3]) > 0.9):
            print(f'axes: {self.axes}')
            print(f'buttons: {self.buttons}')
            self.joy_msg.axes = self.axes
            self.joy_msg.buttons = self.buttons
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy_msg)

        self.axes_pre = list(self.axes)
        self.buttons_pre = list(self.buttons)
        self.set_buttons_zero()

    async def read_by_sbus(self):
        while not self.device_connect:
            try:
                sbus = await SBUSReceiver.create(self.device_port)
                self.device_connect = True
            except Exception as e:
                if self.publish_first:
                    self.get_logger().warn(f'Cannot open {self.device_port}: {e}')
                    self.publish_first = False
                await asyncio.sleep(1)

        while rclpy.ok():
            frame = await sbus.get_frame()
            self.create_joy_msg(frame)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickSbus()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.read_by_sbus())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == '__main__':
    main()
