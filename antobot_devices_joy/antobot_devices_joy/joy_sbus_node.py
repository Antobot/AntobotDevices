#!/usr/bin/env python3
import asyncio
import time
import rclpy
import psutil
from collections import deque
from rclpy.node import Node
from datetime import datetime
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

from .sbus_received import SBUSReceiver
class JoystickSbus(Node):
    def __init__(self):
        super().__init__('joy_sbus_node')

        self.declare_parameter('dev', '/dev/ttyUSB0')
        self.device_port = self.get_parameter('dev').get_parameter_value().string_value

        self.indoor_demo_pub = self.create_publisher(Empty, '/indoor_demo', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy_sbus', 10)
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = self.device_port

        self.device_connect = False
        self.publish_first = True
        self.debug_ = False

        self.axes = [0.0] * 8
        self.buttons = [0] * 11
        self.axes_pre = [0.0] * 8
        self.buttons_pre = [0] * 11

        self.channel5_pre = None
        self.channel6_pre = None
        self.buttons_reset = True

        # 按钮
        self.A = self.B = self.X = self.Y = 0
        self.LB = self.RB = self.BACK =  0
        self.RT = 0.0

        self.buffer_size = 30
        self.data_buffer = deque(maxlen=self.buffer_size)
        self.flag = 0
        self.flag_pre = 0
        self.stable_value = None

        self.blockOut = 0

        self.knob1_norm_orin = 0.0
        self.knob2_norm_orin = 0.0
        self.min_knob1 = float('inf')
        self.openUV = None
        self.openUV_pre = None
        self.releaseStop = None
        self.kaiqikongzhi = None

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

    def uvState(self):
        if self.knob1_norm_orin - self.min_knob1 > 1 and self.knob1_norm_orin > 0.5:
            self.openUV = 1
        elif self.knob1_norm_orin < -0.3:
            self.openUV = 2

    def velC(self):
        if self.flag == 1 and self.flag_pre == 0:
            self.kaiqikongzhi = 1
            self.flag_pre = 1
        elif self.flag == 1 and self.flag_pre == 1:
            self.kaiqikongzhi = 2
            self.flag_pre = 1
        elif self.flag == 0 and self.flag_pre == 1:
            self.kaiqikongzhi = 3
            self.flag_pre = 0

    def create_joy_msg(self, SbusFrame):
        ch = SbusFrame.sbusChannels

        right_rocker_LR = ch[0] #  右摇杆左右
        # print(f"channel1 {ch[0]}")
        right_rocker_FB = ch[2] #  右摇杆上下 只有右遥感前后能设置教练锁死
        # print(f"channel3 {ch[2]}")
        left_rocker_FB = ch[1] #  左摇杆前后
        # print(f"channel2 {ch[1]}")
        left_rocker_LR = ch[3] #  左摇杆左右
        # print(f"channel4 {ch[3]}")
        channel5 = ch[4] # CH5/CH6: 三挡开关
        channel6 = ch[5]
        knob1 = ch[6] # CH7/CH8: 两个旋钮（200~1800）
        knob2 = ch[7]

        self.filterStable(ch[1])
        self.velC()


        left_LR = self.normalize_axis(left_rocker_LR)
        left_FB = self.normalize_axis(left_rocker_FB)
        right_LR = self.normalize_axis(right_rocker_LR)
        right_FB = self.normalize_axis(right_rocker_FB)
        self.knob1_norm_orin = self.normalize_axis(knob1)
        self.knob2_norm_orin = self.normalize_axis(knob2)
        self.min_knob1 = min(self.min_knob1, self.knob1_norm_orin)

        if self.knob1_norm_orin >= 0:
            knob1_norm = 1
        elif self.knob1_norm_orin < 0:
            knob1_norm = -1

        if self.knob2_norm_orin >= 0:
            knob2_norm = 1
        elif self.knob2_norm_orin < 0:
            knob2_norm = -1

        if right_rocker_FB < 50:
            # rocker set ->0
            self.axes = [0.0] * 8
            self.buttons = [0] * 11
            return


        if(right_rocker_FB < 50):
            self.blockOut = 1

        self.uvState()


        ch5_val = self.translate_buttons(channel5)
        ch6_val = self.translate_buttons(channel6)
        # print("ch5_val: ", ch5_val)
        # print("ch6_val: ", ch6_val)
        # print("right_FB: ", right_FB)
        # print("self.flag: ", self.flag)
        # print("self.flag_pre: ", self.flag_pre)

        # task
        if self.buttons_reset:
            # Manual / Standalone for UV treatment / Standalone for Scouting
            if self.kaiqikongzhi == 1:
                self.LB = 1
                self.RB = 1
                self.flag_pre = 1
                if self.debug_:
                    print('Manual')
                self.buttons_reset = True
            elif self.kaiqikongzhi == 2 :
                self.LB = 0
                self.RB = 0

            # UV Switch
            elif self.openUV == 1 :
                self.X = 2
                self.BACK = 2
                self.buttons_reset = True
                # self.openUV_pre = 1
                if self.debug_:
                    print('UV Switch')
            elif self.openUV == 2 :
                self.X = 3
                self.BACK = 3
                self.buttons_reset = True
                # self.openUV_pre = 2
                if self.debug_:
                    print('UV Switch')

            else :
                self.LB = 0
                self.RB = 0
                self.flag_pre = 0




            # Indoor demo - temporary code
        #     elif ch5_val == 2 and ch6_val == 2 and right_FB == 0.0:
        #         self.channel5_pre = ch5_val
        #         self.channel6_pre = ch6_val
        #
        #         self.indoor_demo_pub.publish(Empty())
        #         self.buttons_reset = True
        #         if self.debug_:
        #             print('Start indoor demo - Place the robot in the row entry before trigger')
        #
        #     # Pause Job
        #     elif ch5_val == 0 and ch6_val == 1 and right_FB == 0.0:
        #         self.channel5_pre = ch5_val
        #         self.channel6_pre = ch6_val
        #
        #         self.Y = 1
        #         self.buttons_reset = True
        #         if self.debug_:
        #             print('Pause Job')
        #
        #     # Resume Job
        #     elif ch5_val == 2 and ch6_val == 1 and right_FB == 0.0:
        #         self.channel5_pre = ch5_val
        #         self.channel6_pre = ch6_val
        #
        #         self.A = 1
        #         self.buttons_reset = True
        #         if self.debug_:
        #             print('Resume Job')
        #
        #     # Standalone for Scouting
        #     elif ch5_val == 0 and ch6_val == 1 and right_FB == 1.0:
        #         self.channel5_pre = ch5_val
        #         self.channel6_pre = ch6_val
        #
        #         self.A = 1
        #         self.RT = -1.0
        #         self.buttons_reset = True
        #         if self.debug_:
        #             print('Standalone for Scouting')
        #
        #     # Abort Job
        #     elif ch5_val == 0 and ch6_val == 1 and right_FB == -1.0:
        #         self.channel5_pre = ch5_val
        #         self.channel6_pre = ch6_val
        #
        #         self.B = 1
        #         self.RT = -1.0
        #         self.buttons_reset = True
        #         if self.debug_:
        #             print('Abort Job')
        #
        #     # UV Switch
        #     elif knob1_norm > 0.0 and knob2_norm > 0.0:
        #
        #         self.X = 1
        #         self.BACK = 1
        #         self.buttons_reset = True
        #         if self.debug_:
        #             print('UV Switch')
        #
        # elif ch5_val == 1 and ch6_val == 1:
        #     if self.channel5_pre is None or self.channel6_pre is None:
        #         self.channel5_pre = ch5_val
        #         self.channel6_pre = ch6_val
        #         self.buttons_reset = True
        #     elif ch5_val != self.channel5_pre or ch6_val != self.channel6_pre:
        #         self.buttons_reset = True

        # 映射到 Joy 消息,axes共8通道, 有旋钮消息
        # [LX, LY, RX, RY, RT, knob1, knob2, 保留]
        # self.axes = [
        #     left_LR, left_FB, right_LR, right_FB,
        #     self.RT, knob1_norm, knob2_norm, 0.0
        # ]
        # 映射到 Joy 消息,axes共8通道, 无旋钮消息
        # print(f'knob1_norm: {knob1_norm}')
        # print(f'knob2_norm: {knob2_norm}')

        self.axes = [
            0.0, left_FB, 0.0, -right_LR, right_FB, self.RT, 0.0, left_LR
            # ,self.knob1_norm_orin, self.knob2_norm_orin
        ]
        self.buttons = [self.A, self.B, self.X, self.Y,
                        self.LB, self.RB, self.BACK, 0, 0, 0, self.flag]

        # print(f'axes: {self.axes}')
        # print(f'buttons: {self.buttons}')

        # 数据变化时发布
        if (self.axes != self.axes_pre or self.buttons != self.buttons_pre
                or abs(self.axes[4]) > 0.9 or abs(self.axes[3]) > 0.9):
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
    while True:
        cpu_usage = psutil.cpu_percent(interval=0.5)

        if cpu_usage < 90:
            print(f"[INFO] CPU OK ({cpu_usage}%). Starting node.")
            break
        now = datetime.now().strftime("%H:%M:%S")
        print(f"[WARN] {now} CPU too high ({cpu_usage}%). Delaying node startup...")
        time.sleep(1)
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
