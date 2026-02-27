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
from enum import Enum

from .sbus_received import SBUSReceiver


class RobotState(Enum):
    """机器人状态枚举"""
    SHUTDOWN = 0      # 关机状态
    POWER_ON = 1      # 开机状态
    ACTIVATED = 2     # 激活状态


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
        self.debug_ = True

        # Joy消息数据
        self.axes = [0.0] * 8
        self.buttons = [0] * 11
        self.axes_pre = [0.0] * 8
        self.buttons_pre = [0] * 11

        # 状态机相关
        self.current_state = RobotState.SHUTDOWN
        self.activation_triggered = False  # 记录激活状态是否已触发LB/RB

        # 多帧窗口检测
        self.window_size = 30
        self.ch0_buffer = deque(maxlen=self.window_size)
        self.ch1_buffer = deque(maxlen=self.window_size)
        self.ch2_buffer = deque(maxlen=self.window_size)
        self.ch3_buffer = deque(maxlen=self.window_size)

        # 触发状态记录
        self.ch4_prev = 1000  # ch[4]前一帧的值
        self.ch5_prev = 1000  # ch[5]前一帧的值
        self.ch6_min_in_activation = float('inf')  # 本次激活时ch[6]的最小值
        self.ch6_trigger_2_done = False  # ch[6]触发2是否已完成
        self.ch6_trigger_3_done = False  # ch[6]触发3是否已完成

        # 按钮状态
        self.A = self.B = self.X = self.Y = 0
        self.LB = self.RB = self.BACK = 0
        self.RT = 0.0

        self.get_logger().info(f"Joystick SBUS State Machine node started on port {self.device_port}")

    def normalize_axis(self, val):
        """200~1800 映射到 [-1.0, 1.0]"""
        if val < 100 or val > 1900:
            return 0.0
        norm = (val - 1000) / 800.0
        norm = max(min(norm, 1.0), -1.0)
        if abs(norm) < 0.05:
            norm = 0.0
        return round(norm, 3)

    def is_buffer_stable(self, buffer):
        """检查缓冲区内的值是否稳定（所有值相同）"""
        if len(buffer) < self.window_size:
            return False, None
        unique_values = set(buffer)
        if len(unique_values) == 1:
            return True, buffer[0]
        return False, None

    def is_buffer_dynamic(self, buffer):
        """检查缓冲区内的值是否动态（值不全相同）"""
        if len(buffer) < self.window_size:
            return False
        unique_values = set(buffer)
        return len(unique_values) > 1

    def update_state(self, ch):
        """更新状态机状态"""
        # 更新多帧窗口
        self.ch0_buffer.append(ch[0])
        self.ch1_buffer.append(ch[1])
        self.ch2_buffer.append(ch[2])
        self.ch3_buffer.append(ch[3])

        # 检查ch[0]-ch[3]是否都稳定
        ch0_stable, _ = self.is_buffer_stable(self.ch0_buffer)
        ch1_stable, ch1_val = self.is_buffer_stable(self.ch1_buffer)
        ch2_stable, _ = self.is_buffer_stable(self.ch2_buffer)
        ch3_stable, _ = self.is_buffer_stable(self.ch3_buffer)

        all_stable = ch0_stable and ch1_stable and ch2_stable and ch3_stable

        # 检查ch[0]-ch[3]是否都动态
        all_dynamic = (self.is_buffer_dynamic(self.ch0_buffer) and
                       self.is_buffer_dynamic(self.ch1_buffer) and
                       self.is_buffer_dynamic(self.ch2_buffer) and
                       self.is_buffer_dynamic(self.ch3_buffer))

        prev_state = self.current_state

        # 状态转换逻辑
        if all_stable:
            # ch[0]-ch[3]都稳定 -> 关机状态
            if self.current_state != RobotState.SHUTDOWN:
                self.get_logger().info("State transition: -> SHUTDOWN")
                self.current_state = RobotState.SHUTDOWN
                self.activation_triggered = False
                self.ch6_min_in_activation = float('inf')
                self.ch6_trigger_2_done = False
                self.ch6_trigger_3_done = False
                self.X = 3
                self.BACK = 3

                # 立即发布这个退出信号
                self.axes = [0.0] * 8
                self.buttons = [0, 0, self.X, 0, 0, 0, self.BACK, 0, 0, 0, 0]
                self.joy_msg.axes = self.axes
                self.joy_msg.buttons = self.buttons
                self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                self.joy_pub.publish(self.joy_msg)

                # 更新前值
                self.axes_pre = list(self.axes)
                self.buttons_pre = list(self.buttons)

                # 状态转换
                self.current_state = RobotState.POWER_ON
                self.activation_triggered = False
                self.ch6_min_in_activation = float('inf')
                self.ch6_trigger_2_done = False
                self.ch6_trigger_3_done = False

                # 重置按钮
                self.reset_buttons()

        elif all_dynamic:
            # ch[0]-ch[3]都动态 -> 开机状态
            if self.current_state == RobotState.SHUTDOWN:
                self.get_logger().info("State transition: SHUTDOWN -> POWER_ON")
                self.current_state = RobotState.POWER_ON

        # 开机状态下，检查是否进入激活状态
        if self.current_state == RobotState.POWER_ON:
            if ch1_stable and ch1_val is not None and ch1_val > 300 and ch1_val < 1500:
                self.get_logger().info("State transition: POWER_ON -> ACTIVATED")
                self.current_state = RobotState.ACTIVATED
                self.activation_triggered = False  # 重置触发标志
                self.ch6_min_in_activation = float('inf')  # 重置ch6最小值
                self.ch6_trigger_2_done = False
                self.ch6_trigger_3_done = False

        # 激活状态下，检查是否退出激活状态（取消激活）
        if self.current_state == RobotState.ACTIVATED:

            # 如果ch[1]不再稳定或稳定值不再大于100，退出激活状态
            if not ch1_stable or (ch1_stable and ch1_val is not None and ch1_val <= 100):
                self.get_logger().info("State transition: ACTIVATED -> POWER_ON (deactivation)")

                # 退出激活时触发 X=3, BACK=3
                self.X = 3
                self.BACK = 3

                # 立即发布这个退出信号
                self.axes = [0.0] * 8
                self.buttons = [0, 0, self.X, 0, 0, 0, self.BACK, 0, 0, 0, 0]
                self.joy_msg.axes = self.axes
                self.joy_msg.buttons = self.buttons
                self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                self.joy_pub.publish(self.joy_msg)

                # 更新前值
                self.axes_pre = list(self.axes)
                self.buttons_pre = list(self.buttons)

                # 状态转换
                self.current_state = RobotState.POWER_ON
                self.activation_triggered = False
                self.ch6_min_in_activation = float('inf')
                self.ch6_trigger_2_done = False
                self.ch6_trigger_3_done = False

                # 重置按钮
                self.reset_buttons()

    def reset_buttons(self):
        """重置按钮状态"""
        self.A = self.B = self.X = self.Y = 0
        self.LB = self.RB = self.BACK = 0
        self.RT = 0.0

    def create_joy_msg(self, SbusFrame):
        """使用状态机处理Joy消息创建"""
        ch = SbusFrame.sbusChannels

        # 步骤1: 如果ch[2]小于50，直接返回
        if ch[2] < 50:
            self.axes = [0.0] * 8
            self.buttons = [0] * 11
            return

        # 步骤2-3: 更新状态机
        self.update_state(ch)

        # 如果不在激活状态，或者激活状态未触发过LB/RB，发布零值
        if self.current_state != RobotState.ACTIVATED:
            self.axes = [0.0] * 8
            self.buttons = [0] * 11
            self.reset_buttons()

            # 发布零值消息
            if self.axes != self.axes_pre or self.buttons != self.buttons_pre:
                self.joy_msg.axes = self.axes
                self.joy_msg.buttons = self.buttons
                self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                self.joy_pub.publish(self.joy_msg)
                self.axes_pre = list(self.axes)
                self.buttons_pre = list(self.buttons)
            return

        # 步骤3: 激活状态，首次触发LB/RB
        if not self.activation_triggered:
            self.LB = 1
            self.RB = 1
            self.activation_triggered = True
            self.get_logger().info("Activation triggered: LB=1, RB=1")

            # 首次激活时，只发布LB=1, RB=1，其他都是0
            self.axes = [0.0] * 8
            self.buttons = [0, 0, 0, 0, self.LB, self.RB, 0, 0, 0, 0, 0]

            # 发布消息
            self.joy_msg.axes = self.axes
            self.joy_msg.buttons = self.buttons
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy_msg)

            # 更新前值
            self.axes_pre = list(self.axes)
            self.buttons_pre = list(self.buttons)

            # 重置按钮
            self.reset_buttons()

            # 初始化ch[4]的前值 避免第二帧误触发4.1或4.2
            self.ch4_prev = ch[4]
            self.ch5_prev = ch[5]

            # 如果首次激活时ch[6]小于1000，标记trigger_3已完成，避免误触发
            if ch[6] < 1000:
                self.ch6_trigger_3_done = True

            # 直接返回，不继续处理后续的摇杆映射
            return

        # 步骤4.0: 映射ch[0]-ch[3]为控制方向
        left_rocker_LR = ch[3]
        left_rocker_FB = ch[1]
        right_rocker_LR = ch[0]
        right_rocker_FB = ch[2]

        left_LR = self.normalize_axis(left_rocker_LR)
        left_FB = self.normalize_axis(left_rocker_FB)
        right_LR = self.normalize_axis(right_rocker_LR)
        right_FB = self.normalize_axis(right_rocker_FB)


        # 步骤4.1: ch[4]从1000变为200
        if self.ch4_prev == 1000 and 150 < ch[4] < 250:
            self.A = 1
            self.RT = -1.0
            self.get_logger().info("Trigger 4.1: A=1, RT=-1.0 (ch[4]: 1000->200)")

        # 步骤4.2: ch[4]从1000变为1800
        elif self.ch4_prev == 1000 and 1750 < ch[4] < 1850:
            self.A = -1
            self.RT = -1.0
            self.get_logger().info("Trigger 4.2: A=-1, RT=-1.0 (ch[4]: 1000->1800)")

        # 步骤4.3: ch[5]从1000变为200 --resume
        elif self.ch5_prev == 1000 and 150 < ch[5] < 250:
            self.A = 1
            self.RT = 0.0
            self.get_logger().info("Trigger 4.3: A=1, RT=0.0 (ch[4]: 1000->200)")

        # 步骤4.4: ch[5]从1000变为1800 --pause
        elif self.ch5_prev == 1000 and 1750 < ch[5] < 1850:
            self.Y = 1
            self.RT = 0.0
            self.get_logger().info("Trigger 4.4: Y=1, RT=0.0 (ch[4]: 1000->1800)")

        # 更新ch[4] ch[5]前值
        self.ch4_prev = ch[4]
        self.ch5_prev = ch[5]

        # 更新ch[6]最小值
        self.ch6_min_in_activation = min(self.ch6_min_in_activation, ch[6])

        # 步骤4.3: ch[6]从最小值大800，且自身大于1000
        if (not self.ch6_trigger_2_done and
                ch[6] > 1000 and
                ch[6] - self.ch6_min_in_activation > 800):
            self.X = 2
            self.BACK = 2
            self.ch6_trigger_2_done = True
            self.get_logger().info(f"Trigger 4.3: X=2, BACK=2 (ch[6]={ch[6]}, min={self.ch6_min_in_activation})")

        # 步骤4.4: ch[6]小于1000
        if not self.ch6_trigger_3_done and ch[6] < 1000:
            self.X = 3
            self.BACK = 3
            self.ch6_trigger_3_done = True
            self.ch6_trigger_2_done = False
            self.get_logger().info(f"Trigger 4.4: X=3, BACK=3 (ch[6]={ch[6]})")

        # 如果ch[6]回到大于1000，重置触发3标志
        if ch[6] >= 1000:
            self.ch6_trigger_3_done = False

        # 组装axes和buttons
        self.axes = [
            0.0, left_FB, 0.0, -right_LR, right_FB, self.RT, 0.0, left_LR
        ]
        self.buttons = [
            self.A, self.B, self.X, self.Y,
            self.LB, self.RB, self.BACK, 0, 0, 0, 0
        ]

        # 步骤4.5: 当映射值有更新时发布joy消息
        if (self.axes != self.axes_pre or self.buttons != self.buttons_pre):
            if self.debug_:
                print(f'State: {self.current_state.name}')
                print(f'axes: {self.axes}')
                print(f'buttons: {self.buttons}')

            self.joy_msg.axes = self.axes
            self.joy_msg.buttons = self.buttons
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy_msg)

        # 更新前值
        self.axes_pre = list(self.axes)
        self.buttons_pre = list(self.buttons)

        self.reset_buttons()

    async def read_by_sbus(self):
        while not self.device_connect:
            try:
                sbus = await SBUSReceiver.create(self.device_port)
                self.device_connect = True
                self.get_logger().info(f"Connected to {self.device_port}")
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