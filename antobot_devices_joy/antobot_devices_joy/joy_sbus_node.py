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
    """Robot state enum"""
    SHUTDOWN = 0      # Shutdown state
    POWER_ON = 1      # Power-on state
    ACTIVATED = 2     # Activated state


class JoystickSbus(Node):
    def __init__(self):
        super().__init__('joy_sbus_node')

        self.declare_parameter('dev', '/dev/ttyCH341USB0')
        self.device_port = self.get_parameter('dev').get_parameter_value().string_value

        self.indoor_demo_pub = self.create_publisher(Empty, '/indoor_demo', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy_sbus', 10)
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = self.device_port

        self.device_connect = False
        self.publish_first = True
        self.debug_ = True

        # Joy message data
        self.axes = [0.0] * 8
        self.buttons = [0] * 11
        self.axes_pre = [0.0] * 8
        self.buttons_pre = [0] * 11

        # State machine data
        self.current_state = RobotState.SHUTDOWN
        self.activation_triggered = False  # Whether LB/RB has been triggered in activated state

        # Multi-frame window detection
        self.window_size = 30
        self.ch0_buffer = deque(maxlen=self.window_size)
        self.ch1_buffer = deque(maxlen=self.window_size)
        self.ch2_buffer = deque(maxlen=self.window_size)
        self.ch3_buffer = deque(maxlen=self.window_size)

        # Trigger state tracking
        self.ch4_prev = 1000  # Previous frame value of ch[4]
        self.ch5_prev = 1000  # Previous frame value of ch[5]
        self.ch6_min_in_activation = float('inf')  # Minimum ch[6] value during this activation
        self.ch6_trigger_2_done = False  # Whether trigger 2 for ch[6] has been completed
        self.ch6_trigger_3_done = False  # Whether trigger 3 for ch[6] has been completed
        self.ch7_min_in_activation = float('inf')  # The minimum value of ch[7] during this activation
        self.ch7_trigger_2_done = False  # Whether trigger 2 for ch[7] has been completed
        self.ch7_trigger_3_done = False  # Whether trigger 3 for ch[7] has been completed

        # Button states
        self.A = self.B = self.X = self.Y = 0
        self.LB = self.RB = self.BACK = 0
        self.RT = 0.0

        self.get_logger().info(f"Joystick SBUS State Machine node started on port {self.device_port}")

    def normalize_axis(self, val):
        """Map 200~1800 to [-1.0, 1.0]"""
        if val < 100 or val > 1900:
            return 0.0
        norm = (val - 1000) / 800.0
        norm = max(min(norm, 1.0), -1.0)
        if abs(norm) < 0.05:
            norm = 0.0
        return round(norm, 3)

    def is_buffer_stable(self, buffer):
        """Check whether all values in the buffer are stable."""
        if len(buffer) < self.window_size:
            return False, None
        unique_values = set(buffer)
        if len(unique_values) == 1:
            return True, buffer[0]
        return False, None

    def is_buffer_dynamic(self, buffer):
        """Check whether values in the buffer are dynamic."""
        if len(buffer) < self.window_size:
            return False
        unique_values = set(buffer)
        return len(unique_values) > 1

    def update_state(self, ch):
        """Update the state machine."""
        # Update multi-frame window
        self.ch0_buffer.append(ch[0])
        self.ch1_buffer.append(ch[1])
        self.ch2_buffer.append(ch[2])
        self.ch3_buffer.append(ch[3])

        # Check whether ch[0]-ch[3] are all stable
        ch0_stable, _ = self.is_buffer_stable(self.ch0_buffer)
        ch1_stable, ch1_val = self.is_buffer_stable(self.ch1_buffer)
        ch2_stable, _ = self.is_buffer_stable(self.ch2_buffer)
        ch3_stable, _ = self.is_buffer_stable(self.ch3_buffer)

        all_stable = ch0_stable and ch1_stable and ch2_stable and ch3_stable

        # Check whether ch[0]-ch[3] are all dynamic
        all_dynamic = (self.is_buffer_dynamic(self.ch0_buffer) and
                       self.is_buffer_dynamic(self.ch1_buffer) and
                       self.is_buffer_dynamic(self.ch2_buffer) and
                       self.is_buffer_dynamic(self.ch3_buffer))

        prev_state = self.current_state

        # State transition logic
        if all_stable:
            # ch[0]-ch[3] all stable -> shutdown state
            if self.current_state != RobotState.SHUTDOWN:
                self.get_logger().info("State transition: -> SHUTDOWN")
                self.current_state = RobotState.SHUTDOWN
                self.activation_triggered = False
                self.ch6_min_in_activation = float('inf')
                self.ch6_trigger_2_done = False
                self.ch6_trigger_3_done = False
                self.ch7_min_in_activation = float('inf')
                self.ch7_trigger_2_done = False
                self.ch7_trigger_3_done = False
                self.X = 3
                self.BACK = 3

                # Publish this exit signal immediately
                self.axes = [0.0] * 8
                self.buttons = [0, 0, self.X, 0, 0, 0, self.BACK, 0, 0, 0, 0]
                self.joy_msg.axes = self.axes
                self.joy_msg.buttons = self.buttons
                self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                self.joy_pub.publish(self.joy_msg)

                # Update previous values
                self.axes_pre = list(self.axes)
                self.buttons_pre = list(self.buttons)

                # State transition
                # self.current_state = RobotState.POWER_ON
                # self.activation_triggered = False
                # self.ch6_min_in_activation = float('inf')
                # self.ch6_trigger_2_done = False
                # self.ch6_trigger_3_done = False

                # Reset buttons
                self.reset_buttons()

        elif all_dynamic:
            # ch[0]-ch[3] all dynamic -> power-on state
            if self.current_state == RobotState.SHUTDOWN:
                self.get_logger().info("State transition: SHUTDOWN -> POWER_ON")
                self.current_state = RobotState.POWER_ON

        # In power-on state, check whether to enter activated state
        if self.current_state == RobotState.POWER_ON:
            if ch1_stable and ch1_val is not None and ch1_val > 300 and ch1_val < 1500:
                self.get_logger().info("State transition: POWER_ON -> ACTIVATED")
                self.current_state = RobotState.ACTIVATED
                self.activation_triggered = False  # Reset trigger flag
                self.ch6_min_in_activation = float('inf')  # Reset ch6 minimum value
                self.ch6_trigger_2_done = False
                self.ch6_trigger_3_done = False
                self.ch7_min_in_activation = float('inf')  # Reset ch[7] minimum value
                self.ch7_trigger_2_done = False
                self.ch7_trigger_3_done = False

        # In activated state, check whether to leave activated state
        if self.current_state == RobotState.ACTIVATED:

            # Leave activated state if ch[1] is not stable or its stable value is no longer greater than 100
            if not ch1_stable or (ch1_stable and ch1_val is not None and ch1_val <= 100):
                self.get_logger().info("State transition: ACTIVATED -> POWER_ON (deactivation)")

                # Trigger X=3, BACK=3 when leaving activated state
                self.X = 3
                self.BACK = 3

                # Publish this exit signal immediately
                self.axes = [0.0] * 8
                self.buttons = [0, 0, self.X, 0, 0, 0, self.BACK, 0, 0, 0, 0]
                self.joy_msg.axes = self.axes
                self.joy_msg.buttons = self.buttons
                self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                self.joy_pub.publish(self.joy_msg)

                # Update previous values
                self.axes_pre = list(self.axes)
                self.buttons_pre = list(self.buttons)

                # State transition
                self.current_state = RobotState.POWER_ON
                self.activation_triggered = False
                self.ch6_min_in_activation = float('inf')
                self.ch6_trigger_2_done = False
                self.ch6_trigger_3_done = False
                self.ch7_min_in_activation = float('inf')
                self.ch7_trigger_2_done = False
                self.ch7_trigger_3_done = False

                # Reset buttons
                self.reset_buttons()

    def reset_buttons(self):
        """Reset button states."""
        self.A = self.B = self.X = self.Y = 0
        self.LB = self.RB = self.BACK = 0
        self.RT = 0.0

    def create_joy_msg(self, SbusFrame):
        """Create Joy messages with the state machine."""
        ch = SbusFrame.sbusChannels

        # Step 1: return directly if ch[2] is less than 50
        if ch[2] < 50:
            self.axes = [0.0] * 8
            self.buttons = [0] * 11
            return

        # Steps 2-3: update the state machine
        self.update_state(ch)

        # Publish zero values when not activated or LB/RB has not been triggered
        if self.current_state != RobotState.ACTIVATED:
            self.axes = [0.0] * 8
            self.buttons = [0] * 11
            self.reset_buttons()

            # Publish zero-value message
            if self.axes != self.axes_pre or self.buttons != self.buttons_pre:
                self.joy_msg.axes = self.axes
                self.joy_msg.buttons = self.buttons
                self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                self.joy_pub.publish(self.joy_msg)
                self.axes_pre = list(self.axes)
                self.buttons_pre = list(self.buttons)
            return

        # Step 3: activated state, first LB/RB trigger
        if not self.activation_triggered:
            self.LB = 1
            self.RB = 1
            self.activation_triggered = True
            self.get_logger().info("Activation triggered: LB=1, RB=1")

            # On first activation, publish only LB=1 and RB=1; all others are 0
            self.axes = [0.0] * 8
            self.buttons = [0, 0, 0, 0, self.LB, self.RB, 0, 0, 0, 0, 0]

            # Publish message
            self.joy_msg.axes = self.axes
            self.joy_msg.buttons = self.buttons
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy_msg)

            # Update previous values
            self.axes_pre = list(self.axes)
            self.buttons_pre = list(self.buttons)

            # Reset buttons
            self.reset_buttons()

            # Initialize previous values to avoid false triggers on the second frame
            self.ch4_prev = ch[4]
            self.ch5_prev = ch[5]

            # Mark trigger_3 as done if ch[6] is below 1000 on first activation
            if ch[6] < 1000:
                self.ch6_trigger_3_done = True

            if ch[7] < 1000:
                self.ch7_trigger_3_done = True

            # Return directly and skip subsequent joystick mapping
            return

        # Step 4.0: map ch[0]-ch[3] to control directions
        left_rocker_LR = ch[3]
        left_rocker_FB = ch[1]
        right_rocker_LR = ch[0]
        right_rocker_FB = ch[2]

        left_LR = self.normalize_axis(left_rocker_LR)
        left_FB = self.normalize_axis(left_rocker_FB)
        right_LR = self.normalize_axis(right_rocker_LR)
        right_FB = self.normalize_axis(right_rocker_FB)


        # Step 4.1: ch[4] changes from 1000 to 200
        if self.ch4_prev == 1000 and 150 < ch[4] < 250:
            self.A = 1
            self.RT = -1.0
            self.get_logger().info("Trigger 4.1: A=1, RT=-1.0 (ch[4]: 1000->200)")

        # Step 4.2: ch[4] changes from 1000 to 1800
        elif self.ch4_prev == 1000 and 1750 < ch[4] < 1850:
            self.A = -1
            self.RT = -1.0
            self.get_logger().info("Trigger 4.2: A=-1, RT=-1.0 (ch[4]: 1000->1800)")

        # Step 4.3: ch[5] changes from 1000 to 200 --resume
        elif self.ch5_prev == 1000 and 150 < ch[5] < 250:
            self.A = 1
            self.RT = 0.0
            self.get_logger().info("Trigger 4.3: A=1, RT=0.0 (ch[4]: 1000->200)")

        # Step 4.4: ch[5] changes from 1000 to 1800 --pause
        elif self.ch5_prev == 1000 and 1750 < ch[5] < 1850:
            self.Y = 1
            self.RT = 0.0
            self.get_logger().info("Trigger 4.4: Y=1, RT=0.0 (ch[4]: 1000->1800)")

        # Update previous values for ch[4] and ch[5]
        self.ch4_prev = ch[4]
        self.ch5_prev = ch[5]

        # Update ch[6] minimum value
        self.ch6_min_in_activation = min(self.ch6_min_in_activation, ch[6])



        # Step 4.3: ch[6] is 800 above the minimum and greater than 1000
        if (not self.ch6_trigger_2_done and
                ch[6] > 1000 and
                ch[6] - self.ch6_min_in_activation > 800):
            self.X = 2
            self.BACK = 2
            self.ch6_trigger_2_done = True
            self.get_logger().info(f"Trigger 4.3: X=2, BACK=2 (ch[6]={ch[6]}, min={self.ch6_min_in_activation})")

        # Step 4.4: ch[6] is less than 1000
        if not self.ch6_trigger_3_done and ch[6] < 1000:
            self.X = 3
            self.BACK = 3
            self.ch6_trigger_2_done = False  # Reset trigger 2 when trigger 3 is activated
            self.ch6_trigger_3_done = True
            self.ch6_trigger_2_done = False
            self.get_logger().info(f"Trigger 4.4: X=3, BACK=3 (ch[6]={ch[6]})")

        # Reset trigger 3 flag if ch[6] returns to 1000 or above
        if ch[6] >= 1000:
            self.ch6_trigger_3_done = False

        # update ch[7] minimum value
        self.ch7_min_in_activation = min(self.ch7_min_in_activation, ch[7])
                # ch7
        if (not self.ch7_trigger_2_done and
                ch[7] > 1000 and
                ch[7] - self.ch7_min_in_activation > 800):
            self.X = 4
            self.BACK = 4
            self.ch7_trigger_2_done = True
            self.get_logger().info(f"Trigger: X=4, BACK=4 (ch[7]={ch[7]}, min={self.ch7_min_in_activation})")

        if not self.ch7_trigger_3_done and ch[7] < 1000:
            self.X = 5
            self.BACK = 5
            self.ch7_trigger_2_done = False  # Reset trigger 2 when trigger 3 is activated
            self.ch7_trigger_3_done = True
            self.get_logger().info(f"Trigger: X=5, BACK=5 (ch[7]={ch[7]})")

        if ch[7] >= 1000:
            self.ch7_trigger_3_done = False        # Build axes and buttons
        self.axes = [
            0.0, left_FB, 0.0, -right_LR, right_FB, self.RT, 0.0, left_LR
        ]
        self.buttons = [
            self.A, self.B, self.X, self.Y,
            self.LB, self.RB, self.BACK, 0, 0, 0, 0
        ]

        # Step 4.5: publish Joy message when mapped values change
        if (self.axes != self.axes_pre or self.buttons != self.buttons_pre):
            if self.debug_:
                print(f'State: {self.current_state.name}')
                print(f'axes: {self.axes}')
                print(f'buttons: {self.buttons}')

            self.joy_msg.axes = self.axes
            self.joy_msg.buttons = self.buttons
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy_msg)

        # Update previous values
        self.axes_pre = list(self.axes)
        self.buttons_pre = list(self.buttons)

        self.reset_buttons()

    def publish_disconnect(self):
        self.get_logger().warn("Publishing disconnect message")
        self.axes = [0.0] * 8
        self.buttons = [0] * 11
        self.joy_msg.axes = self.axes
        self.joy_msg.buttons = self.buttons
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.joy_pub.publish(self.joy_msg)

    async def read_by_sbus(self):
        sbus = None
        while not self.device_connect and rclpy.ok():
            try:
                sbus = await SBUSReceiver.create(self.device_port)
                self.device_connect = True
                self.get_logger().info(f"Connected to {self.device_port}")
            except Exception as e:
                if self.publish_first:
                    self.get_logger().warn(f'Cannot open {self.device_port}: {e}')
                    self.publish_first = False
                await asyncio.sleep(1)

        # After the initial connection, only SBUSReceiver handles reconnecting.
        # A frame timeout does not open the serial device again.
        while rclpy.ok():
            try:
                frame = await asyncio.wait_for(sbus.get_frame(), timeout=0.5)
                if not self.device_connect:
                    self.get_logger().info(
                        f"reconnect successfully to {self.device_port}")
                self.device_connect = True
                self.create_joy_msg(frame)
            except asyncio.TimeoutError:
                if self.device_connect:
                    self.device_connect = False
                    self.get_logger().error("Wait frame timeout")
                    self.publish_disconnect()
            except Exception as e:
                self.get_logger().error(f"Read sbus failed: {e}")
                await asyncio.sleep(0.1)




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
