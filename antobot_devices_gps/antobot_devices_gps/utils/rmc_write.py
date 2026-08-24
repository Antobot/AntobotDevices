#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configure serial port (adjust the port name for your system)
serial_port = '/dev/ttyUSB0'  # Replace with the correct COM port for Windows (e.g., COM3)
baud_rate = 9600


class RmcWrite(Node):
    def __init__(self):
        super().__init__('rmc_write')
        # Subscribe to the topic that publishes GNRMC sentences
        self.sub = self.create_subscription(String, '/gnrmc_topic', self.callback, 10)

    def callback(self, data):
        try:
            # Replace 'GNRMC' with 'GPRMC' in the received message
            modified_message = data.data.replace('GNRMC', 'GPRMC')

            # Open the serial port and send the modified message
            with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
                ser.write((modified_message + '\r\n').encode('utf-8'))
                self.get_logger().info(f"Sent: {modified_message}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RmcWrite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
