#!/usr/bin/env python3
import serial
import time
import rospy
from std_msgs.msg import String

# Configure serial port (adjust the port name for your system)
serial_port = '/dev/ttyUSB0'  # Replace with the correct COM port for Windows (e.g., COM3)
baud_rate = 9600

# Initialize the ROS node
rospy.init_node('rmc_write', anonymous=True)

def callback(data):
    try:
        # Replace 'GNRMC' with 'GPRMC' in the received message
        modified_message = data.data.replace('GNRMC', 'GPRMC')

        # Open the serial port and send the modified message
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            ser.write((modified_message + '\r\n').encode('utf-8'))
            rospy.loginfo(f"Sent: {modified_message}")

    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

# Subscribe to the topic that publishes GNRMC sentences
rospy.Subscriber('/gnrmc_topic', String, callback)

# Keep the node running
rospy.spin()
