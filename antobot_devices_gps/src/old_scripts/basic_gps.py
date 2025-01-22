#!/usr/bin/env python3
import sys
from serial import Serial
from pyubx2 import UBXReader

import rospy
from sensor_msgs.msg import NavSatFix


class BasicGPS:
    def __init__(self):
        self.stream = Serial(port='/dev/ttyUSB0', baudrate=38400)
        self.stream.flush()
        self.ubr = UBXReader(self.stream)

        self.gps_pub = rospy.Publisher('gps', NavSatFix, queue_size=1)

    def run(self):
        (raw_data,parsed_data) = self.ubr.read()
            
        if type(parsed_data) is not str:
            if parsed_data.identity[2:5] == 'GGA':
                print("Parsed data: ",parsed_data)
                # print("Parsed data: ",parsed_data)
                # print("Latitude is : ", parsed_data.lat)
                # print("Longitude is: ",parsed_data.lon)
                # print("Number of satellites available: ", parsed_data.numSV)
                
                msg = NavSatFix()
                msg.header.stamp = rospy.Time.now()
                msg.latitude = parsed_data.lat
                msg.longitude = parsed_data.lon
                msg.altitude = parsed_data.alt

                self.gps_pub.publish(msg)   


def main(argv):
    rospy.init_node('gps_publisher')

    gps = BasicGPS()

    while not rospy.is_shutdown():
        try:
            gps.run()
        except KeyboardInterrupt:
            print("Terminated by keyboard interrupt")
    

if __name__ == '__main__':
    main(sys.argv)