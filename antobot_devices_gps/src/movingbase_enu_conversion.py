#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64


def calculate_enu_angle(xA, yA, xB, yB):
    # Compute the differences in the ENU (East-North) coordinate system
    dx = xB - xA  # East-West difference (X axis)
    dy = yB - yA  # North-South difference (Y axis)
    
    # Calculate the angle using atan2, relative to the East (positive X-axis)
    angle_radians = math.atan2(dy, dx)
    
    # Convert the angle to degrees
    angle_degrees = math.degrees(angle_radians)
    
    # Normalize the angle to be within 0 to 360 degrees
    if angle_degrees < 0:
        angle_degrees += 360
    
    return angle_degrees

class MovingBase_Heading:

    def __init__(self,angle):
        self.angle = angle # angle between the vector created by the two antenna and the x axis of the base_link

        self.sub_heading_enu = rospy.Subscriber('am_heading_enu', Float64, self.headingCallback)
        self.pub_heading_robot = rospy.Publisher('am_heading_robot', Float64, queue_size=10)


    def headingCallback(self,msg_enu):
        msgs = Float64()
        msgs.data = ((msg_enu.data) - self.angle)%360.0 # Normalise the result to be within 0 to 360
        self.pub_heading_robot.publish(msgs)
        
if __name__ == '__main__':
    
    rospy.init_node('ENUConversion', anonymous=True)

    # Only considering px and py (two antennas should be placed at the same height)
    urcu_px = rospy.get_param("/gps/urcu/px",0.1)
    urcu_py = rospy.get_param("/gps/urcu/py",0.6)

    rover_px = rospy.get_param("/gps/ublox_rover/px",0.1)
    rover_py = rospy.get_param("/gps/ublox_rover/py",-0.6)

    angle = calculate_enu_angle(urcu_px, urcu_py, rover_px, rover_py)
    print(f"ENU Angle (relative to East): {angle:.2f} degrees")
    
    MH = MovingBase_Heading(angle)

    rospy.spin()

  

