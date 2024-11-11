#!/usr/bin/env python3

import time
import logging
import rospy
from threading import Thread
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from datetime import datetime
from geometry_msgs.msg import Point, Quaternion, Twist

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

class LocalisationTests:
    def __init__(self):
        logger.info("Initializing ROS node 'dual_topic_publisher'")
        unique_name = "localisation_node_" + str(int(time.time()))
        rospy.init_node(unique_name)
        # Publishers for GPS and Odometry data
        self.gps_pub = rospy.Publisher("gps/fix", NavSatFix, queue_size=10)
        self.odom_pub = rospy.Publisher("odometry/filtered", Odometry, queue_size=10)

        self.rate = rospy.Rate(0.5)  # Publish rate at 0.5 Hz (once every 2 seconds)
        logger.info("Publishers initialized: 'gps/fix' and 'odometry/filtered' topics")

    def publish_dummy_gps(self):
        """Publishes dummy GPS data at a regular interval."""
        logger.info("Starting GPS publisher thread")
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            gps_msg = NavSatFix()
            gps_msg.header.stamp = current_time

            # Set dummy GPS data
            gps_msg.latitude = 37.7749
            gps_msg.longitude = -122.4194
            gps_msg.altitude = 0.0

            # Get the current time and format it as a string
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # Print the GPS data with the timestamp
            print(f"[{timestamp}] Publishing GPS data: Latitude={gps_msg.latitude:.4f}, "
                f"Longitude={gps_msg.longitude:.4f}, Altitude={gps_msg.altitude:.1f}")

            self.gps_pub.publish(gps_msg)
            self.rate.sleep()

    def publish_dummy_odometry(self):
        """Publishes dummy Odometry data at a regular interval."""
        logger.info("Starting Odometry publisher thread")
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            # Set dummy position and orientation
            odom_msg.pose.pose.position = Point(1.0, 2.0, 0.0)
            odom_msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            odom_msg.twist.twist = Twist()
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # Print the odometry data with the timestamp
            print(f"[{timestamp}] Publishing Odometry data: Position=({odom_msg.pose.pose.position.x:.1f}, "
                f"{odom_msg.pose.pose.position.y:.1f}, {odom_msg.pose.pose.position.z:.1f}), "
                f"Orientation=({odom_msg.pose.pose.orientation.x:.1f}, {odom_msg.pose.pose.orientation.y:.1f}, "
                f"{odom_msg.pose.pose.orientation.z:.1f}, {odom_msg.pose.pose.orientation.w:.1f})")
            self.odom_pub.publish(odom_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        logger.info("Starting LocalisationTests node")
        dual_topic_publisher = LocalisationTests()

        # Start two threads, one for each topic
        gps_thread = Thread(target=dual_topic_publisher.publish_dummy_gps)
        odom_thread = Thread(target=dual_topic_publisher.publish_dummy_odometry)

        logger.info("Launching GPS and Odometry publisher threads")
        gps_thread.start()
        odom_thread.start()

        # Monitor the thread states
        while not rospy.is_shutdown():
            print("Main loop running - GPS thread alive: %s, Odom thread alive: %s",
                         gps_thread.is_alive(), odom_thread.is_alive())
            rospy.sleep(5)  # Check every 5 seconds

        # Join threads after ROS shutdown
        gps_thread.join()
        odom_thread.join()
        logger.info("Shutting down LocalisationTests node")

    except rospy.ROSInterruptException:
        logger.error("ROS Interrupt Exception occurred, shutting down.")
