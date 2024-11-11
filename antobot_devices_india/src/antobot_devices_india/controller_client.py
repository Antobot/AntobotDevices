#!/usr/bin/env python3

import time
import logging
import rospy
from antobot_devices_india.srv import Start, StartResponse, Stop, StopResponse
from antobot_devices_india.srv import Ready, ReadyResponse, Config, ConfigResponse
from antobot_devices_india.srv import Restart, RestartResponse, Sleep, SleepResponse
import numpy as np

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

class ControllerClient:
    """This class provides control server testing functionality.

    Attributes:
        rate (float): The rate at which the control server operates.
    """

    def __init__(self): 
        """Initialize the ControllerClient class."""
        logger.info("Initializing ROS node 'control_service_client'")
        

    def config_client(self):
        """Configure the control server.

        Returns:
            int: The return code of the configuration service.
        """
        print("Waiting for '/config' service to become available")
        rospy.wait_for_service(service='/config')
        try:
            ready_service = rospy.ServiceProxy(name='/config', service_class=Config)
            print("Calling '/config' service")
            response = ready_service()
            logger.info("Received response from '/config': Return Code=%d", response.return_code)
            return response.return_code
        except rospy.ServiceException as e:
            logger.error("Service call to '/config' failed: %s", str(e))
            return 10
    
    def ready_client(self):
        """Check if the control server is ready.

        Returns:
            int: The return code of the ready service.
        """
        print("Waiting for '/ready' service to become available")
        rospy.wait_for_service(service='/ready')
        try:
            ready_service = rospy.ServiceProxy(name='/ready', service_class=Ready)
            print("Calling '/ready' service")
            response = ready_service()
            logger.info("Received response from '/ready': Return Code=%d", response.return_code)
            return response.return_code
        except rospy.ServiceException as e:
            logger.error("Service call to '/ready' failed: %s", str(e))
            return 10
        
    def start_client(self):
        """Start the control server.

        Returns:
            int: The return code of the start service.
        """
        print("Waiting for '/start' service to become available")
        rospy.wait_for_service(service='/start')
        try:
            start_service = rospy.ServiceProxy(name='/start', service_class=Start)
            print("Calling '/start' service")
            response = start_service()
            logger.info("Received response from '/start': Return Code=%d", response.return_code)
            return response.return_code
        except rospy.ServiceException as e:
            logger.error("Service call to '/start' failed: %s", str(e))
            return 10
        
    def stop_client(self):
        """Stop the control server.

        Returns:
            int: The return code of the stop service.
        """
        print("Waiting for '/stop' service to become available")
        rospy.wait_for_service(service='/stop')
        try:
            ready_service = rospy.ServiceProxy(name='/stop', service_class=Stop)
            print("Calling '/stop' service")
            response = ready_service()
            logger.info("Received response from '/stop': Return Code=%d", response.return_code)
            return response.return_code
        except rospy.ServiceException as e:
            logger.error("Service call to '/stop' failed: %s", str(e))
            return 10
        
    def restart_client(self):
        """Restart the control server.

        Returns:
            int: The return code of the restart service.
        """
        print("Waiting for '/restart' service to become available")
        rospy.wait_for_service(service='/restart')
        try:
            ready_service = rospy.ServiceProxy(name='/restart', service_class=Restart)
            print("Calling '/restart' service")
            response = ready_service()
            logger.info("Received response from '/restart': Return Code=%d", response.return_code)
            return response.return_code
        except rospy.ServiceException as e:
            logger.error("Service call to '/restart' failed: %s", str(e))
            return 10
        
    def sleep_client(self):
        """Put the control server to sleep.

        Returns:
            int: The return code of the sleep service.
        """
        print("Waiting for '/sleep' service to become available")
        rospy.wait_for_service(service='/sleep')
        try:
            ready_service = rospy.ServiceProxy(name='/sleep', service_class=Sleep)
            print("Calling '/sleep' service")
            response = ready_service()
            logger.info("Received response from '/sleep': Return Code=%d", response.return_code)
            return response.return_code
        except rospy.ServiceException as e:
            logger.error("Service call to '/sleep' failed: %s", str(e))
            return 10

if __name__ == "__main__":
    # Initialise the ControllerClient class

    unique_name = "controller_node_" + str(int(time.time()))
    rospy.init_node(unique_name)

    ct = ControllerClient()
    ct.rate = rospy.Rate(hz=0.5)

    logger.info("Starting control service test loop")
    while True:
        # Make the ready service call
        response = ct.ready_client()
        if response == 0:
            logger.info("Ready call successful. Return Code: %d", response)
            response = ct.start_client()
            if response == 0:
                logger.info("Start call successful. Return Code: %d", response)
                time.sleep(5)
                while True:
                    response = ct.stop_client()
                    if response == 0:
                        logger.info("Stop call successful. Return Code: %d", response)
                        break
                    else:
                        time.sleep(1)
        else:
            logger.error("Service call failed.")

        print("Sleeping before the next loop iteration")
        time.sleep(3600)
