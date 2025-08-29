#!/usr/bin/env python3
import os
import sys
import yaml
import rclpy
import time
import roslaunch
import rospkg
import roslib
roslib.load_manifest("rosparam")
import rosparam
#from antobot_manager_software.launchManager import Node,RoslaunchWrapperObject


class imuManager():
    def __init__(self):
        device_bno=False
        device_xsens = False
        nav_imu = "urcu"
        rospack = rospkg.RosPack()
        configPath=rospack.get_path('antobot_description')
        config_file = configPath+ "/config/platform_config.yaml"
        #paramPath = rospack.get_path('xsens_mti_driver')
        #param_file = paramPath+ "/param/xsens_mti_node.yaml"

        imu_config = self.load_config()


        for device, settings in imu_config.items():
            switch = settings.get("switch", False)
            mode = settings.get("mode")
            if switch == True:
                if device=="urcu":
                    device_bno=True
                    if mode == "navigation":
                        nav_imu = device
                    else:
                        nav_imu = "xsens"

                if device == "xsens":
                    device_xsens = True
                    if mode == "navigation":
                        nav_imu = device
                    else:
                        nav_imu = "urcu"
        print(nav_imu)
        self.createLauncher(device_bno, device_xsens,nav_imu)



    def load_config(self):
        """Load platform_config."""
        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_description')
        path = packagePath + "/config/platform_config.yaml"
        with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            imu_data = data['imu']
        return imu_data



    def createLauncher(self, device_bno, device_xsens,nav_imu):

 
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        if device_bno  and not device_xsens :
            rclpy.loginfo("Launching BNO550 IMU ")
            node_bno = roslaunch.core.Node(package="am_bno055_imu",
                                           node_type="am_bno055_imu_node",
                                           name="imu_bno",namespace="imu")
            launch.launch(node_bno)

        elif not device_bno  and  device_xsens :
            rclpy.loginfo("Launching Xsens IMU")
            # node_xsens = roslaunch.core.Node(package="xsens_mti_driver",
            #                                node_type="xsens_mti_node",
            #                                name="imu_xsens",namespace="/",output="screen")    
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            cli_args=['xsens_mti_driver','xsens_mti_node.launch']
            #launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/catkin_ws/src/towerScan/AntoExternal/xsens_ros_mti_driver/launch/xsens_mti_node.launch"])
            #launch.start()
            roslaunch_xsens = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            roslaunch_xsens_args = cli_args[2:]
            launch_files=[(roslaunch_xsens,roslaunch_xsens_args)]
            parent=roslaunch.parent.ROSLaunchParent(uuid,launch_files)     
            parent.start()

        elif device_bno  and  device_xsens:
            # When both device are enabled, bno's IMU to remain on the default topic ("imu/data")
            # and rename the xsens’s topic accordingly.
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            if nav_imu == "urcu":
                rclpy.loginfo("Launching BNO's node (/imu/data) and XSENS's node  /xsens/imu/data")
                node_bno = roslaunch.core.Node(package="am_bno055_imu",
                                           node_type="am_bno055_imu_node",
                                           name="imu",namespace="imu")  
                cli_args=['xsens_mti_driver','xsens_mti_node.launch','name_space:=xsens']
            else:
                rclpy.loginfo("Launching BNO's node (/bno/imu/data) and XSENS's node /imu/data")
                node_bno = roslaunch.core.Node(package="am_bno055_imu",
                                           node_type="am_bno055_imu_node",
                                           name="imu",namespace="bno/imu")  
                cli_args=['xsens_mti_driver','xsens_mti_node.launch']


            launch.launch(node_bno)
            roslaunch_xsens = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            roslaunch_xsens_args = cli_args[2:]
            launch_files=[(roslaunch_xsens,roslaunch_xsens_args)]
            parent=roslaunch.parent.ROSLaunchParent(uuid,launch_files)     
            parent.start()
                
            
            
        else:
            rclpy.loginfo("No IMU device enabled in YAML. Nothing to launch.")
            sys.exit(0)

        
        

def main():
        try:
            rclpy.init_node('imuManager')
            imuMgr = imuManager()
            rate = rclpy.Rate(50) 
            while not rclpy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            rclpy.loginfo("Shutting down launcher...")
        finally:
            pass

if __name__ == '__main__':
        main()
