#!/usr/bin/env python3
import os
import sys
import yaml
import rospy
import roslaunch
import rospkg

class imuManager():
    def __init__(self):
        device_BNO=False
        device_xsens = False
        configPath=rospack.get_path('antobot_description')
        config_file = os.path.join(configPath, "/config/platform_config.yaml")
        paramPath = rospack.get_path('xsens_mti_driver')
        param_file = os.path.join(paramPath, "/param/xsens_mti_node.yaml")
        if not os.path.isfile(config_file):
            rospy.logerr("Configuration file %s not found.", config_file)
            sys.exit(1)
        imu_config = self.load_config()

        
        
        # Initialise ROS Launcher
        self._launch = roslaunch.scriptapi.ROSLaunch()
        self._launch.start()
        for device, settings in imu_config.items():
            switch = settings.get("switch", False)
            if switch == True:
                if device="urcu":
                    device_bno=True
                if device = "xsens":
                    device_xsens = True
                    if os.path.isfile(param_file):
                        rospy.loginfo("Loading parameters from %s", param_file)
                        rosparam.load_file(param_file, default_namespace="/", verbose=True)
        self.createLauncher(device_bno, device_xsens)
        print("launching executable {}".format(exec_name))



    def load_config(config_path):
        """Load platform_config."""
        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_description')
        path = packagePath + "/config/platform_config.yaml"
        with open(config_path, 'r') as yamlfile:
            data = yaml.safe_load(f)
            imu_data = date['imu']
        return imu_data



    def createLauncher(self, device_bno, device_xsens):


        nodes_to_launch = []
        

        if device_bno  and not device_xsens :
                rospy.loginfo("Launching BNO550 IMU ")
                node = roslaunch.core.Node(package="am_bno055_imu",
                                           node_type="am_bno055_imu_node",
                                           name="imu_bno")
                nodes_to_launch.append(node)

        elif not device_bno  and  device_xsens :
           

                rospy.loginfo("Launching Xsens IMU")
                node = roslaunch.core.Node(package="xsens_mti_driver",
                                           node_type="xsens_mti_node",
                                           name="imu_xsens")
                nodes_to_launch.append(node)
            

        elif device_bno  and  device_xsens:
            # When both device are enabled, bno's IMU to remain on the default topic ("imu/data")
            # and remap the xsens’s topic accordingly.
            
                rospy.loginfo("Launching BNO's node (default imu/data) and XSENS's node remapped to /imu/data/xsens")
                node_bno = roslaunch.core.Node(package="am_bno055_imu",
                                           node_type="am_bno055_imu_node",
                                           name="imu_bno")
                node_xsens = roslaunch.core.Node(package="xsens_mti_driver",
                                           node_type="xsens_mti_node",
                                           name="imu_xsens",
                                             remappings=[("imu/data", "/imu/data/xsens")])
                nodes_to_launch.extend([node_bno, node_xsens])
            
            
        else:
            rospy.loginfo("No IMU device enabled in YAML. Nothing to launch.")
            sys.exit(0)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, nodes_to_launch)
        launch.start()
        

    def main():
        try:
            rospy.init_node('imuManager', anonymous=True)
            rate = rospy.Rate(50) 
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down launcher...")
        finally:
            launch.shutdown()

    if __name__ == '__main__':
        main()
