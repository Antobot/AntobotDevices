#!/usr/bin/env python3
import os
import sys
import yaml
import rospy
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

        rospack = rospkg.RosPack()
        configPath=rospack.get_path('antobot_description')
        config_file = configPath+ "/config/platform_config.yaml"
        paramPath = rospack.get_path('xsens_mti_driver')
        param_file = paramPath+ "/param/xsens_mti_node.yaml"

        imu_config = self.load_config()
        print("imu_config:\n",imu_config)
        
        # Initialise ROS Launcher
        #self._launch = roslaunch.scriptapi.ROSLaunch()
        #self._launch.start()

        for device, settings in imu_config.items():
            switch = settings.get("switch", False)
            if switch == True:
                if device=="urcu":
                    device_bno=True
                if device == "xsens":
                    device_xsens = True
                    if os.path.isfile(param_file):
                        rospy.loginfo("Loading parameters from %s", param_file)
                        xsens_paramlist=rosparam.load_file(param_file, default_namespace="/xsens_mti_node")  #
                        print(xsens_paramlist)
                        for params, ns in xsens_paramlist:
                            rospy.loginfo(f"Loading parameters to namespace: {ns}")
                            rosparam.upload_params(ns,params)#
                        #xsens_param_file=rospy.get_param('~raram_file','$(find xsens_mti_driver)/param/xsens_mti_node.yaml')
                        #rospy.loginfo(f"Loading parameters from: {xsens_param_file}")
                        #rospy.set_param('imu_xsens',rospy.get_param('',rospy.load_param(xsens_param_file)))
        #rospy.sleep(4)
        if rospy.has_param("/xsens_mti_node/port"):
            rospy.loginfo("parameter port found in param server")
        else:
            rospy.logerr("parameter port not found in param server")
        self.createLauncher(device_bno, device_xsens)
        #print("launching executable {}".format(exec_name))



    def load_config(self):
        """Load platform_config."""
        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_description')
        path = packagePath + "/config/platform_config.yaml"
        with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            imu_data = data['imu']
        return imu_data



    def createLauncher(self, device_bno, device_xsens):


        nodes_to_launch = []
        
        #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        if device_bno  and not device_xsens :
                rospy.loginfo("Launching BNO550 IMU ")
               
                #launch.start()
                node_bno = roslaunch.core.Node(package="am_bno055_imu",
                                           node_type="am_bno055_imu_node",
                                           name="imu_bno",namespace="imu")
                #nodes_to_launch.append(node)
                launch.launch(node_bno)
                #launch = roslaunch.parent.ROSLaunchParent(uuid, nodes_to_launch)
                
                #launcher = RoslaunchWrapperObject(uuid, nodes_to_launch)
                #launcher.start_node_name("imu_bno")
                #launch.start()

        elif not device_bno  and  device_xsens :
           

                rospy.loginfo("Launching Xsens IMU")
                node_xsens = roslaunch.core.Node(package="xsens_mti_driver",
                                           node_type="xsens_mti_node",
                                           name="imu_xsens",namespace="xsens_mti_node",output="screen")
                #nodes_to_launch.append(node)
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/catkin_ws/src/towerScan/AntoExternal/xsens_ros_mti_driver/launch/xsens_mti_node.launch"])
                
                #launcher = RoslaunchWrapperObject(uuid, nodes_to_launch)
                #launcher.start_node_name("imu_xsens")
                #launch.launch(node_xsens)
                launch.start()

        elif device_bno  and  device_xsens:
            # When both device are enabled, bno's IMU to remain on the default topic ("imu/data")
            # and remap the xsens’s topic accordingly.
                
                rospy.loginfo("Launching BNO's node (default imu/data) and XSENS's node remapped to /imu/data/xsens")
                node_bno = roslaunch.core.Node(package="am_bno055_imu",
                                           node_type="am_bno055_imu_node",
                                           name="imu_bno",namespace="bno/imu")
                launch.launch(node_bno)
                
                node_xsens = roslaunch.core.Node(package="xsens_mti_driver",
                                           node_type="xsens_mti_node",
                                           name="imu_xsens",
                                             )
               
                
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                """
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/catkin_ws/src/towerScan/AntoExternal/xsens_ros_mti_driver/launch/xsens_mti_node_remap.launch"])
                """
                cli_args=['xsens_mti_driver','xsens_mti_node.launch','name_space:=xsens']
                roslaunch_xsens = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
                roslaunch_xsens_args = cli_args[2:]
                launch_files=[(roslaunch_xsens,roslaunch_xsens_args)]
                parent=roslaunch.parent.ROSLaunchParent(uuid,launch_files)
               
                #nodes_to_launch.extend([node_bno, node_xsens])
                #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                #roslaunch.configure_logging(uuid)
                #launch = roslaunch.parent.ROSLaunchParent(uuid,nodes_to_launch)
                #launch.start()
                #launcher = RoslaunchWrapperObject(uuid, nodes_to_launch)
                #launcher.start_node_name("imu_bno")
                #launcher.start_node_name("imu_xsens")
                
                #launch = roslaunch.parent.ROSLaunchParent(uuid, [node_xsens])
                
                parent.start()
                
            
            
        else:
            rospy.loginfo("No IMU device enabled in YAML. Nothing to launch.")
            sys.exit(0)

        
        

def main():
        try:
            rospy.init_node('imuManager')
            imuMgr = imuManager()
            rate = rospy.Rate(50) 
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down launcher...")
        finally:
            pass

if __name__ == '__main__':
        main()
