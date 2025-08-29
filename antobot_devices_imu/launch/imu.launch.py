from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import rclpy
import yaml
import os
from ament_index_python.packages import get_package_share_directory

packagePath=get_package_share_directory('antobot_description')
path = packagePath + "/config/platform_config.yaml"
nodelist = []

def generate_launch_description():
    waittime=0.0
    icm_node=Node(
            name='icm_imu',
            package='icm_imu',
            namespace='/icm',
            executable='icm_node',
        )
    bno_node=Node(
            name='bno_imu',
            package='bno055_imu',
            namespace='/bno',
            executable='bno055_i2c_node',
            output='screen',
            parameters=[{'use_sim_time': False},{'publish_rate': 100.0}],
        )
    config = os.path.join(
        get_package_share_directory('tm_imu'),
        'config',
        'params.yaml'
    )
    tm_node=Node(
            name='tm_imu',
            package='tm_imu',
            namespace='/tm',
            executable='transducer_m_imu',
            parameters=[{'use_sim_time': False},config],
        )
    packagePath=get_package_share_directory('antobot_description')
    path = packagePath + "/config/platform_config.yaml"
    with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            imu_data = data['imu']

    for device, settings in imu_data.items():
            switch = settings.get("switch", False)
            mode = settings.get("mode")
            if switch == True:
                if device=="bno":
                    if mode == "navigation":
                        bno_node=Node(package='bno055_imu',namespace='/imu', executable='bno055_i2c_node',)
                    nodelist.append(TimerAction(period=waittime,actions=[bno_node]))
                    waittime=waittime+1.0
                
                if device == "xsens":
                    if mode == "navigation":
                        xsens_node=Node(package='xsens_mti_driver',namespace='/imu', executable='xsens_mti_node',)
                    nodelist.append(TimerAction(period=waittime,actions=[xsens_node]))
                    waittime=waittime+1.0
                if device=="icm":
                    if mode == "navigation":
                        icm_node=Node(package='icm_imu',namespace='/imu', executable='icm_node',)
                    nodelist.append(TimerAction(period=waittime,actions=[icm_node])) 
                    waittime=waittime+1.0
                if device=="ahrs":
                    if mode == "navigation":
                        tm_node=Node(package='tm_imu',namespace='/imu', executable='transducer_m_imu',)
                    nodelist.append(TimerAction(period=waittime,actions=[tm_node]))  
                    waittime=waittime+1.0
                    
    return LaunchDescription(nodelist)
    
