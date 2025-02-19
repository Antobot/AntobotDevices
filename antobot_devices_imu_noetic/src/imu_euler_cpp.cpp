/*
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: 	The primary purpose of this code is to create a new frame called laser_imu using imu's roll and pitch
#               data. By creating this new laser_imu frame that will be the origin frame of the 3D lidar, we can compnesate
#               the roll and pitch angle of the robot. This will avoid including the floor to the costmap when the robot 
#               is looking downwards. Currently this is only useful for bumps or irregular floor condition in the flat farm.  
# Subscribes to: imu topic (/imu/data_corrected) - after calibration 
# Publishes : laser_imu tf frame - which is then used as the frame for 3D lidar (when roll pitch angle compensation is enabled)

*/

#include <ros/ros.h>
#include <vector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>


tf2::Quaternion q_imu;
std::vector<double> angles;
bool imu_received = false;

std::vector<double> eulerFromQuat(tf2::Quaternion q){
    // Description: Convert tf2::Quaternion to euler and return vector<double> that is filled with euler values. 
    // Input: tf2::Quaternion q
    // Output: vector with euler value - roll, pitch, yaw 

    tf2::Matrix3x3 m(q);
    double r,p,y;
    m.getRPY(r,p,y);
    std::vector<double> v;
    v.push_back(r);
    v.push_back(p);
    v.push_back(y);
    return v; 
}

tf2::Quaternion quatFromEuler(double a, double b, double c){
    // Description: Convert euler values to tf2::Quaternion 
    // Input: euler value - roll, pitch, yaw 
    // Returns: tf2::Quaternion

    tf2::Quaternion q;
    q.setRPY(a,b,c);
    q =q.normalize();
    return q;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // Description: IMU callback function (IMU sensor raw data)
    tf2::convert(msg->orientation , q_imu);
    angles = eulerFromQuat(q_imu);
    if (!imu_received) imu_received = true;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "imu_euler_cpp");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped1;
    std::vector<double> rpy1;

    try{
        transformStamped1 = tfBuffer.lookupTransform("base_link", "laser_link_front_static", ros::Time::now(), ros::Duration(10.0)); // to prevent node dying 
        tf2::Quaternion q_tmp;
        tf2::convert(transformStamped1.transform.rotation , q_tmp);
        rpy1 = eulerFromQuat(q_tmp);
        ROS_INFO("Front lidar excists");
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ROS_ERROR("Front lidar frame not available - Node will have seg fault");
      ros::Duration(1.0).sleep();
    }
    
    bool rear_lidar_avail = true;
    geometry_msgs::TransformStamped transformStamped2;
    std::vector<double> rpy2;

    try{
        transformStamped2 = tfBuffer.lookupTransform("base_link", "laser_link_back_static", ros::Time::now(), ros::Duration(3.0));
        tf2::Quaternion q_tmp;
        tf2::convert(transformStamped2.transform.rotation , q_tmp);
        rpy2 = eulerFromQuat(q_tmp);
        ROS_INFO("Back lidar exists");
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ROS_WARN("Back lidar frame not available");
      rear_lidar_avail = false;
      ros::Duration(1.0).sleep();
    }
    

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/imu/data", 1, imuCallback);
    
    tf2::Quaternion q_cur_imu, q_front, q_rear, q_result;
    q_front = quatFromEuler(0,rpy1[1],rpy1[2]); //RPY
    
    geometry_msgs::TransformStamped transformStamped_front_tmp;

    transformStamped_front_tmp.header.frame_id = "base_link";
    transformStamped_front_tmp.child_frame_id = "laser_tmp_1";

    geometry_msgs::TransformStamped transformStamped_front;

    transformStamped_front.header.frame_id = "laser_tmp_1";
    transformStamped_front.child_frame_id = "laser_link_front";

    geometry_msgs::TransformStamped transformStamped_rear_tmp;

    transformStamped_rear_tmp.header.frame_id = "base_link";
    transformStamped_rear_tmp.child_frame_id = "laser_tmp_2";

    geometry_msgs::TransformStamped transformStamped_rear;

    transformStamped_rear.header.frame_id = "laser_tmp_2";
    transformStamped_rear.child_frame_id = "laser_link_back";

    if (rear_lidar_avail){
        q_rear = quatFromEuler(0,rpy2[1],rpy2[2]); //RPY
    }
    

    // Spin
    ros::Rate r(50); // 10 hz
    while (ros::ok())
    {   
        if (imu_received){
            // front lidar
            q_cur_imu = quatFromEuler(angles[0],0,0); //RPY
            
            q_result = q_cur_imu * q_front;
            q_result.normalize();

            transformStamped_front_tmp.header.stamp = ros::Time::now();
            transformStamped_front_tmp.transform.translation.x = transformStamped1.transform.translation.x;
            transformStamped_front_tmp.transform.translation.y = transformStamped1.transform.translation.y;
            transformStamped_front_tmp.transform.translation.z = 0.0;

            transformStamped_front_tmp.transform.rotation.x = q_result.x();
            transformStamped_front_tmp.transform.rotation.y = q_result.y();
            transformStamped_front_tmp.transform.rotation.z = q_result.z();
            transformStamped_front_tmp.transform.rotation.w = q_result.w();

            br.sendTransform(transformStamped_front_tmp);

            transformStamped_front.header.stamp = ros::Time::now();
            transformStamped_front.transform.translation.x = 0.0;
            transformStamped_front.transform.translation.y = 0.0;
            transformStamped_front.transform.translation.z = transformStamped1.transform.translation.z;
            q_result = quatFromEuler(0,0,0); //RPY
            transformStamped_front.transform.rotation.x = q_result.x();
            transformStamped_front.transform.rotation.y = q_result.y();
            transformStamped_front.transform.rotation.z = q_result.z();
            transformStamped_front.transform.rotation.w = q_result.w();

            br.sendTransform(transformStamped_front);
            

            // rear lidar
            if(rear_lidar_avail){
                //ROS_INFO("rear lidar found");

                q_result = q_cur_imu * q_rear;
                q_result.normalize();

                transformStamped_rear_tmp.header.stamp = ros::Time::now();
                transformStamped_rear_tmp.transform.translation.x = transformStamped2.transform.translation.x;
                transformStamped_rear_tmp.transform.translation.y = transformStamped2.transform.translation.y;
                transformStamped_rear_tmp.transform.translation.z = 0.0;

                transformStamped_rear_tmp.transform.rotation.x = q_result.x();
                transformStamped_rear_tmp.transform.rotation.y = q_result.y();
                transformStamped_rear_tmp.transform.rotation.z = q_result.z();
                transformStamped_rear_tmp.transform.rotation.w = q_result.w();

                br.sendTransform(transformStamped_rear_tmp);

                transformStamped_rear.header.stamp = ros::Time::now();
                transformStamped_rear.transform.translation.x = 0.0;
                transformStamped_rear.transform.translation.y = 0.0;
                transformStamped_rear.transform.translation.z = transformStamped2.transform.translation.z;
                q_result = quatFromEuler(0,0,0); //RPY
                transformStamped_rear.transform.rotation.x = q_result.x();
                transformStamped_rear.transform.rotation.y = q_result.y();
                transformStamped_rear.transform.rotation.z = q_result.z();
                transformStamped_rear.transform.rotation.w = q_result.w();

                br.sendTransform(transformStamped_rear);

            }

        }
        ros::spinOnce();
        r.sleep();
        }
        }
