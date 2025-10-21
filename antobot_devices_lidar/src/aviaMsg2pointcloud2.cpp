#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>

class aviaMsgNode : public rclcpp::Node
{
public:
    aviaMsgNode() : Node("aviaMsgNode") {
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>("", 10, std::bind(&aviaMsgNode::lidarCB, this, std::placeholders::_1));

        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);

        input_lidar_topic = config["input_lidar_topic"].as<std::string>();
        output_lidar_topic = config["output_lidar_topic"].as<std::string>();
        output_lidar_frame = config["output_lidar_frame"].as<std::string>();
        filter_num = config["filter_num"].as<int>();
        lidar_max_range = config["lidar_max_range"].as<double>();
        lidar_min_range = config["lidar_min_range"].as<double>();

        m_lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_lidar_topic, 10000);



    }


private:


    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud->clear();

        cloud = livox2PCL(msg, filter_num, lidar_min_range, lidar_max_range);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = msg->header.stamp;
        cloud_msg.header.frame_id = output_lidar_frame;
        m_lidar_pub->publish(cloud_msg);


    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg,
        int filter_num, double min_range, double max_range)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        int point_num = msg->point_num;
        cloud->reserve(point_num / filter_num + 1);
        for (int i = 0; i < point_num; i += filter_num)
        {
            if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {

                float x = msg->points[i].x;
                float y = msg->points[i].y;
                float z = msg->points[i].z;
                if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                    continue;
                pcl::PointXYZINormal p;
                p.x = x;
                p.y = y;
                p.z = z;
                p.intensity = msg->points[i].reflectivity;
                p.curvature = msg->points[i].offset_time / 1000000.0f;
                cloud->push_back(p);
            }
        }
        return cloud;
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_pub;


    std::string input_lidar_topic = "livox/lidar_192_168_1_200";
    std::string output_lidar_frame = "livox_frame_pointcloud2";
    int filter_num = 3;
    double lidar_max_range = 20.0;
    double lidar_min_range = 0.0;

};
