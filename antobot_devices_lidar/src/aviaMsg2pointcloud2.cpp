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
        this->declare_parameter("config_path", "/home/yu/antobot/install/antobot_devices_lidar/share/antobot_devices_lidar/config/aviaMsgConvert.yaml");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);

        input_lidar_topic1 = config["input_lidar_topic1"].as<std::string>();
        input_lidar_topic2 = config["input_lidar_topic2"].as<std::string>();
        output_lidar_topic1 = config["output_lidar_topic1"].as<std::string>();
        output_lidar_topic2 = config["output_lidar_topic2"].as<std::string>();
        output_lidar_frame1 = config["output_lidar_frame1"].as<std::string>();
        output_lidar_frame2 = config["output_lidar_frame2"].as<std::string>();

        lidar_num = config["lidar_num"].as<int>();
        filter_num = config["filter_num"].as<int>();

        lidar_max_range = config["lidar_max_range"].as<double>();
        lidar_min_range = config["lidar_min_range"].as<double>();

        if (lidar_num == 1) {
            m_lidar_sub_1 = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(input_lidar_topic1, 10, std::bind(&aviaMsgNode::lidarCB_1, this, std::placeholders::_1));
        }else if (lidar_num == 2) {
            m_lidar_sub_1 = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(input_lidar_topic1, 10, std::bind(&aviaMsgNode::lidarCB_1, this, std::placeholders::_1));
            m_lidar_sub_2 = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(input_lidar_topic2, 10, std::bind(&aviaMsgNode::lidarCB_2, this, std::placeholders::_1));
        }


        m_lidar_pub_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_lidar_topic1, 10000);
        m_lidar_pub_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_lidar_topic2, 10000);


    }


private:


    void lidarCB_1(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud->clear();

        cloud = livox2PCL(msg, filter_num, lidar_min_range, lidar_max_range);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = msg->header.stamp;
        cloud_msg.header.frame_id = output_lidar_frame1;
        m_lidar_pub_1->publish(cloud_msg);
    }
    void lidarCB_2(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud->clear();

        cloud = livox2PCL(msg, filter_num, lidar_min_range, lidar_max_range);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = msg->header.stamp;
        cloud_msg.header.frame_id = output_lidar_frame2;
        m_lidar_pub_2->publish(cloud_msg);
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

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub_1;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub_2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_pub_1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_pub_2;


    std::string input_lidar_topic1 = "livox/lidar_192_168_1_200";
    std::string input_lidar_topic2 = "livox/lidar_192_168_1_201";
    std::string output_lidar_frame1 = "livox_frame_pointcloud2";
    std::string output_lidar_frame2 = "livox_frame_pointcloud2";
    std::string output_lidar_topic1 = "livox_pointcloud2";
    std::string output_lidar_topic2 = "livox_pointcloud2";
    int lidar_num = 1;
    int filter_num = 3;
    double lidar_max_range = 20.0;
    double lidar_min_range = 0.0;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aviaMsgNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
