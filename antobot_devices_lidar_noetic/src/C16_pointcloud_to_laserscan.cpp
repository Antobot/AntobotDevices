/*
# Copyright (c) 2019, ANTOBOT LTD.
# All rights reserved.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description:    

Contacts: soyoung.kim@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <vector>
#include <queue>
#include <limits>

ros::Publisher pub;

// struct to save range (distance) and angle (-PI ~ PI)
struct point {
    double range;
    double angle;
};

// comparing function used for sorting the unordered points
struct classcomp
{
    bool operator() (const point& a, const point& b) const
    {
        return a.angle > b.angle;
    }
};

void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // build laserscan output
    sensor_msgs::LaserScan output;
    output.header= cloud_msg->header;
    output.header.frame_id = "laser_front";


    output.angle_min = -3.14159265;
    output.angle_max = 3.14159265;
    output.angle_increment = 0.003141593;
    output.time_increment = 0.0;
    output.scan_time = 0.1;
    output.range_min = 0.1;
    output.range_max = 10.0;

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    sensor_msgs::PointCloud2ConstPtr cloud_out;
    cloud_out = cloud_msg;

    std::vector<point> tmp;

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
        iter_z(*cloud_out, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        double x = -1*(*iter_x);
        double y = -1*(*iter_y);
        double z = (*iter_z);

        // Needs tuning, which height we will use
        double target_height = 0.7;
        double threshold = 0.2; // +/- threshold
        if ((target_height - threshold < z) && (z < target_height + threshold) ){ 
            double range = hypot(x, y);
            double angle = atan2(y, x);
            if (range < output.range_min)
            {
                ROS_WARN("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, 0.1, x, y, z);
                point a;
                a.range = -1;
                a.angle = angle;
                tmp.push_back(a);
                continue;
            }
            if (range > output.range_max)
            {
                //ROS_WARN("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, 30.0, x, y, z);
                point a;
                a.range = -1;
                a.angle = angle;
                tmp.push_back(a);
                continue;
            }

            // if (angle < -1.0472 || angle > 1.0472)
            // {
            //     ROS_WARN("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
            //     //tmp.push_back(std::make_pair(-1,angle))
            //     continue;
            // }

            point a;
            a.range = range;
            a.angle = angle;
            tmp.push_back(a);
            
        }
        else{
            continue;
        }
    }

    int n = tmp.size();
    ROS_WARN("size after filtering %d",n);

    // Sort the unordered points based on the angle values
    std::priority_queue<point, std::vector<point>, classcomp> priQue;            
    for (unsigned int i = 0; i < tmp.size(); ++i)
        priQue.push(tmp[i]);

    // Fill in the laserscan data with the sorted points
    int index_prev = -1;
    while (!priQue.empty())
    {
        const point a = priQue.top();
        //std::cout << "x:" << point.x << ", " << "y:" << point.y << ", z:" << point.z << std::endl;
        int index = (a.angle - (-3.14159265)) / output.angle_increment;
        if (index_prev != index){
            output.ranges[index] = a.range;
        }
        else{ // same index - Then choose closer point
            std::cout << "same index";
            if (output.ranges[index]>a.range){
                output.ranges[index] = a.range;
            }
        }
        
        priQue.pop();
        index_prev = index;
    }

    // Publish laser scan data
    pub.publish(output);
}



int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "leishen_pointcloud_to_laserscan");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/c16_front/lslidar_point_cloud", 1, cloudCb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::LaserScan> ("/cloud_output", 1);

    // Spin
    ros::spin ();
}
