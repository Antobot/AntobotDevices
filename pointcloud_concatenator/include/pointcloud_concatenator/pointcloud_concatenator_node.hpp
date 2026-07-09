// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace pointcloud_concatenator
{

class PointCloudConcatenatorNode : public rclcpp::Node
{
public:
  explicit PointCloudConcatenatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  enum class CollectorStatus { Idle, Processing, Finished };

  struct Parameters
  {
    std::vector<std::string> input_topics;
    std::string output_frame;
    std::string matching_strategy;
    std::vector<double> lidar_timestamp_offsets;
    std::vector<double> lidar_timestamp_noise_window;
    int maximum_queue_size{5};
    int num_collectors{3};
    double timeout_sec{0.1};
    double tf_timeout_sec{0.2};
    double rosbag_length{0.0};
    bool publish_previous_but_late_pointcloud{false};
  };

  struct CollectorInfo
  {
    double reference_time{0.0};
    double noise_window{0.0};
  };

  struct Collector
  {
    CollectorStatus status{CollectorStatus::Idle};
    CollectorInfo info;
    std::unordered_map<std::string, PointCloud2ConstPtr> topic_to_cloud;
    rclcpp::TimerBase::SharedPtr timer;

    void reset()
    {
      status = CollectorStatus::Idle;
      info = CollectorInfo{};
      topic_to_cloud.clear();
      if (timer && !timer->is_canceled()) {
        timer->cancel();
      }
    }
  };

  struct MatchingParams
  {
    std::string topic_name;
    double cloud_timestamp{0.0};
    double cloud_arrival_time{0.0};
  };

  void declare_and_validate_parameters();
  void initialize_pub_sub();
  void initialize_collectors();

  void cloud_callback(const PointCloud2ConstPtr & cloud, const std::string & topic_name);
  void timeout_callback(const std::shared_ptr<Collector> & collector);

  std::shared_ptr<Collector> select_collector(const MatchingParams & params);
  std::optional<std::shared_ptr<Collector>> match_existing_collector(
    const MatchingParams & params) const;
  std::shared_ptr<Collector> find_idle_or_recycle_oldest_collector();
  void set_collector_info(
    const std::shared_ptr<Collector> & collector, const MatchingParams & params);
  void manage_collectors();

  void publish_collector(const std::shared_ptr<Collector> & collector, const char * reason);
  std::optional<PointCloud2> concatenate_clouds(
    const std::unordered_map<std::string, PointCloud2ConstPtr> & topic_to_cloud);
  std::optional<PointCloud2> transform_to_output_frame(const PointCloud2 & cloud);

  bool has_compatible_layout(const PointCloud2 & reference, const PointCloud2 & candidate) const;
  void append_cloud(PointCloud2 & output, const PointCloud2 & input) const;
  double topic_offset(const std::string & topic_name) const;
  double topic_noise_window(const std::string & topic_name) const;

  Parameters params_;
  std::unordered_map<std::string, double> topic_to_offset_;
  std::unordered_map<std::string, double> topic_to_noise_window_;

  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> pointcloud_subs_;
  rclcpp::Publisher<PointCloud2>::SharedPtr concatenated_cloud_pub_;

  std::list<std::shared_ptr<Collector>> collectors_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;
  double latest_published_stamp_{0.0};
};

}  // namespace pointcloud_concatenator
