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

#include "pointcloud_concatenator/pointcloud_concatenator_node.hpp"

#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pointcloud_concatenator
{

PointCloudConcatenatorNode::PointCloudConcatenatorNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_concatenator", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_and_validate_parameters();
  initialize_pub_sub();
  initialize_collectors();

  RCLCPP_INFO(
    get_logger(), "Standalone point cloud concatenator started with %zu input topics",
    params_.input_topics.size());
}

void PointCloudConcatenatorNode::declare_and_validate_parameters()
{
  const std::vector<std::string> default_input_topics;
  const std::vector<double> default_double_vector;

  params_.input_topics =
    declare_parameter<std::vector<std::string>>("input_topics", default_input_topics);
  params_.output_frame = declare_parameter<std::string>("output_frame", "");
  params_.matching_strategy = declare_parameter<std::string>("matching_strategy.type", "naive");
  params_.lidar_timestamp_offsets =
    declare_parameter<std::vector<double>>(
      "matching_strategy.lidar_timestamp_offsets", default_double_vector);
  params_.lidar_timestamp_noise_window =
    declare_parameter<std::vector<double>>(
      "matching_strategy.lidar_timestamp_noise_window", default_double_vector);
  params_.maximum_queue_size = declare_parameter<int>("maximum_queue_size", 5);
  params_.num_collectors = declare_parameter<int>("num_collectors", 3);
  params_.timeout_sec = declare_parameter<double>("timeout_sec", 0.1);
  params_.tf_timeout_sec = declare_parameter<double>("tf_timeout_sec", 0.2);
  params_.rosbag_length = declare_parameter<double>("rosbag_length", 0.0);
  params_.publish_previous_but_late_pointcloud =
    declare_parameter<bool>("publish_previous_but_late_pointcloud", false);

  if (params_.input_topics.size() < 2) {
    throw std::runtime_error("input_topics must contain at least two point cloud topics.");
  }
  if (params_.output_frame.empty()) {
    throw std::runtime_error("output_frame must be set.");
  }
  if (params_.matching_strategy != "naive" && params_.matching_strategy != "advanced") {
    throw std::runtime_error("matching_strategy.type must be either 'naive' or 'advanced'.");
  }
  if (params_.maximum_queue_size <= 0) {
    throw std::runtime_error("maximum_queue_size must be positive.");
  }
  if (params_.num_collectors <= 0) {
    throw std::runtime_error("num_collectors must be positive.");
  }
  if (params_.timeout_sec <= 0.0) {
    throw std::runtime_error("timeout_sec must be positive.");
  }
  if (params_.tf_timeout_sec < 0.0) {
    throw std::runtime_error("tf_timeout_sec must not be negative.");
  }

  if (params_.matching_strategy == "advanced") {
    if (params_.lidar_timestamp_offsets.size() != params_.input_topics.size()) {
      throw std::runtime_error(
        "matching_strategy.lidar_timestamp_offsets must have the same length as input_topics.");
    }
    if (params_.lidar_timestamp_noise_window.size() != params_.input_topics.size()) {
      throw std::runtime_error(
        "matching_strategy.lidar_timestamp_noise_window must have the same length as "
        "input_topics.");
    }
  } else {
    params_.lidar_timestamp_offsets.assign(params_.input_topics.size(), 0.0);
    params_.lidar_timestamp_noise_window.assign(params_.input_topics.size(), 0.0);
  }

  for (size_t i = 0; i < params_.input_topics.size(); ++i) {
    topic_to_offset_[params_.input_topics[i]] = params_.lidar_timestamp_offsets[i];
    topic_to_noise_window_[params_.input_topics[i]] = params_.lidar_timestamp_noise_window[i];
  }
}

void PointCloudConcatenatorNode::initialize_pub_sub()
{
  const auto qos = rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size);
  concatenated_cloud_pub_ = create_publisher<PointCloud2>("/pointcloud/concatenated", qos);

  for (const auto & topic : params_.input_topics) {
    auto callback = [this, topic](const PointCloud2ConstPtr msg) {
      cloud_callback(msg, topic);
    };
    pointcloud_subs_.push_back(create_subscription<PointCloud2>(topic, qos, callback));
    RCLCPP_INFO(get_logger(), "Subscribing to input topic: %s", topic.c_str());
  }
}

void PointCloudConcatenatorNode::initialize_collectors()
{
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(params_.timeout_sec));

  for (int i = 0; i < params_.num_collectors; ++i) {
    auto collector = std::make_shared<Collector>();
    collector->timer = rclcpp::create_timer(this, get_clock(), period_ns, [this, collector]() {
      timeout_callback(collector);
    });
    collector->timer->cancel();
    collectors_.push_back(std::move(collector));
  }
}

void PointCloudConcatenatorNode::cloud_callback(
  const PointCloud2ConstPtr & cloud, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  manage_collectors();

  MatchingParams params;
  params.topic_name = topic_name;
  params.cloud_timestamp = rclcpp::Time(cloud->header.stamp).seconds();
  params.cloud_arrival_time = now().seconds();

  auto collector = select_collector(params);
  if (!collector) {
    RCLCPP_ERROR(get_logger(), "No collector is available for topic '%s'.", topic_name.c_str());
    return;
  }

  if (collector->status == CollectorStatus::Idle) {
    collector->status = CollectorStatus::Processing;
    collector->timer->reset();
  }

  if (collector->topic_to_cloud.count(topic_name) > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Collector already contains topic '%s'; replacing the previous cloud.", topic_name.c_str());
  }

  collector->topic_to_cloud[topic_name] = cloud;
  if (collector->topic_to_cloud.size() == params_.input_topics.size()) {
    publish_collector(collector, "all topics arrived");
  }
}

void PointCloudConcatenatorNode::timeout_callback(const std::shared_ptr<Collector> & collector)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (collector->status != CollectorStatus::Processing) {
    return;
  }
  publish_collector(collector, "timeout");
}

std::shared_ptr<PointCloudConcatenatorNode::Collector>
PointCloudConcatenatorNode::select_collector(const MatchingParams & params)
{
  auto matched = match_existing_collector(params);
  if (matched) {
    return matched.value();
  }

  auto collector = find_idle_or_recycle_oldest_collector();
  if (collector) {
    set_collector_info(collector, params);
  }
  return collector;
}

std::optional<std::shared_ptr<PointCloudConcatenatorNode::Collector>>
PointCloudConcatenatorNode::match_existing_collector(const MatchingParams & params) const
{
  if (params_.matching_strategy == "naive") {
    std::optional<double> smallest_time_difference;
    std::shared_ptr<Collector> closest_collector;

    for (const auto & collector : collectors_) {
      if (
        collector->status != CollectorStatus::Processing ||
        collector->topic_to_cloud.count(params.topic_name) > 0) {
        continue;
      }

      const double time_difference =
        std::abs(params.cloud_arrival_time - collector->info.reference_time);
      if (!smallest_time_difference || time_difference < smallest_time_difference.value()) {
        smallest_time_difference = time_difference;
        closest_collector = collector;
      }
    }

    if (closest_collector) {
      return closest_collector;
    }
    return std::nullopt;
  }

  const double corrected_cloud_time = params.cloud_timestamp - topic_offset(params.topic_name);
  const double cloud_noise_window = topic_noise_window(params.topic_name);

  for (const auto & collector : collectors_) {
    if (
      collector->status != CollectorStatus::Processing ||
      collector->topic_to_cloud.count(params.topic_name) > 0) {
      continue;
    }

    const double collector_min = collector->info.reference_time - collector->info.noise_window;
    const double collector_max = collector->info.reference_time + collector->info.noise_window;
    if (
      corrected_cloud_time > collector_min - cloud_noise_window &&
      corrected_cloud_time < collector_max + cloud_noise_window) {
      return collector;
    }
  }

  return std::nullopt;
}

std::shared_ptr<PointCloudConcatenatorNode::Collector>
PointCloudConcatenatorNode::find_idle_or_recycle_oldest_collector()
{
  for (const auto & collector : collectors_) {
    if (collector->status == CollectorStatus::Idle) {
      return collector;
    }
  }

  auto oldest_it = collectors_.end();
  double oldest_reference_time = std::numeric_limits<double>::max();
  for (auto it = collectors_.begin(); it != collectors_.end(); ++it) {
    if ((*it)->status != CollectorStatus::Processing) {
      continue;
    }
    if ((*it)->info.reference_time < oldest_reference_time) {
      oldest_reference_time = (*it)->info.reference_time;
      oldest_it = it;
    }
  }

  if (oldest_it == collectors_.end()) {
    return nullptr;
  }

  RCLCPP_WARN(
    get_logger(),
    "All collectors are busy. Resetting the oldest in-flight collector before it timed out.");
  (*oldest_it)->reset();
  return *oldest_it;
}

void PointCloudConcatenatorNode::set_collector_info(
  const std::shared_ptr<Collector> & collector, const MatchingParams & params)
{
  if (params_.matching_strategy == "naive") {
    collector->info.reference_time = params.cloud_arrival_time;
    collector->info.noise_window = 0.0;
    return;
  }

  collector->info.reference_time = params.cloud_timestamp - topic_offset(params.topic_name);
  collector->info.noise_window = topic_noise_window(params.topic_name);
}

void PointCloudConcatenatorNode::manage_collectors()
{
  for (const auto & collector : collectors_) {
    if (collector->status == CollectorStatus::Finished) {
      collector->reset();
    }
  }
}

void PointCloudConcatenatorNode::publish_collector(
  const std::shared_ptr<Collector> & collector, const char * reason)
{
  if (collector->timer && !collector->timer->is_canceled()) {
    collector->timer->cancel();
  }

  auto concatenated_cloud = concatenate_clouds(collector->topic_to_cloud);
  collector->status = CollectorStatus::Finished;

  if (!concatenated_cloud) {
    RCLCPP_WARN(
      get_logger(), "Skipping publish after %s: no valid cloud could be concatenated.", reason);
    return;
  }

  const double current_stamp = rclcpp::Time(concatenated_cloud->header.stamp).seconds();
  if (
    current_stamp < latest_published_stamp_ && !params_.publish_previous_but_late_pointcloud &&
    !(params_.rosbag_length > 0.0 &&
      latest_published_stamp_ - current_stamp > params_.rosbag_length)) {
    RCLCPP_WARN(
      get_logger(),
      "Dropping concatenated cloud after %s because its stamp is older than latest output.",
      reason);
    return;
  }

  latest_published_stamp_ = current_stamp;
  const auto point_count = concatenated_cloud->width;
  concatenated_cloud_pub_->publish(concatenated_cloud.value());
  RCLCPP_DEBUG(
    get_logger(), "Published concatenated cloud after %s with %u points.", reason,
    point_count);
}

std::optional<PointCloudConcatenatorNode::PointCloud2>
PointCloudConcatenatorNode::concatenate_clouds(
  const std::unordered_map<std::string, PointCloud2ConstPtr> & topic_to_cloud)
{
  if (topic_to_cloud.empty()) {
    return std::nullopt;
  }

  std::vector<rclcpp::Time> stamps;
  stamps.reserve(topic_to_cloud.size());
  for (const auto & [topic, cloud] : topic_to_cloud) {
    (void)topic;
    stamps.emplace_back(cloud->header.stamp);
  }
  std::sort(stamps.begin(), stamps.end(), std::greater<rclcpp::Time>());
  const auto oldest_stamp = stamps.back();

  std::optional<PointCloud2> output_cloud;
  std::vector<std::string> missing_topics;
  std::vector<std::string> skipped_topics;

  for (const auto & topic : params_.input_topics) {
    const auto cloud_it = topic_to_cloud.find(topic);
    if (cloud_it == topic_to_cloud.end()) {
      missing_topics.push_back(topic);
      continue;
    }

    auto transformed_cloud = transform_to_output_frame(*cloud_it->second);
    if (!transformed_cloud) {
      skipped_topics.push_back(topic);
      continue;
    }

    if (!output_cloud) {
      output_cloud = PointCloud2{};
      output_cloud->header.frame_id = params_.output_frame;
      output_cloud->header.stamp = oldest_stamp;
      output_cloud->fields = transformed_cloud->fields;
      output_cloud->is_bigendian = transformed_cloud->is_bigendian;
      output_cloud->point_step = transformed_cloud->point_step;
      output_cloud->height = 1;
      output_cloud->width = 0;
      output_cloud->row_step = 0;
      output_cloud->is_dense = transformed_cloud->is_dense;
    } else if (!has_compatible_layout(output_cloud.value(), transformed_cloud.value())) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping topic '%s' because its PointCloud2 fields do not match the first valid cloud.",
        topic.c_str());
      skipped_topics.push_back(topic);
      continue;
    }

    append_cloud(output_cloud.value(), transformed_cloud.value());
    output_cloud->is_dense = output_cloud->is_dense && transformed_cloud->is_dense;
  }

  if (!missing_topics.empty()) {
    RCLCPP_WARN(
      get_logger(), "Concatenating without %zu missing topic(s) because the collector timed out.",
      missing_topics.size());
  }
  if (!skipped_topics.empty()) {
    RCLCPP_WARN(get_logger(), "Skipped %zu topic(s) during concatenation.", skipped_topics.size());
  }

  if (!output_cloud || output_cloud->width == 0) {
    return std::nullopt;
  }
  return output_cloud;
}

std::optional<PointCloudConcatenatorNode::PointCloud2>
PointCloudConcatenatorNode::transform_to_output_frame(const PointCloud2 & cloud)
{
  if (cloud.header.frame_id == params_.output_frame) {
    PointCloud2 output = cloud;
    output.header.frame_id = params_.output_frame;
    return output;
  }

  try {
    const auto transform = tf_buffer_.lookupTransform(
      params_.output_frame, cloud.header.frame_id, cloud.header.stamp,
      rclcpp::Duration::from_seconds(params_.tf_timeout_sec));
    PointCloud2 output;
    tf2::doTransform(cloud, output, transform);
    output.header.frame_id = params_.output_frame;
    return output;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(), "Failed to transform cloud from '%s' to '%s': %s",
      cloud.header.frame_id.c_str(), params_.output_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

bool PointCloudConcatenatorNode::has_compatible_layout(
  const PointCloud2 & reference, const PointCloud2 & candidate) const
{
  if (
    reference.fields.size() != candidate.fields.size() ||
    reference.point_step != candidate.point_step ||
    reference.is_bigendian != candidate.is_bigendian) {
    return false;
  }

  for (size_t i = 0; i < reference.fields.size(); ++i) {
    const auto & lhs = reference.fields[i];
    const auto & rhs = candidate.fields[i];
    if (
      lhs.name != rhs.name || lhs.offset != rhs.offset || lhs.datatype != rhs.datatype ||
      lhs.count != rhs.count) {
      return false;
    }
  }
  return true;
}

void PointCloudConcatenatorNode::append_cloud(PointCloud2 & output, const PointCloud2 & input) const
{
  const uint32_t row_payload_size = input.width * input.point_step;
  const uint32_t point_count = input.width * input.height;

  if (input.row_step == row_payload_size) {
    output.data.insert(output.data.end(), input.data.begin(), input.data.end());
  } else {
    for (uint32_t row = 0; row < input.height; ++row) {
      const auto row_begin = input.data.begin() + row * input.row_step;
      output.data.insert(output.data.end(), row_begin, row_begin + row_payload_size);
    }
  }

  output.width += point_count;
  output.height = 1;
  output.row_step = static_cast<uint32_t>(output.data.size());
}

double PointCloudConcatenatorNode::topic_offset(const std::string & topic_name) const
{
  const auto it = topic_to_offset_.find(topic_name);
  return it == topic_to_offset_.end() ? 0.0 : it->second;
}

double PointCloudConcatenatorNode::topic_noise_window(const std::string & topic_name) const
{
  const auto it = topic_to_noise_window_.find(topic_name);
  return it == topic_to_noise_window_.end() ? 0.0 : it->second;
}

}  // namespace pointcloud_concatenator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_concatenator::PointCloudConcatenatorNode>());
  rclcpp::shutdown();
  return 0;
}
