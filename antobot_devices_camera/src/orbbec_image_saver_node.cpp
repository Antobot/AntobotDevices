#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Image = sensor_msgs::msg::Image;

namespace
{

int64_t stamp_to_ns(const Image & msg)
{
  return static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
         static_cast<int64_t>(msg.header.stamp.nanosec);
}

std::string stamp_to_name(const Image & msg)
{
  std::ostringstream ss;
  ss << msg.header.stamp.sec << "_" << std::setw(9) << std::setfill('0')
     << msg.header.stamp.nanosec;
  return ss.str();
}

std::string relpath_string(const fs::path & path, const fs::path & base)
{
  return fs::relative(path, base).generic_string();
}

std::string timestamp_session_name()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  std::ostringstream ss;
  ss << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return ss.str();
}

fs::path default_output_dir()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr || std::string(home).empty()) {
    throw std::runtime_error("HOME is not set; cannot create camera output directory");
  }
  return fs::path(home) / "ac_robot" / "camera_data" / timestamp_session_name();
}

struct SaverOptions
{
  int save_every_n = 1;
  int jpeg_quality = 90;
  bool sync = false;
  double sync_slop_ms = 30.0;
  size_t queue_size = 10;
  bool depth_float_to_mm = false;
};

struct CameraTopics
{
  std::string name;
  std::string color_topic;
  std::string depth_topic;
};

class CameraRecorder
{
public:
  CameraRecorder(
    rclcpp::Node & node,
    const CameraTopics & topics,
    const fs::path & output_root,
    const SaverOptions & options,
    const rclcpp::QoS & qos)
  : node_(node),
    topics_(topics),
    options_(options),
    output_dir_(output_root / topics.name),
    color_dir_(output_dir_ / "color"),
    depth_dir_(output_dir_ / "depth"),
    index_path_(output_dir_ / "index.csv")
  {
    fs::create_directories(color_dir_);
    fs::create_directories(depth_dir_);

    index_file_.open(index_path_, std::ios::out | std::ios::app);
    if (!index_file_.is_open()) {
      throw std::runtime_error("Failed to open index file: " + index_path_.string());
    }

    if (fs::is_empty(index_path_)) {
      index_file_
        << "pair_id,color_file,depth_file,color_stamp_ns,depth_stamp_ns,"
        << "color_encoding,depth_encoding,color_width,color_height,depth_width,depth_height\n";
      index_file_.flush();
    }

    color_sub_ = node_.create_subscription<Image>(
      topics_.color_topic, qos,
      [this](Image::ConstSharedPtr msg) { color_callback(std::move(msg)); });
    depth_sub_ = node_.create_subscription<Image>(
      topics_.depth_topic, qos,
      [this](Image::ConstSharedPtr msg) { depth_callback(std::move(msg)); });

    RCLCPP_INFO(node_.get_logger(), "[%s] Saving to %s", topics_.name.c_str(), output_dir_.c_str());
    RCLCPP_INFO(node_.get_logger(), "[%s] Color topic: %s", topics_.name.c_str(), topics_.color_topic.c_str());
    RCLCPP_INFO(node_.get_logger(), "[%s] Depth topic: %s", topics_.name.c_str(), topics_.depth_topic.c_str());
  }

  void log_stats(double elapsed)
  {
    if (elapsed <= 0.0) {
      return;
    }

    const double entry_rate = static_cast<double>(saved_count_ - last_stat_saved_count_) / elapsed;
    const double color_rate =
      static_cast<double>(color_saved_count_ - last_stat_color_saved_count_) / elapsed;
    const double depth_rate =
      static_cast<double>(depth_saved_count_ - last_stat_depth_saved_count_) / elapsed;

    if (options_.sync) {
      RCLCPP_INFO(
        node_.get_logger(),
        "[%s] Saved rate: %.2f pairs/s, color %.2f images/s, depth %.2f images/s; total pairs %zu",
        topics_.name.c_str(), entry_rate, color_rate, depth_rate, saved_count_);
    } else {
      RCLCPP_INFO(
        node_.get_logger(),
        "[%s] Saved rate: %.2f index rows/s, color %.2f images/s, depth %.2f images/s; total rows %zu",
        topics_.name.c_str(), entry_rate, color_rate, depth_rate, saved_count_);
    }

    last_stat_saved_count_ = saved_count_;
    last_stat_color_saved_count_ = color_saved_count_;
    last_stat_depth_saved_count_ = depth_saved_count_;
  }

  void log_summary(double duration)
  {
    if (options_.sync) {
      RCLCPP_INFO(
        node_.get_logger(),
        "[%s] Recording stopped. Duration: %.2fs, saved pairs: %zu, color images: %zu, depth images: %zu",
        topics_.name.c_str(), duration, saved_count_, color_saved_count_, depth_saved_count_);
    } else {
      RCLCPP_INFO(
        node_.get_logger(),
        "[%s] Recording stopped. Duration: %.2fs, index rows: %zu, color images: %zu, depth images: %zu",
        topics_.name.c_str(), duration, saved_count_, color_saved_count_, depth_saved_count_);
    }
  }

private:
  bool should_save_color()
  {
    ++color_frame_count_;
    return color_frame_count_ % static_cast<size_t>(options_.save_every_n) == 0;
  }

  bool should_save_depth()
  {
    ++depth_frame_count_;
    return depth_frame_count_ % static_cast<size_t>(options_.save_every_n) == 0;
  }

  bool should_save_pair()
  {
    ++pair_frame_count_;
    return pair_frame_count_ % static_cast<size_t>(options_.save_every_n) == 0;
  }

  void color_callback(Image::ConstSharedPtr msg)
  {
    try {
      if (options_.sync) {
        color_queue_.push_back(std::move(msg));
        trim_queues();
        try_save_synced_pair();
        return;
      }

      if (!should_save_color()) {
        return;
      }

      const std::string color_file = save_color(*msg, stamp_to_name(*msg));
      write_index(color_file, "", msg.get(), nullptr, std::nullopt);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node_.get_logger(), "[%s] Color save failed: %s", topics_.name.c_str(), ex.what());
    }
  }

  void depth_callback(Image::ConstSharedPtr msg)
  {
    try {
      if (options_.sync) {
        depth_queue_.push_back(std::move(msg));
        trim_queues();
        try_save_synced_pair();
        return;
      }

      if (!should_save_depth()) {
        return;
      }

      const std::string depth_file = save_depth(*msg, stamp_to_name(*msg));
      write_index("", depth_file, nullptr, msg.get(), std::nullopt);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node_.get_logger(), "[%s] Depth save failed: %s", topics_.name.c_str(), ex.what());
    }
  }

  void trim_queues()
  {
    while (color_queue_.size() > options_.queue_size) {
      color_queue_.pop_front();
    }
    while (depth_queue_.size() > options_.queue_size) {
      depth_queue_.pop_front();
    }
  }

  void try_save_synced_pair()
  {
    if (color_queue_.empty() || depth_queue_.empty()) {
      return;
    }

    const auto color_msg = color_queue_.back();
    const int64_t color_ns = stamp_to_ns(*color_msg);
    const auto best_depth_it = std::min_element(
      depth_queue_.begin(), depth_queue_.end(),
      [color_ns](const auto & lhs, const auto & rhs) {
        return std::llabs(stamp_to_ns(*lhs) - color_ns) < std::llabs(stamp_to_ns(*rhs) - color_ns);
      });

    const auto depth_msg = *best_depth_it;
    const int64_t delta_ns = std::llabs(stamp_to_ns(*depth_msg) - color_ns);
    const int64_t slop_ns = static_cast<int64_t>(options_.sync_slop_ms * 1000000.0);
    if (delta_ns > slop_ns) {
      return;
    }

    if (!should_save_pair()) {
      color_queue_.clear();
      depth_queue_.clear();
      return;
    }

    const std::string pair_id = zero_padded_id(saved_count_);
    const std::string color_file = save_color(*color_msg, pair_id);
    const std::string depth_file = save_depth(*depth_msg, pair_id);
    write_index(color_file, depth_file, color_msg.get(), depth_msg.get(), pair_id);

    color_queue_.clear();
    depth_queue_.clear();
  }

  std::string save_color(const Image & msg, const std::string & name)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &) {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }

    const fs::path path = color_dir_ / (name + ".jpg");
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, options_.jpeg_quality};
    if (!cv::imwrite(path.string(), cv_ptr->image, params)) {
      throw std::runtime_error("Failed to write color image: " + path.string());
    }

    ++color_saved_count_;
    return relpath_string(path, output_dir_);
  }

  std::string save_depth(const Image & msg, const std::string & name)
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
    cv::Mat depth = cv_ptr->image;

    if (depth.depth() == CV_32F || depth.depth() == CV_64F) {
      if (!options_.depth_float_to_mm) {
        throw std::runtime_error(
          "Depth image is float. Set depth_float_to_mm:=true to save 16-bit PNG in millimeters.");
      }

      if (depth.channels() != 1) {
        throw std::runtime_error("Float depth image must be single-channel");
      }
      depth = float_depth_to_mm(depth);
    }

    const fs::path path = depth_dir_ / (name + ".png");
    if (!cv::imwrite(path.string(), depth)) {
      throw std::runtime_error("Failed to write depth image: " + path.string());
    }

    ++depth_saved_count_;
    return relpath_string(path, output_dir_);
  }

  void write_index(
    const std::string & color_file,
    const std::string & depth_file,
    const Image * color_msg,
    const Image * depth_msg,
    const std::optional<std::string> & pair_id)
  {
    index_file_
      << pair_id.value_or(zero_padded_id(saved_count_)) << ','
      << color_file << ','
      << depth_file << ','
      << (color_msg ? std::to_string(stamp_to_ns(*color_msg)) : "") << ','
      << (depth_msg ? std::to_string(stamp_to_ns(*depth_msg)) : "") << ','
      << (color_msg ? color_msg->encoding : "") << ','
      << (depth_msg ? depth_msg->encoding : "") << ','
      << (color_msg ? std::to_string(color_msg->width) : "") << ','
      << (color_msg ? std::to_string(color_msg->height) : "") << ','
      << (depth_msg ? std::to_string(depth_msg->width) : "") << ','
      << (depth_msg ? std::to_string(depth_msg->height) : "") << '\n';
    index_file_.flush();
    ++saved_count_;
  }

  static std::string zero_padded_id(size_t value)
  {
    std::ostringstream ss;
    ss << std::setw(6) << std::setfill('0') << value;
    return ss.str();
  }

  static cv::Mat float_depth_to_mm(const cv::Mat & depth)
  {
    cv::Mat depth_mm(depth.rows, depth.cols, CV_16UC1, cv::Scalar(0));
    const double max_mm = static_cast<double>(std::numeric_limits<uint16_t>::max());

    for (int row = 0; row < depth.rows; ++row) {
      uint16_t * out = depth_mm.ptr<uint16_t>(row);
      if (depth.depth() == CV_32F) {
        const float * in = depth.ptr<float>(row);
        for (int col = 0; col < depth.cols; ++col) {
          const double value_mm = static_cast<double>(in[col]) * 1000.0;
          if (std::isfinite(value_mm) && value_mm > 0.0) {
            out[col] = static_cast<uint16_t>(std::min(value_mm, max_mm));
          }
        }
      } else {
        const double * in = depth.ptr<double>(row);
        for (int col = 0; col < depth.cols; ++col) {
          const double value_mm = in[col] * 1000.0;
          if (std::isfinite(value_mm) && value_mm > 0.0) {
            out[col] = static_cast<uint16_t>(std::min(value_mm, max_mm));
          }
        }
      }
    }

    return depth_mm;
  }

  rclcpp::Node & node_;
  CameraTopics topics_;
  SaverOptions options_;
  fs::path output_dir_;
  fs::path color_dir_;
  fs::path depth_dir_;
  fs::path index_path_;
  std::ofstream index_file_;

  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;

  std::deque<Image::ConstSharedPtr> color_queue_;
  std::deque<Image::ConstSharedPtr> depth_queue_;

  size_t color_frame_count_ = 0;
  size_t depth_frame_count_ = 0;
  size_t pair_frame_count_ = 0;
  size_t saved_count_ = 0;
  size_t color_saved_count_ = 0;
  size_t depth_saved_count_ = 0;
  size_t last_stat_saved_count_ = 0;
  size_t last_stat_color_saved_count_ = 0;
  size_t last_stat_depth_saved_count_ = 0;
};

class MultiOrbbecImageSaver : public rclcpp::Node
{
public:
  MultiOrbbecImageSaver()
  : Node("multi_orbbec_image_saver"),
    start_time_(std::chrono::steady_clock::now()),
    last_stat_time_(start_time_)
  {
    SaverOptions options;
    options.save_every_n = declare_parameter<int>("save_every_n", 1);
    options.jpeg_quality = declare_parameter<int>("jpeg_quality", 90);
    options.sync = declare_parameter<bool>("sync", false);
    options.sync_slop_ms = declare_parameter<double>("sync_slop_ms", 30.0);
    options.queue_size = static_cast<size_t>(declare_parameter<int>("queue_size", 10));
    options.depth_float_to_mm = declare_parameter<bool>("depth_float_to_mm", false);
    const int qos_depth = declare_parameter<int>("qos_depth", 5);
    const int log_interval_sec = declare_parameter<int>("log_interval_sec", 5);

    if (options.save_every_n < 1) {
      throw std::runtime_error("save_every_n must be >= 1");
    }
    if (options.jpeg_quality < 1 || options.jpeg_quality > 100) {
      throw std::runtime_error("jpeg_quality must be between 1 and 100");
    }
    if (options.queue_size < 1) {
      throw std::runtime_error("queue_size must be >= 1");
    }
    if (qos_depth < 1) {
      throw std::runtime_error("qos_depth must be >= 1");
    }
    if (log_interval_sec < 0) {
      throw std::runtime_error("log_interval_sec must be >= 0");
    }

    const fs::path output_root = default_output_dir();
    fs::create_directories(output_root);

    const auto front_topics = CameraTopics{
      "camera_front",
      declare_parameter<std::string>("camera_front.color_topic", "/camera_front/color/image_raw"),
      declare_parameter<std::string>("camera_front.depth_topic", "/camera_front/depth/image_raw")};
    const auto rear_topics = CameraTopics{
      "camera_rear",
      declare_parameter<std::string>("camera_rear.color_topic", "/camera_rear/color/image_raw"),
      declare_parameter<std::string>("camera_rear.depth_topic", "/camera_rear/depth/image_raw")};

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth)))
                 .best_effort()
                 .durability_volatile();

    recorders_.push_back(std::make_unique<CameraRecorder>(*this, front_topics, output_root, options, qos));
    recorders_.push_back(std::make_unique<CameraRecorder>(*this, rear_topics, output_root, options, qos));

    if (log_interval_sec > 0) {
      timer_ = create_wall_timer(
        std::chrono::seconds(log_interval_sec),
        [this]() { log_stats(); });
    }

    RCLCPP_INFO(get_logger(), "Saving %s to %s", options.sync ? "synchronized pairs" : "independent frames",
      output_root.c_str());
  }

  void log_summary()
  {
    const auto now = std::chrono::steady_clock::now();
    const double duration = std::chrono::duration<double>(now - start_time_).count();
    for (auto & recorder : recorders_) {
      recorder->log_summary(duration);
    }
  }

private:
  void log_stats()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_stat_time_).count();
    for (auto & recorder : recorders_) {
      recorder->log_stats(elapsed);
    }
    last_stat_time_ = now;
  }

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_stat_time_;
  std::vector<std::unique_ptr<CameraRecorder>> recorders_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiOrbbecImageSaver>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Unhandled exception: %s", ex.what());
  }
  node->log_summary();
  rclcpp::shutdown();
  return 0;
}
