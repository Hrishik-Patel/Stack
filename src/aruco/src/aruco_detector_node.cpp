#pragma once

#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

namespace aruco_detector
{

class ArucoDetectorNode : public rclcpp::Node
{
public:
  explicit ArucoDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("aruco_detector", options),
    FollowPattern(kMoveForward),
    search_ref_set_(false),
    spot_turn_back_(false),
    spot_done_(false),
    search_cycle_(0),
    search_end_time_(this->now()),
    search_forward_time_(4.0),
    search_skew(kNoSkew),
    aruco_detect_(false)
  {
    this->declare_parameter<std::string>("image_topic", "/zed/zed_node/rgb/color/rect/image");
    this->declare_parameter<int>("dictionary_type", cv::aruco::DICT_5X5_100);
    this->declare_parameter<int>("min_valid_id", 51);
    this->declare_parameter<int>("max_valid_id", 65);
    this->declare_parameter<bool>("spot_turn_back", false);
    this->declare_parameter<double>("search_forward_time", 4.0);
    this->declare_parameter<int>("search_skew", 0);

    this->get_parameter("image_topic",         image_topic_);
    this->get_parameter("dictionary_type",     dict_type_);
    this->get_parameter("min_valid_id",        min_valid_id_);
    this->get_parameter("max_valid_id",        max_valid_id_);
    this->get_parameter("spot_turn_back",      spot_turn_back_);
    this->get_parameter("search_forward_time", search_forward_time_);

    int skew_param = 0;
    this->get_parameter("search_skew", skew_param);
    search_skew = (skew_param == 1) ? kRightSkew : (skew_param == -1) ? kLeftSkew : kNoSkew;

    if (min_valid_id_ > max_valid_id_) {
      RCLCPP_FATAL(this->get_logger(),
        "min_valid_id (%d) must be <= max_valid_id (%d).", min_valid_id_, max_valid_id_);
      throw std::invalid_argument("min_valid_id > max_valid_id");
    }

    RCLCPP_INFO(this->get_logger(),
      "Parameters: image_topic=%s  valid_id_range=[%d, %d]  "
      "spot_turn_back=%s  search_forward_time=%.1f  search_skew=%d",
      image_topic_.c_str(), min_valid_id_, max_valid_id_,
      spot_turn_back_ ? "true" : "false", search_forward_time_, skew_param);

    if (spot_turn_back_) {
      search_end_time_ = this->now() + rclcpp::Duration::from_seconds(6.5);
    }

    initDetector();

    const auto reliable_qos = rclcpp::QoS(10).reliable();

    detected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/aruco_detected", reliable_qos);

    ids_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/detected_aruco_ids", reliable_qos);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    image_sub_ = image_transport::create_subscription(
      this,
      image_topic_,
      std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1),
      "raw",
      rclcpp::SensorDataQoS().get_rmw_qos_profile());

    search_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ArucoDetectorNode::searchLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "ArucoDetectorNode ready. Subscribing to: %s", image_topic_.c_str());
  }

  ~ArucoDetectorNode() override = default;

private:

  enum SearchPattern { kMoveForward, kTurnA, kTurnB, kTurnC };
  enum SearchSkew    { kNoSkew = 0, kLeftSkew = -1, kRightSkew = 1 };

  void initDetector()
  {
    dictionary_ = cv::aruco::getPredefinedDictionary(
      static_cast<cv::aruco::PredefinedDictionaryType>(dict_type_));

    params_ = cv::aruco::DetectorParameters();
    params_.cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

    detector_ = cv::aruco::ArucoDetector(dictionary_, params_);

    RCLCPP_INFO(this->get_logger(), "ArUco detector initialised (CORNER_REFINE_NONE).");
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Received empty or zero-dimension image – skipping.");
      return;
    }

    cv::Mat frame;
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::cvtColor(cv_ptr->image, frame, cv::COLOR_BGR2GRAY);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "cv_bridge conversion failed: %s", e.what());
      return;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Unexpected exception in cv_bridge: %s", e.what());
      return;
    }

    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "cv_bridge returned an empty Mat – skipping.");
      return;
    }

    std::vector<int> detected_ids;
    std::vector<std::vector<cv::Point2f>> detected_corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    try {
      detector_.detectMarkers(frame, detected_corners, detected_ids, rejected);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "OpenCV ArUco threw: %s", e.what());
      return;
    }

    std::vector<int> valid_ids;
    std::vector<std::vector<cv::Point2f>> valid_corners;
    filterValidMarkers(detected_ids, detected_corners, valid_ids, valid_corners);

    aruco_detect_ = !valid_ids.empty();

    std_msgs::msg::Bool detected_msg;
    detected_msg.data = aruco_detect_;
    detected_pub_->publish(detected_msg);

    std_msgs::msg::Int32MultiArray ids_msg;
    ids_msg.data.assign(valid_ids.begin(), valid_ids.end());
    ids_pub_->publish(ids_msg);

    ++frame_count_;
    if (frame_count_ % 100 == 0) {
      RCLCPP_DEBUG(this->get_logger(),
        "Frame #%lu | raw: %zu | valid: %zu",
        frame_count_, detected_ids.size(), valid_ids.size());
    }
  }

  void filterValidMarkers(
    const std::vector<int> & detected_ids,
    const std::vector<std::vector<cv::Point2f>> & detected_corners,
    std::vector<int> & valid_ids,
    std::vector<std::vector<cv::Point2f>> & valid_corners) const
  {
    valid_ids.clear();
    valid_corners.clear();

    std::set<int> seen_ids;

    for (std::size_t i = 0; i < detected_ids.size(); ++i) {
      const int id = detected_ids[i];

      if (id < min_valid_id_ || id > max_valid_id_) continue;
      if (seen_ids.count(id) > 0) continue;

      seen_ids.insert(id);
      valid_ids.push_back(id);
      valid_corners.push_back(detected_corners[i]);
    }
  }

  void searchLoop()
  {
    if (aruco_detect_) {
      publishVel(geometry_msgs::msg::Twist());
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[ARUCO] Detected — rover stopped.");
      return;
    }
    callSearchPattern();
  }

  void callSearchPattern()
  {
    geometry_msgs::msg::Twist cmd;
    auto clock = this->get_clock();
    auto now   = clock->now();

    const double ang_vel = 1.0;
    const double lin_vel = 0.65;

    if (spot_turn_back_ && !spot_done_)
    {
      cmd.angular.z = ang_vel;
      publishVel(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000,
        "[SEARCH][SPOT] Turning in place | ang=%.2f", cmd.angular.z);
      if (now >= search_end_time_)
      {
        spot_done_       = true;
        search_ref_set_  = false;
        RCLCPP_INFO(get_logger(), "[SEARCH][SPOT] turn done → start pattern");
      }
      return;
    }

    if (!search_ref_set_)
    {
      FollowPattern    = kMoveForward;
      search_end_time_ = now + rclcpp::Duration::from_seconds(search_forward_time_);
      search_ref_set_  = true;
      RCLCPP_INFO(get_logger(), "[SEARCH] Starting search pattern, moving forward");
      cmd.linear.x = lin_vel;
      publishVel(cmd);
      return;
    }

    if (FollowPattern == kTurnA)
    {
      const bool right_skew = (search_skew == kRightSkew);
      cmd.angular.z = right_skew ? -ang_vel : +ang_vel;
      publishVel(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000,
        "[SEARCH][TURN A] ang=%.2f skew=%d cycle=%d",
        cmd.angular.z, search_skew, search_cycle_);
      if (now >= search_end_time_)
      {
        FollowPattern    = kTurnB;
        search_end_time_ = now + rclcpp::Duration::from_seconds(7.0);
      }
      return;
    }

    if (FollowPattern == kTurnB)
    {
      const bool right_skew = (search_skew == kRightSkew);
      cmd.angular.z = right_skew ? +ang_vel : -ang_vel;
      publishVel(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000,
        "[SEARCH][TURN B] ang=%.2f skew=%d cycle=%d",
        cmd.angular.z, search_skew, search_cycle_);
      if (now >= search_end_time_)
      {
        FollowPattern    = kTurnC;
        double extra     = (search_skew != kNoSkew) ? search_cycle_ : 0.0;
        search_end_time_ = now + rclcpp::Duration::from_seconds(3.5 + extra);
      }
      return;
    }

    if (FollowPattern == kTurnC)
    {
      const bool right_skew = (search_skew == kRightSkew);
      cmd.angular.z = right_skew ? -ang_vel : +ang_vel;
      publishVel(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000,
        "[SEARCH][TURN C] ang=%.2f skew=%d cycle=%d",
        cmd.angular.z, search_skew, search_cycle_);
      if (now >= search_end_time_)
      {
        FollowPattern    = kMoveForward;
        search_end_time_ = now + rclcpp::Duration::from_seconds(search_forward_time_);
      }
      return;
    }

    if (FollowPattern == kMoveForward)
    {
      cmd.linear.x = lin_vel;
      publishVel(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000,
        "[SEARCH][FORWARD] lin=%.2f cycle=%d skew=%d",
        cmd.linear.x, search_cycle_, search_skew);
      if (now >= search_end_time_)
      {
        search_cycle_++;
        FollowPattern    = kTurnA;
        search_end_time_ = now + rclcpp::Duration::from_seconds(3.5);
      }
      return;
    }
  }

  void publishVel(const geometry_msgs::msg::Twist & msg)
  {
    geometry_msgs::msg::Twist cmd = msg;
    cmd.linear.x  = std::clamp(cmd.linear.x,  0.0, 2.0);
    cmd.angular.z = std::clamp(cmd.angular.z, -1.5, 1.5);
    cmd_vel_pub_->publish(cmd);
  }

  // ROS interfaces
  image_transport::Subscriber                                  image_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ids_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr                                 search_timer_;

  // Parameters
  std::string image_topic_;
  int         dict_type_;
  int         min_valid_id_;
  int         max_valid_id_;

  // ArUco
  cv::aruco::Dictionary         dictionary_;
  cv::aruco::DetectorParameters params_;
  cv::aruco::ArucoDetector      detector_;
  uint64_t                      frame_count_{0};
  bool                          aruco_detect_;

  // Search pattern
  SearchPattern  FollowPattern;
  SearchSkew     search_skew;
  bool           search_ref_set_;
  bool           spot_turn_back_;
  bool           spot_done_;
  int            search_cycle_;
  rclcpp::Time   search_end_time_;
  double         search_forward_time_;
};

}  // namespace aruco_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_detector::ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
