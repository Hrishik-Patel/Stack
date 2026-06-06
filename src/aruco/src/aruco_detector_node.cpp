#include "aruco_detector/aruco_detector_node.hpp"

#include <algorithm>
#include <stdexcept>

namespace aruco_detector
{

ArucoDetectorNode::ArucoDetectorNode(const rclcpp::NodeOptions & options)
: Node("aruco_detector", options)
{
  this->declare_parameter<std::string>("image_topic", "/zed/zed_node/rgb/color/rect/image");
  this->declare_parameter<int>("dictionary_type", cv::aruco::DICT_5X5_100);
  this->declare_parameter<int>("min_valid_id", 51);
  this->declare_parameter<int>("max_valid_id", 65);

  this->get_parameter("image_topic",    image_topic_);
  this->get_parameter("dictionary_type", dict_type_);
  this->get_parameter("min_valid_id",    min_valid_id_);
  this->get_parameter("max_valid_id",    max_valid_id_);

  if (min_valid_id_ > max_valid_id_) {
    RCLCPP_FATAL(this->get_logger(),
      "min_valid_id (%d) must be <= max_valid_id (%d).", min_valid_id_, max_valid_id_);
    throw std::invalid_argument("min_valid_id > max_valid_id");
  }

  RCLCPP_INFO(this->get_logger(),
    "Parameters: image_topic=%s  valid_id_range=[%d, %d]",
    image_topic_.c_str(), min_valid_id_, max_valid_id_);

  initDetector();

  const auto reliable_qos = rclcpp::QoS(10).reliable();

  detected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/aruco_detected", reliable_qos);

  ids_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
    "/detected_aruco_ids", reliable_qos);

  image_sub_ = image_transport::create_subscription(
    this,
    image_topic_,
    std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1),
    "raw",
    rclcpp::SensorDataQoS().get_rmw_qos_profile());

  RCLCPP_INFO(this->get_logger(),
    "ArucoDetectorNode ready. Subscribing to: %s", image_topic_.c_str());
}

void ArucoDetectorNode::initDetector()
{
  dictionary_ = cv::aruco::getPredefinedDictionary(
    static_cast<cv::aruco::PredefinedDictionaryType>(dict_type_));

  params_ = cv::aruco::DetectorParameters();
  params_.cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

  detector_ = cv::aruco::ArucoDetector(dictionary_, params_);

  RCLCPP_INFO(this->get_logger(), "ArUco detector initialised (CORNER_REFINE_NONE).");
}

void ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Received empty or zero-dimension image – skipping.");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat frame;
  try {
    // Safely copies and converts ZED's bgra8 format into 1-channel grayscale
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::bgr8);
    frame = cv_ptr->image;
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

  std_msgs::msg::Bool detected_msg;
  detected_msg.data = !(valid_ids.empty());
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

void ArucoDetectorNode::filterValidMarkers(
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

} // namespace aruco_detector

// STANDALONE EXECUTABLE ENTRY POINT
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_detector::ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
