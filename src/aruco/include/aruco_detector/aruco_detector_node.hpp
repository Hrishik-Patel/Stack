#pragma once

#include <string>
#include <vector>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

namespace aruco_detector
{

class ArucoDetectorNode : public rclcpp::Node
{
public:
  explicit ArucoDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ArucoDetectorNode() override = default;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void initDetector();
  void filterValidMarkers(
    const std::vector<int> & detected_ids,
    const std::vector<std::vector<cv::Point2f>> & detected_corners,
    std::vector<int> & valid_ids,
    std::vector<std::vector<cv::Point2f>> & valid_corners) const;

  image_transport::Subscriber image_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ids_pub_;

  std::string image_topic_;
  int         dict_type_;
  int         min_valid_id_;
  int         max_valid_id_;

  cv::aruco::Dictionary      dictionary_;
  cv::aruco::DetectorParameters params_;
  cv::aruco::ArucoDetector   detector_;

  uint64_t frame_count_{0};
};

}  // namespace aruco_detector
