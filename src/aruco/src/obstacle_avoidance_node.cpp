#include <cmath>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
  ObstacleAvoidanceNode()
  : Node("obstacle_avoidance"),
    obstacle_active_(false),
    aruco_active_(false),
    obs_x_(0.0f), obs_y_(0.0f)
  {
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/local_grid_safe", 10,
      std::bind(&ObstacleAvoidanceNode::pclCallback, this, std::placeholders::_1));

    // Int8: 0 = no detection, >0 = detected marker ID
    aruco_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/aruco_detected", 10,
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        aruco_active_ = (msg->data > 0);
      });

    cmd_vel_pub_      = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    obs_active_pub_   = this->create_publisher<std_msgs::msg::Bool>("/obstacle_active", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ObstacleAvoidanceNode::loop, this));

    RCLCPP_INFO(this->get_logger(), "ObstacleAvoidanceNode ready.");
  }

private:

  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    constexpr float min_x  = 0.5f;
    constexpr float max_x  = 3.0f;
    constexpr float half_w = 0.40f;

    bool found = false;
    float best_x = std::numeric_limits<float>::max();
    float x = 0.0f, y = 0.0f;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");

    for (; it_x != it_x.end(); ++it_x, ++it_y) {
      const float px = *it_x, py = *it_y;
      if (px < min_x || px > max_x)  continue;
      if (std::abs(py) > half_w)      continue;
      if (px < best_x) { best_x = px; x = px; y = py; found = true; }
    }

    obstacle_active_ = found;
    obs_x_ = x;
    obs_y_ = y;
  }

  void loop()
  {
    // Always publish obstacle state so search + follower can go silent
    std_msgs::msg::Bool obs_msg;
    obs_msg.data = obstacle_active_;
    obs_active_pub_->publish(obs_msg);

    if (!obstacle_active_)
      return;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;

    // ArUco visible → spin LEFT (toward where marker likely is)
    // No ArUco     → spin RIGHT (default)
    cmd.angular.z = aruco_active_ ? +1.0 : -1.0;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "[OA] Obstacle x=%.2f y=%.2f | ArUco=%s → spin %s",
      obs_x_, obs_y_,
      aruco_active_ ? "YES" : "NO",
      aruco_active_ ? "LEFT" : "RIGHT");

    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr            aruco_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               obs_active_pub_;
  rclcpp::TimerBase::SharedPtr                                    timer_;

  bool  obstacle_active_;
  bool  aruco_active_;
  float obs_x_, obs_y_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleAvoidanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
