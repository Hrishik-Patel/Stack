#include <cmath>
#include <limits>
#include <algorithm>
#include <mutex>
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
    obs_x_(0.0f), obs_y_(0.0f),
    clear_count_(0),
    oa_state_(OAState::IDLE),
    oa_step_count_(0),
    oa_turn_dir_(0.0f)
  {
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/local_grid_obstacle", 10,
      std::bind(&ObstacleAvoidanceNode::pclCallback, this, std::placeholders::_1));

    aruco_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/aruco_detected", 10,
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        aruco_active_ = (msg->data > 0);
      });

    cmd_vel_pub_    = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    obs_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_active", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ObstacleAvoidanceNode::loop, this));

    RCLCPP_INFO(this->get_logger(), "ObstacleAvoidanceNode ready.");
  }

private:
  // ── Detection constants ──────────────────────────────────────────────
  static constexpr float MIN_X        = 0.8f;
  static constexpr float MAX_X        = 6.0f;  // ← updated
  static constexpr float HALF_W       = 0.7f;  // ← updated
  static constexpr float SIDE_BAND    = 0.35f;
  static constexpr int   CLEAR_NEEDED = 6;

  // ── OA state machine constants ───────────────────────────────────────
  // MAX_X=6m needs longer move to guarantee clearance
  static constexpr int TURN_STEPS = 25;   // 25 × 50ms = 1.25 sec turn
  static constexpr int MOVE_STEPS = 60;   // 60 × 50ms = 3.0 sec × 0.4m/s = 1.2m forward

  enum class OAState { IDLE, TURNING, MOVING_CLEAR };

  // ── Point cloud callback ─────────────────────────────────────────────
  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    bool  found  = false;
    float best_x = std::numeric_limits<float>::max();
    float cx = 0.0f, cy = 0.0f;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");

    for (; it_x != it_x.end(); ++it_x, ++it_y) {
      const float px = *it_x, py = *it_y;
      if (!std::isfinite(px) || !std::isfinite(py)) continue;
      if (px < MIN_X || px > MAX_X)  continue;
      if (std::abs(py) > HALF_W)     continue;
      if (px < best_x) { best_x = px; cx = px; cy = py; found = true; }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (found) {
      obstacle_active_ = true;
      clear_count_     = 0;
      obs_x_ = cx;
      obs_y_ = cy;
    } else {
      if (obstacle_active_) {
        if (++clear_count_ >= CLEAR_NEEDED) {
          obstacle_active_ = false;
          clear_count_     = 0;
          obs_x_ = obs_y_ = 0.0f;
        }
      }
    }
  }

  // ── Control loop ─────────────────────────────────────────────────────
  void loop()
  {
    bool  active, aruco;
    float ox, oy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active = obstacle_active_;
      ox     = obs_x_;
      oy     = obs_y_;
      aruco  = aruco_active_;
    }

    // true only while OA state machine is active
    std_msgs::msg::Bool obs_msg;
    obs_msg.data = (oa_state_ != OAState::IDLE);
    obs_active_pub_->publish(obs_msg);

    geometry_msgs::msg::Twist cmd;   // default zero

    switch (oa_state_)
    {
      // ── IDLE: wait for obstacle, search node has control ─────────────
      case OAState::IDLE:
        if (active) {
          // obstacle RIGHT (oy < 0) → turn LEFT  (+z)
          // obstacle LEFT  (oy > 0) → turn RIGHT (-z)
          if      (oy < -SIDE_BAND) oa_turn_dir_ = +1.0f;
          else if (oy >  SIDE_BAND) oa_turn_dir_ = -1.0f;
          else                      oa_turn_dir_ = aruco ? +1.0f : -1.0f;

          oa_step_count_ = 0;
          oa_state_      = OAState::TURNING;

          RCLCPP_WARN(this->get_logger(),
            "[OA] Obstacle detected x=%.2f y=%.2f → turning %s",
            ox, oy, oa_turn_dir_ > 0 ? "LEFT" : "RIGHT");
        }
        return;

      // ── TURNING: rotate away from obstacle ──────────────────────────
      case OAState::TURNING:
        cmd.linear.x  = 0.0;
        cmd.angular.z = oa_turn_dir_ * 1.0;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "[OA][TURN] x=%.2f y=%.2f | dir=%s | step=%d/%d",
          ox, oy, oa_turn_dir_ > 0 ? "LEFT" : "RIGHT",
          oa_step_count_, TURN_STEPS);

        if (++oa_step_count_ >= TURN_STEPS) {
          oa_step_count_ = 0;
          oa_state_      = OAState::MOVING_CLEAR;
          RCLCPP_INFO(this->get_logger(), "[OA] Turn done → moving forward to clear");
        }
        break;

      // ── MOVING_CLEAR: drive forward past the obstacle ────────────────
      case OAState::MOVING_CLEAR:
        cmd.linear.x  = 0.4;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "[OA][MOVE] step=%d/%d", oa_step_count_, MOVE_STEPS);

        if (++oa_step_count_ >= MOVE_STEPS) {
          oa_step_count_ = 0;
          oa_state_      = OAState::IDLE;

          std::lock_guard<std::mutex> lock(mutex_);
          obstacle_active_ = false;
          clear_count_     = 0;
          obs_x_ = obs_y_ = 0.0f;

          RCLCPP_INFO(this->get_logger(), "[OA] Obstacle cleared → resuming search");
        } else {
          // new obstacle while moving → immediately re-turn
          std::lock_guard<std::mutex> lock(mutex_);
          if (obstacle_active_) {
            oa_turn_dir_   = (obs_y_ < 0.0f) ? +1.0f : -1.0f;
            oa_step_count_ = 0;
            oa_state_      = OAState::TURNING;
            RCLCPP_WARN(this->get_logger(),
              "[OA] New obstacle while clearing x=%.2f y=%.2f → re-turning %s",
              obs_x_, obs_y_, oa_turn_dir_ > 0 ? "LEFT" : "RIGHT");
          }
        }
        break;
    }
  }

  // ── Members ──────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr            aruco_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               obs_active_pub_;
  rclcpp::TimerBase::SharedPtr                                    timer_;

  std::mutex mutex_;
  bool  obstacle_active_;
  bool  aruco_active_;
  float obs_x_, obs_y_;
  int   clear_count_;

  OAState oa_state_;
  int     oa_step_count_;
  float   oa_turn_dir_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
  rclcpp::shutdown();
}
