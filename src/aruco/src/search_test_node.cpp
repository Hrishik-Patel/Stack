#include <string>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"

class SearchTestNode : public rclcpp::Node
{
public:
  SearchTestNode()
  : Node("search_test"),
    FollowPattern(kMoveForward),
    search_ref_set_(false),
    spot_turn_back_(false),
    spot_done_(false),
    search_cycle_(0),
    search_end_time_(this->now()),
    search_forward_time_(4.0),
    search_skew(kNoSkew),
    aruco_detect_(false),
    obstacle_active_(false),
    obs_clear_count_(0),
    prev_pattern_(kMoveForward),
    was_avoiding_(false)
  {
    this->declare_parameter<bool>("spot_turn_back", false);
    this->declare_parameter<double>("search_forward_time", 4.0);
    this->declare_parameter<int>("search_skew", 0);

    this->get_parameter("spot_turn_back",      spot_turn_back_);
    this->get_parameter("search_forward_time", search_forward_time_);

    int skew_param = 0;
    this->get_parameter("search_skew", skew_param);
    search_skew = (skew_param == 1) ? kRightSkew : (skew_param == -1) ? kLeftSkew : kNoSkew;

    if (spot_turn_back_)
      search_end_time_ = this->now() + rclcpp::Duration::from_seconds(6.5);

    RCLCPP_INFO(this->get_logger(),
      "Parameters: spot_turn_back=%s  search_forward_time=%.1f  search_skew=%d",
      spot_turn_back_ ? "true" : "false", search_forward_time_, skew_param);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    aruco_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/aruco_detected", 10,
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        aruco_detect_ = (msg->data == 1);
      });

    obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/obstacle_active", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          // Obstacle signal is high — set immediately and reset clear counter
          obstacle_active_ = true;
          obs_clear_count_ = 0;
        } else {
          // Obstacle signal dropped — require OBS_CLEAR_NEEDED consecutive
          // false ticks before considering it actually clear.
          // This prevents a single missed tick from re-triggering search.
          constexpr int OBS_CLEAR_NEEDED = 6;  // 6 × 50 ms = 300 ms hysteresis
          if (obstacle_active_) {
            if (++obs_clear_count_ >= OBS_CLEAR_NEEDED) {
              obstacle_active_ = false;
              obs_clear_count_ = 0;
            }
          }
        }
      });

    search_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SearchTestNode::searchLoop, this));

    RCLCPP_INFO(this->get_logger(), "SearchTestNode ready.");
  }

private:

  enum SearchPattern { kMoveForward, kTurnA, kTurnB, kTurnC };
  enum SearchSkew    { kNoSkew = 0, kLeftSkew = -1, kRightSkew = 1 };

  // ── Main loop ──────────────────────────────────────────────────────
  void searchLoop()
  {
    // ArUco detected → hand off control, go silent
    if (aruco_detect_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[ARUCO] Detected — handing control to aruco node.");
      return;
    }

    // Obstacle active → OA node is publishing /cmd_vel, go silent
    if (obstacle_active_) {
      if (!was_avoiding_) {
        was_avoiding_ = true;
        prev_pattern_ = FollowPattern;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "[SEARCH] Obstacle active — OA node has control.");
      }
      return;
    }

    // Resuming after obstacle cleared
    if (was_avoiding_) {
      was_avoiding_   = false;
      FollowPattern   = prev_pattern_;
      search_ref_set_ = false;   // re-init timing for current phase
      RCLCPP_INFO(this->get_logger(), "[SEARCH] Obstacle cleared — resuming search pattern.");
    }

    callSearchPattern();
  }

  // ── Search pattern (unchanged) ─────────────────────────────────────
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
        spot_done_      = true;
        search_ref_set_ = false;
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

  // ── Members ────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr          aruco_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          obstacle_sub_;
  rclcpp::TimerBase::SharedPtr                                  search_timer_;

  SearchPattern  FollowPattern;
  bool           search_ref_set_;
  bool           spot_turn_back_;
  bool           spot_done_;
  int            search_cycle_;
  rclcpp::Time   search_end_time_;
  double         search_forward_time_;
  SearchSkew     search_skew;
  bool           aruco_detect_;
  bool           obstacle_active_;
  int            obs_clear_count_;
  bool           was_avoiding_;
  SearchPattern  prev_pattern_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
