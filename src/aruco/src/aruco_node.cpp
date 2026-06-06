#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <atomic>
#include <thread>
#include <vector>
#include <algorithm>
#include <memory>
#include <cmath>

class ArUcoMatrixFollower : public rclcpp::Node
{
public:
    ArUcoMatrixFollower()
        : Node("aruco_follower_node"),
          chosen_target_id_(-1),
          is_waiting_for_input_(false)
    {
        image_sub_ =
            create_subscription<sensor_msgs::msg::CompressedImage>(
                "/zed/zed_node/rgb/color/rect/image/compressed",
                10,
                std::bind(
                    &ArUcoMatrixFollower::rgb_callback,
                    this,
                    std::placeholders::_1));

        depth_sub_ =
            create_subscription<sensor_msgs::msg::Image>(
                "/zed/zed_node/depth/depth_registered",
                10,
                std::bind(
                    &ArUcoMatrixFollower::depth_callback,
                    this,
                    std::placeholders::_1));

        cmd_vel_pub_ =
            create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10);

        prompt_timer_ =
            create_wall_timer(
                std::chrono::seconds(2),
                std::bind(
                    &ArUcoMatrixFollower::timer_callback,
                    this));

        dictionary_ =
            cv::aruco::getPredefinedDictionary(
                cv::aruco::DICT_5X5_100);

        detector_ =
            std::make_unique<cv::aruco::ArucoDetector>(
                dictionary_,
                cv::aruco::DetectorParameters());

        RCLCPP_INFO(
            get_logger(),
            "ArUco Depth Follower Started");
    }

private:
    void depth_callback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            auto cv_ptr =
                cv_bridge::toCvShare(
                    msg,
                    "32FC1");

            latest_depth_ =
                cv_ptr->image.clone();

            RCLCPP_INFO_ONCE(
                get_logger(),
                "Depth image: %dx%d Type=%d",
                latest_depth_.cols,
                latest_depth_.rows,
                latest_depth_.type());
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Depth conversion error: %s",
                e.what());
        }
    }

    void rgb_callback(
        const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            if (is_waiting_for_input_)
            {
                stop_robot();
                return;
            }

            cv::Mat compressed(
                1,
                msg->data.size(),
                CV_8UC1,
                (void *)msg->data.data());

            cv::Mat frame =
                cv::imdecode(
                    compressed,
                    cv::IMREAD_COLOR);

            if (frame.empty())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Decoded image is empty");
                return;
            }

            RCLCPP_INFO_ONCE(
                get_logger(),
                "RGB image: %dx%d",
                frame.cols,
                frame.rows);

            RCLCPP_INFO(
                get_logger(),
                "Checkpoint 1");

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            RCLCPP_INFO(
                get_logger(),
                "Checkpoint 2");

            detector_->detectMarkers(
                frame,
                corners,
                ids,
                rejected);

            RCLCPP_INFO(
                get_logger(),
                "Checkpoint 3");

            latest_ids_ = ids;

            bool target_found = false;
            float distance_z = 0.0f;

            if (chosen_target_id_ != -1 &&
                !latest_depth_.empty())
            {
                auto it =
                    std::find(
                        ids.begin(),
                        ids.end(),
                        chosen_target_id_);

                if (it != ids.end())
                {
                    int idx =
                        std::distance(
                            ids.begin(),
                            it);

                    cv::Point2f center(0.f, 0.f);

                    for (const auto &p : corners[idx])
                        center += p;

                    center *= 0.25f;

                    int px =
                        static_cast<int>(
                            std::round(center.x));

                    int py =
                        static_cast<int>(
                            std::round(center.y));

                    px =
                        std::clamp(
                            px,
                            0,
                            latest_depth_.cols - 1);

                    py =
                        std::clamp(
                            py,
                            0,
                            latest_depth_.rows - 1);

                    distance_z =
                        latest_depth_.at<float>(
                            py,
                            px);

                    RCLCPP_INFO(
                        get_logger(),
                        "Marker=%d Pixel=(%d,%d) Depth=%.3f",
                        chosen_target_id_,
                        px,
                        py,
                        distance_z);

                    if (std::isfinite(distance_z) &&
                        distance_z > 0.1f &&
                        distance_z < 10.0f)
                    {
                        target_found = true;
                    }
                }
                else
                {
                    chosen_target_id_ = -1;
                }
            }

            geometry_msgs::msg::Twist cmd;

            if (target_found)
            {
                if (distance_z <= 0.5f)
                {
                    cmd.linear.x = 0.0;
                }
                else if (distance_z <= 4.0f)
                {
                    cmd.linear.x = 0.8;
                }
                else
                {
                    cmd.angular.z = 0.4;
                }
            }

            cmd_vel_pub_->publish(cmd);
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "OpenCV Exception: %s",
                e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "STD Exception: %s",
                e.what());
        }
    }

    void timer_callback()
    {
        if (chosen_target_id_ == -1 &&
            !latest_ids_.empty() &&
            !is_waiting_for_input_)
        {
            is_waiting_for_input_ = true;

            std::thread(
                [this]()
                {
                    std::cout
                        << "\nDetected IDs: ";

                    for (auto id : latest_ids_)
                        std::cout
                            << "[" << id << "] ";

                    std::cout
                        << "\nSelect target ID: ";

                    std::cin
                        >> chosen_target_id_;

                    std::cout
                        << "Locked on "
                        << chosen_target_id_
                        << std::endl;

                    is_waiting_for_input_ = false;
                })
                .detach();
        }
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr prompt_timer_;

    cv::Mat latest_depth_;
    std::vector<int> latest_ids_;

    int chosen_target_id_;
    std::atomic<bool> is_waiting_for_input_;

    cv::aruco::Dictionary dictionary_;
    std::unique_ptr<cv::aruco::ArucoDetector> detector_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ArUcoMatrixFollower>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}