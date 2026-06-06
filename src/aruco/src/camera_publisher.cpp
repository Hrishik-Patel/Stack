#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher() : Node("camera_publisher") {
        // --- Declare Parameters ---
        this->declare_parameter<int>("device_id", 1);          // 0 is usually the built-in webcam
        this->declare_parameter<double>("frame_rate", 30.0);   // Frames per second
        this->declare_parameter<string>("topic_name", "/camera/image_raw");

        // --- Get Parameter Values ---
        int device_id = this->get_parameter("device_id").as_int();
        double frame_rate = this->get_parameter("frame_rate").as_double();
        string topic_name = this->get_parameter("topic_name").as_string();

        // --- Initialize OpenCV Video Capture ---
        cap_.open(device_id);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video device index %d", device_id);
            throw runtime_error("Failed to open camera.");
        }

        // --- Set Camera Resolution (640x480) ---
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        // --- Initialize Publisher ---
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

        // --- Create a Timer Based on Desired Frame Rate ---
        // Converts double FPS to a precise nanosecond representation duration
        auto timer_period = chrono::duration<double>(1.0 / frame_rate);
        timer_ = this->create_wall_timer(
            chrono::duration_cast<chrono::nanoseconds>(timer_period),
            bind(&CameraPublisher::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Publishing camera footage to '%s' at %.1f FPS...", topic_name.c_str(), frame_rate);
    }

    // --- Destructor replaces destroy_node resource cleanup ---
    ~CameraPublisher() override {
        if (cap_.isOpened()) {
            cap_.release();
            RCLCPP_INFO(this->get_logger(), "Released camera device interface safely.");
        }
    }

private:
    void timer_callback() {
        cv::Mat frame;
        bool ret = cap_.read(frame);

        if (ret && !frame.empty()) {
            // Convert OpenCV BGR Mat frame to ROS 2 Image message pointer
            std_msgs::msg::Header header;
            std::shared_ptr<sensor_msgs::msg::Image> img_msg = 
                cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

            // Assign timestamps and frame ID (crucial for sensor synchronization)
            img_msg->header.stamp = this->get_clock()->now();
            img_msg->header.frame_id = "camera_frame";

            // Publish frame
            publisher_->publish(*img_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to grab frame from camera source.");
        }
    }

    // --- ROS 2 Handles & Infrastructure ---
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(make_shared<CameraPublisher>());
    } catch (const exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node initialization termination crash: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}