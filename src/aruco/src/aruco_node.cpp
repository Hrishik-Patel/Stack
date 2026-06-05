#include "rclcpp/rclcpp.hpp"

class ArucoNode : public rclcpp::Node {
public:
    ArucoNode() : Node("aruco_node") {
        RCLCPP_INFO(this->get_logger(), "==== Entered ArUco Node ====");
        
        this->declare_parameter<std::string>("search_pattern", "none");
        std::string pattern;
        this->get_parameter("search_pattern", pattern);
        
        RCLCPP_INFO(this->get_logger(), "Active Search Pattern: %s", pattern.c_str());
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to exit and return to menu");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
