#include "rclcpp/rclcpp.hpp"

class AttNode : public rclcpp::Node {
public:
    AttNode() : Node("att_node") {
        RCLCPP_INFO(this->get_logger(), "==== Entered ATT (Attitude Tracking) Node ====");
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to exit and return to menu.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AttNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
