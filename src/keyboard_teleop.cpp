// #############################################################################
// keyboard_teleop.cpp
//
// Node allows user to publish Twist commands to a topic (default: /cmd_vel).
// #############################################################################

#include "rif-ros2-utils/keyboard_teleop.h"

void KeyboardTeleop::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

// --------------------------
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}
