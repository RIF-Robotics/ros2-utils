// #############################################################################
// keyboard_teleop.cpp
//
// Node allows user to publish Twist commands to a topic (default: /cmd_vel).
// #############################################################################

#include <ncurses.h>

#include "rif-ros2-utils/keyboard_teleop.h"

KeyboardTeleop::KeyboardTeleop() : Node("keyboard_teleop") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 1);
    read_arrow_keys();
}

void KeyboardTeleop::read_arrow_keys() {
    int c;
    auto message = geometry_msgs::msg::Twist();

    // Initialize ncurses
    initscr();
    keypad(stdscr, TRUE);
    noecho();
    addstr("Press arroy keys to move robot. Press 'q' to quit.\n");

    do {
        c = getch();
        switch (c) {
        case 259: // key up
            message.linear.x = 0;
            message.linear.y = 2;
            message.linear.z = 0;
            break;
        case 260: // key left
            message.linear.x = -2;
            message.linear.y = 0;
            message.linear.z = 0;
            break;
        case 261: // key right
            message.linear.x = 2;
            message.linear.y = 0;
            message.linear.z = 0;
            break;
        case 258: // key down
            message.linear.x = 0;
            message.linear.y = -2;
            message.linear.z = 0;
            break;
        default:
            continue;
            break;
        }

        // publish the message
        cmd_vel_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear: x: '%.2f', y: '%.2f', z: '%.2f' \r",
                    message.linear.x, message.linear.y, message.linear.z);

    } while(c != 113); // lower-case q

    endwin();
    return;
}

// --------------------------
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}
