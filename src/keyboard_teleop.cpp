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
    char buff [50];
    int c;
    auto message = geometry_msgs::msg::Twist();

    // Initialize ncurses
    initscr();
    keypad(stdscr, TRUE);
    noecho();
    sprintf(buff, "Publishing to topic: %s.\n", topic_name.c_str());
    addstr(buff);
    addstr("Press arroy keys to move robot. Press 's' to STOP. Press 'q' to QUIT.\n");

    do {
        c = getch();
        switch (c) {
        case 259: // key up | forward
            message.linear.x = 1;
            message.linear.y = 0;
            message.linear.z = 0;
            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = 0;
            break;
        case 260: // key left
            message.linear.x = 0;
            message.linear.y = 0;
            message.linear.z = 0;
            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = 1;
            break;
        case 261: // key right
            message.linear.x = 0;
            message.linear.y = 0;
            message.linear.z = 0;
            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = -1;
            break;
        case 258: // key down | backward
            message.linear.x = -1;
            message.linear.y = 0;
            message.linear.z = 0;
            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = 0;
            break;
        case 115: // lower-case s | STOP
            message.linear.x = 0;
            message.linear.y = 0;
            message.linear.z = 0;
            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = 0;
            break;
        default:
            continue;
            break;
        }

        // publish the message
        cmd_vel_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear: x: '%.2f', y: '%.2f', z: '%.2f'\r",
                    message.linear.x, message.linear.y, message.linear.z);
        RCLCPP_INFO(this->get_logger(), "Publishing: angular: x: '%.2f', y: '%.2f', z: '%.2f'\r\n",
                    message.angular.x, message.angular.y, message.angular.z);

    } while(c != 113); // lower-case q | QUIT

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
