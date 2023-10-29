#include "command.hpp"

Command::Command() : Node("command_node"){
    joy_sub = this->create_subscription<joyMsg>("joy", 10, std::bind(
                &Command::commandCallback, this, std::placeholders::_1));
    joy_pub = this->create_publisher<twistMsg>("cmd_vel", 10);
}

void Command::commandCallback(const joyMsg & msg){
    twistMsg data;
    data.linear.x  = msg.axes[0];
    data.angular.z = msg.axes[1];
    joy_pub->publish(data);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Command>());
    rclcpp::shutdown();
    return 0;
}