#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "command_type.hpp"

using int32Msg = std_msgs::msg::Int32;
using twistMsg = geometry_msgs::msg::Twist;
using joyMsg = sensor_msgs::msg::Joy;

class Command : public rclcpp::Node{
private:
    rclcpp::Subscription<joyMsg>::SharedPtr joy_sub;
    rclcpp::Publisher<twistMsg>::SharedPtr joy_pub;

public:
    Command();
    void commandCallback(const joyMsg &);
};

#endif