#ifndef __COMMAND_NODE_HPP__
#define __COMMAND_NODE_HPP__

#include <iostream>

#include <fcntl.h>   
#include <errno.h> 
#include <unistd.h>  
#include <termios.h> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include "command_type.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;

class Command: public rclcpp::Node{
private:
    rx_t data;
    joyMsg joy_data;
    device_t device;
    std::string control_unit;
    rclcpp::TimerBase::SharedPtr timer_;  
    rclcpp::Publisher<joyMsg>::SharedPtr command_pub;
    rclcpp::Subscription<joyMsg>::SharedPtr joy_sub;
    rclcpp::Subscription<int32Msg>::SharedPtr keyboard_sub;

public:
    Command();
    ~Command();
    void controlSelection();
    void keyboardCallback(const int32Msg);
    void joyCallback(const joyMsg);
    bool initPort();
    bool configure();
    void dataRead();
    void initParam();
};

#endif