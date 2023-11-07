#ifndef __COMMAND_NODE_HPP__
#define __COMMAND_NODE_HPP__

#include <iostream>

#include <fcntl.h>   
#include <errno.h> 
#include <unistd.h>  
#include <termios.h> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "command_type.hpp"

using joyMsg = sensor_msgs::msg::Joy;

class Command: public rclcpp::Node{
private:
    rx_t data;
    joyMsg msg;
    device_t device;
    rclcpp::TimerBase::SharedPtr timer_;  
    rclcpp::Publisher<joyMsg>::SharedPtr joy_pub;

public:
    Command();
    ~Command();
    bool initPort();
    bool configure();
    void dataRead();
    void initParam();
};

#endif