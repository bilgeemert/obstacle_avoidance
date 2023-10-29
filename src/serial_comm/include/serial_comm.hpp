#ifndef __SERIAL_COMM_HPP__
#define __SERIAL_COMM_HPP__

#include <iostream>

#include <fcntl.h>   
#include <errno.h> 
#include <unistd.h>  
#include <termios.h> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial_type.hpp"

using JoyMsg = sensor_msgs::msg::Joy;
double mapValues(double, double, double, double, double);

class SerialComm : public rclcpp::Node{
    private:
        device_t device;
        JoyMsg msg;
        rx_t data;

    private:
        rclcpp::TimerBase::SharedPtr timer_;  
        rclcpp::Publisher<JoyMsg>::SharedPtr joy_pub;

    public:
        SerialComm();
        ~SerialComm();
        bool initPort();
        bool configure();
        void dataRead();
        void initParam();
};

#endif