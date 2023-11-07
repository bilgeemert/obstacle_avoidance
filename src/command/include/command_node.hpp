#ifndef __COMMAND_NODE_HPP__
#define __COMMAND_NODE_HPP__

#include <iostream>

#include <fcntl.h>   
#include <errno.h> 
#include <unistd.h>  
#include <termios.h> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using joyMsg = sensor_msgs::msg::Joy;


#endif