#include "command_type.hpp"
#include "command_node.hpp"

double mapValues(double data, double in_min, double in_max, double out_min, double out_max){   
  return ((((data - in_min)*(out_max - out_min))/(in_max - in_min)) + out_min);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<SerialComm>());
    rclcpp::shutdown();
    return 0;
}