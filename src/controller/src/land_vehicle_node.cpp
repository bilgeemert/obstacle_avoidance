#include "land_vehicle_node.hpp"

using namespace std::placeholders;

LandVehicle::LandVehicle() : Node("land_vehicle_node"){
    sub.joy = this->create_subscription<joyMsg>("joy", 10, std::bind(
                                &LandVehicle::joyCallback, this, _1));
    sub.cloud = this->create_subscription<pointCloudMsg>("X1/base_link/front_laser", 100, std::bind(
                                &LandVehicle::pointCloudCallback, this, _1));

    pub.joy = this->create_publisher<twistMsg>("cmd_vel", 10);
}

void LandVehicle::joyCallback(const joyMsg &msg){
    twistMsg data;
    data.linear.x  = UPDATE_DATA(msg.axes[0]);
    data.angular.z = UPDATE_DATA(msg.axes[1]);
    pub.joy->publish(data);

}

void LandVehicle::pointCloudCallback(const pointCloudMsg &msg){

}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandVehicle>());
    rclcpp::shutdown();
    return 0;
}