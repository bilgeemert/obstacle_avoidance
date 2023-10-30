#include "land_vehicle_node.hpp"

using namespace std::placeholders;

LandVehicle::LandVehicle() : Node("land_vehicle_node"){
    sub.joy = this->create_subscription<joyMsg>("joy", 10, std::bind(
                                &LandVehicle::joyCallback, this, _1));
    sub.cloud = this->create_subscription<pointCloudMsg>("/lidar", 100, std::bind(
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
    pcl_conversions::toPCL(msg, pcl_data.merged);
    pcl::fromPCLPointCloud2(pcl_data.merged, pcl_data.cloud); 
    for (size_t i = 0; i < pcl_data.cloud.size(); i++) {
        if(std::isinf(std::abs(pcl_data.cloud.points[i].x)) || 
                std::isnan(std::abs(pcl_data.cloud.points[i].x))){
            pcl_data.cloud.points[i].x = 0.0f;
            pcl_data.cloud.points[i].y = 0.0f;
            pcl_data.cloud.points[i].z = 0.0f;
        }
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandVehicle>());
    rclcpp::shutdown();
    return 0;
}

/*
* Linear Velocity X
* Angular Velocity W
*/