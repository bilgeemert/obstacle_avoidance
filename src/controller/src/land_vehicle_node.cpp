#include "land_vehicle_node.hpp"

using namespace std::placeholders;

LandVehicle::LandVehicle() : Node("land_vehicle_node"){
    declareParameters();
    initTopic();
}

void LandVehicle::joyCallback(const joyMsg &msg){
    data.linear.x  = UPDATE_DATA(msg.axes[0]);
    data.angular.z = UPDATE_DATA(msg.axes[1]);
    obstacleAvoidance();
}

void LandVehicle::obstacleAvoidance(){
    updateSetpoint(data.linear.x, data.angular.z);
    makerCallback();
    pub.markers->publish(marker_array);
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
    detectObject(pcl_data.cloud);
}

void LandVehicle::makerCallback(){
    for(int i = 0; i < ALL_V; i++){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "X1/base_link/front_laser";
        marker.ns = "markers" + std::to_string(i);
        marker.type   = markerMsg::ARROW;
        marker.id = i;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.04;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = i % 3 == 0 ? 1.0 : 0.0;
        marker.color.g = i % 3 == 1 ? 1.0 : 0.0;
        marker.color.b = i % 3 == 2 ? 1.0 : 0.0;
        marker.points.push_back(first_point[i]);
        marker.points.push_back(last_point[i]);
        marker_array.markers.push_back(marker);
        marker.action = markerMsg::DELETEALL;
    }
}

void LandVehicle::declareParameters(){
    this->declare_parameter("rules", std::vector<double> (4, 0.0)); 
    rules = this->get_parameter("rules").as_double_array();
    for(int i=0; i < ALL_V; i++){
        first_point[i].x = 0.0;
        first_point[i].y = 0.0;
        first_point[i].z = 0.0;
    } 
}

void LandVehicle::initTopic(){
    sub.joy = this->create_subscription<joyMsg>("command_data", 10, std::bind(
                                &LandVehicle::joyCallback, this, _1));
    sub.cloud = this->create_subscription<pointCloudMsg>("/lidar", 100, std::bind(
                                &LandVehicle::pointCloudCallback, this, _1));

    pub.joy = this->create_publisher<twistMsg>("cmd_vel", 10);
    pub.markers = this->create_publisher<markerArrayMsg>("marker_visulation", 100);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandVehicle>());
    rclcpp::shutdown();
    return 0;
}