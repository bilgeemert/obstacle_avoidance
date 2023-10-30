#ifndef __LAND_VEHICLE_NODE__
#define __LAND_VEHICLE_NODE__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "land_vehicle_type.hpp"
#include "command.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;
using twistMsg = geometry_msgs::msg::Twist;
using pointCloudMsg = sensor_msgs::msg::PointCloud2;

typedef struct{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
    rclcpp::Subscription<pointCloudMsg>::SharedPtr cloud;
} sub_t;

typedef struct{
    rclcpp::Publisher<twistMsg>::SharedPtr joy;
} pub_t;

class LandVehicle : public rclcpp::Node{
private:
    pub_t pub;
    sub_t sub;

public:
    LandVehicle();
    void joyCallback(const joyMsg &);
    void pointCloudCallback(const pointCloudMsg &);
};

#endif