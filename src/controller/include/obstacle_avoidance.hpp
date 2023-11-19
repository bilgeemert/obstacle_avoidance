#ifndef __OBSTACLE_AVOIDANCE_HPP__
#define __OBSTACLE_AVOIDANCE_HPP__

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include "visualization_msgs/msg/marker.hpp"
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/point.hpp"

#include "land_vehicle_type.hpp"
#include "geometry_utils.hpp"

typedef struct {
    pcl::PCLPointCloud2 merged;
	pcl::PointCloud<pcl::PointXYZ> cloud;
}pcl_t;

typedef enum{ LINEAR_V, ANGULAR_V, ORIENTATION_V, ALL_V } velocity_e;

using markerMsg       = visualization_msgs::msg::Marker;
using pointXYZMsg     = pcl::PointCloud<pcl::PointXYZ>;
using pointMsg        = geometry_msgs::msg::Point;
using pointIndicesMsg = std::vector<pcl::PointIndices>;


class ObstacleAvoidance{
private:
    std::array<std::array<float, VERTICAL>, HORIZONTAL> histogram;

protected:
    std::vector<double> rules;
    pointMsg first_point[ALL_V];
    pointMsg last_point[ALL_V];

public:
    void detectObject(pointXYZMsg&);
    void getClusterPoint(pointIndicesMsg& , pointXYZMsg&);
    void updateHistogram(float*);
    void clearHistogram();
    void updateSetpoint(double &, double &);
    float calculateDistance(float, int);
    float avoidanceDistance(float, int);
};

#endif