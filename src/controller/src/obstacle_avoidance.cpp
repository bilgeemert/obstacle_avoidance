#include "obstacle_avoidance.hpp"

void ObstacleAvoidance::updateSetpoint(double & linear_x, double & linear_w){
    for(int i = -30; i < 30; i++){
        for(int j = 70; j < 110; j++){
            int angle = normalizeAngle(i);
            if((histogram[angle][j] >= rules[VEHICLE_RAD]) && (histogram[angle][j] <= rules[SAFETY_DIS])){
                std::cout << "distance: " << histogram[angle][j] << " error: " 
                     << calculateDistance(histogram[angle][j], angle) << " angle: " << angle << std::endl;
                linear_w += CAL_YAW(i) * 0.1;
            }
        }
    }
}

float ObstacleAvoidance::calculateDistance(float distance, int angle) {
    return cos(DEG_TO_RAD * angle);
}

void ObstacleAvoidance::detectObject(pointXYZMsg& cloud_data){
    pointIndicesMsg cluster_indices;
    pointXYZMsg::Ptr cloud_m (new pointXYZMsg);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    *cloud_m = cloud_data;
    ec.setClusterTolerance(rules[OBJECT_WIDTH]); 
    tree->setInputCloud(cloud_m);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_m);
    ec.extract(cluster_indices);
    getClusterPoint(cluster_indices, cloud_data);
}

void ObstacleAvoidance::getClusterPoint(pointIndicesMsg& indices_c, pointXYZMsg& cloud_c){
    clearHistogram();
    float cartesian[] = {0, 0, 0};
    for (pointIndicesMsg::const_iterator it =  indices_c.begin(); it !=  indices_c.end(); ++it){
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            cartesian[X] = cloud_c.points[*pit].x;
            cartesian[Y] = cloud_c.points[*pit].y;
            cartesian[Z] = cloud_c.points[*pit].z;
            updateHistogram(cartesian);
        }
    }
}

void ObstacleAvoidance::updateHistogram(float* cc_data){
    float distance = sqrtf(powf(cc_data[X], 2) + powf(cc_data[Y], 2) + powf(cc_data[Z], 2));
    if((distance > MIN_DISTANCE) && (distance < MAX_DISTANCE)){
        Coordinate_t spherical;
        cartesian2Spherical(cc_data, spherical.pos);
        histogram[spherical.pos[PHI]][spherical.pos[THETA]] = spherical.pos[RADIUS];
    }
}

void ObstacleAvoidance::clearHistogram(){
    for(int i = 0; i < HORIZONTAL; i++){
            histogram[i].fill(MAX_DISTANCE);
    }
}