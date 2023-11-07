#include "obstacle_avoidance.hpp"

void ObstacleAvoidance::updateSetpoint(double & linear_x, double & linear_w){
for(int i = -45; i < 45; i++){
    int angle = normalizeAngle(i);
    // for(int j = 0; j < VERTICAL; j++){
        if(histogram[angle][92] >= rules[VEHICLE_RAD] && histogram[angle][92] <= rules[SAFETY_DIS]){
            std::cout << "error: " << calculateDistance(histogram[angle][92], angle) << " angle: " << angle << std::endl;
            // linear_w += calculateDistance(histogram[angle][92], angle);
        }
    // }
}
}

// float ObstacleAvoidance::calculateDistance(float distance, int angle) {
//     return distance * abs(cosf(DEG_TO_RAD * angle)) + distance * abs(sinf(DEG_TO_RAD * angle));
// }

float ObstacleAvoidance::calculateDistance(float distance, int angle) {
    return -sinf(DEG_TO_RAD * angle) / 2;
}

// Update Histogram
void ObstacleAvoidance::detectObject(pointXYZMsg& cloud_data){
    pointIndicesMsg cluster_indices;

    pointXYZMsg::Ptr cloud_m (new pointXYZMsg);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    *cloud_m = cloud_data;
    ec.setClusterTolerance(rules[OBJECT_WIDTH]); 
    tree->setInputCloud(cloud_m);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(256);
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
    // std::cout  << "distance: " << distance << std::endl;
    if((distance > 0.0) && (distance <= rules[THRESHOLD_DIS])){
        Coordinate_t spherical;
        cartesian2Spherical(cc_data, spherical.pos);
        histogram[spherical.pos[PHI]][spherical.pos[THETA]] = spherical.pos[RADIUS];
        // std::cout << "Phi: " << spherical.pos[PHI] << " Theta: " 
        //           << spherical.pos[THETA] << " Radius: " << spherical.pos[RADIUS] << std::endl;
    }
}

void ObstacleAvoidance::clearHistogram(){
    for(int i = 0; i < HORIZONTAL; i++){
            histogram[i].fill(0.0f);
    }
}