#include "obstacle_avoidance.hpp"

// void ObstacleAvoidance::updateSetpoint(double & linear_x, double & linear_w){
//     for(int i = -30; i < 30; i++){
//         for(int j = 70; j < 110; j++){
//             int angle = normalizeAngle(i);

//             if((histogram[angle][j] >= rules[VEHICLE_RAD]) && (histogram[angle][j] <= rules[SAFETY_DIS])){
//                 std::cout << "distance: " << histogram[angle][j] << " error: " 
//                      << calculateDistance(histogram[angle][j], angle) << " angle: " << angle << std::endl;
//                 linear_w += CAL_YAW(i) * 0.5;
//             }
//         }
//     }
// }

void ObstacleAvoidance::updateSetpoint(double & linear_x, double & linear_w){
    float error = 0.0f;
    bool is_change = false;
    for(int theta = 0; theta < VERTICAL; theta++){
        for(int phi = 0; phi < HORIZONTAL; phi++){
            if(histogram[phi][theta] < calculateDistance(VEHICLE_RADIUS, phi)){  
                continue;
            }else if((histogram[phi][theta] < 1.0) && (phi % 90 != 0)){

                // std::cout << "phi: " << phi << " theta: " << theta
                // << " distance: " << histogram[phi][theta] 
                // << " error: " << calculateDistance(VEHICLE_RADIUS, phi) << std::endl;
                // linear_w += calculateDistance(histogram[phi][theta], phi);
                // std::cout << "force: " << calculateDistance(histogram[phi][theta], phi)
                    //  << " linear w: " << linear_w << std::endl;
                error += avoidanceDistance(histogram[phi][theta], phi);
            }
        }
    }
    std::cout << " ERROR: " << error << std::endl;
}

float ObstacleAvoidance::calculateDistance(float distance, int angle) {
    return distance * (abs(cos(DEG2RAD * angle)) + abs(sin(DEG2RAD * angle)));
}

float ObstacleAvoidance::avoidanceDistance(float distance, int angle){
    float kForce = 0.01f;
    std::cout << " angle: " << angle << std::endl;
    return kForce * cosf(DEG2RAD * angle)/ (sinf(DEG2RAD * angle) * distance);
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