#include "obstacle_avoidance.hpp"

void ObstacleAvoidance::updateSetpoint(double & linear_x, double & linear_w){
    float error = 0.0f;
    bool is_change = false;
    for(int theta = 0; theta < VERTICAL; theta++){
        for(int phi = 0; phi < HORIZONTAL; phi++){
            int angle = phi % 15;
            if((histogram[angle][theta] < 0.80f) && (histogram[359 - angle][theta] < 0.80f) ){
                linear_x = 0.0;
                std::cout << "STOP" << std::endl;
                int left_force = histogram[89][theta] + histogram[90][theta] + histogram[91][theta];
                int right_force = histogram[269][theta] + histogram[270][theta] + histogram[271][theta];
                linear_w = left_force > right_force ?  0.5 : -0.5;
            }
            if(histogram[phi][theta] < calculateDistance(VEHICLE_RADIUS, phi)){  
                continue;
            }else if((histogram[phi][theta] < (2.2 * CAL_FORCE(phi))) && (phi % 90 != 0)){
                if(linear_x * cosf(DEG2RAD * phi) > 0){
                    error += avoidanceDistance(histogram[phi][theta], phi);
                }
            }
        }
    }
    std::cout << " ERROR: " << error << "\n_______________________" << std::endl;
    if(abs(linear_x) > OFSET  && abs(error) > OFSET){
        linear_w = error;
    }

    last_point[LINEAR_V].x = linear_x;
    first_point[ANGULAR_V].x = last_point[LINEAR_V].x;
    last_point[ANGULAR_V].x = last_point[LINEAR_V].x;
    last_point[ANGULAR_V].y = linear_w;
    last_point[ORIENTATION_V].x = last_point[LINEAR_V].x;
    last_point[ORIENTATION_V].y = last_point[ANGULAR_V].y;

}

float ObstacleAvoidance::calculateDistance(float distance, int angle) {
    return distance * (abs(cos(DEG2RAD * angle)) + abs(sin(DEG2RAD * angle)));
}

float ObstacleAvoidance::avoidanceDistance(float distance, int angle){
    float kForce = -0.1f;
    std::cout << " angle: " << angle << std::endl;
    return kForce * cosf(DEG2RAD * angle) * sinf(DEG2RAD * angle) / distance;
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