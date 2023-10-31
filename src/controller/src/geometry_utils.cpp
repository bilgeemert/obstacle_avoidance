#include "geometry_utils.hpp"
#include <cmath>

float capAngle(float angle) {
  angle = fmod(angle + 180.0, 360.0);
  while (angle < 0) {
    angle += 360.0;
  }
  return angle - 180.0;
}

float normalizeAngle(float angle){
    while(angle < 0)    { angle = angle + 360; } 
    while(angle >= 360) { angle = angle - 360; }
    return angle;
}

void body2World(float* body_frame, float* world_frame, float yaw ){
    world_frame[X] = body_frame[X] * cosf(yaw) - body_frame[Y] * sinf(yaw);
    world_frame[Y] = body_frame[X] * sinf(yaw) + body_frame[Y] * cosf(yaw);
}

void world2Body(float* world_frame, float* body_frame, float yaw){
    body_frame[X] = world_frame[X] * cosf(yaw) + world_frame[Y] * sinf(yaw);
    body_frame[Y] = -world_frame[X] * sinf(yaw) + world_frame[Y] * cosf(yaw);
}

void cartesian2Spherical(float* cart_data, float* spe_data){
    spe_data[RADIUS] = sqrtf(powf(cart_data[X], 2) + powf(cart_data[Y], 2) + powf(cart_data[Z], 2));
    spe_data[THETA] = acosf(cart_data[Z] / spe_data[RADIUS]) * RAD_TO_DEG;
    spe_data[THETA] = static_cast<int>(std::round(spe_data[THETA]));
    spe_data[PHI] = std::atan2(cart_data[Y], cart_data[X]) * RAD_TO_DEG;
    spe_data[PHI] = static_cast<int>(std::round(spe_data[PHI]));
    spe_data[PHI] = normalizeAngle(spe_data[PHI]);

}

void spherical2Cartesian(float* spe_data, float* cart_data){
   cart_data[X] = spe_data[RADIUS] * sin(spe_data[THETA] * DEG_TO_RAD) * cos(spe_data[PHI] * DEG_TO_RAD);
   cart_data[Y] = spe_data[RADIUS] * sin(spe_data[THETA] * DEG_TO_RAD) * sin(spe_data[PHI] * DEG_TO_RAD);
   cart_data[Z] = spe_data[RADIUS] * cos(spe_data[THETA] * DEG_TO_RAD);
}