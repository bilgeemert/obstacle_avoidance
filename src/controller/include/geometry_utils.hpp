#ifndef __GEOMETRY_UTILS_HPP__
#define __GEOMETRY_UTILS_HPP__

#include "land_vehicle_type.hpp"

float capAngle(float angle);
float normalizeAngle(float angle);

void body2World(float* body_frame, float* world_frame, float yaw);
void world2Body(float* world_frame, float* body_frame, float yaw);
void cartesian2Spherical(float* cart_data, float* spe_data);
void spherical2Cartesian(float* spe_data, float* cart_data);

#endif