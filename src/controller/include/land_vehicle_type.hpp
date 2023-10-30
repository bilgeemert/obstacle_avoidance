#ifndef __LAND_VEHICLE_TYPE_HPP__
#define __LAND_VEHICLE_TYPE_HPP__

#include <cmath>

#define OFSET 0.1f
#define UPDATE_DATA(x) (abs(x) > OFSET ? x : 0.0f) 

typedef enum {
  LINEAR, ANGULAR, SPEED_ALL_DATA
} Joy_e;

typedef union{
  struct{
    float linear;
    float angular;
  };
  float data[SPEED_ALL_DATA];
}Joy_t;

#endif