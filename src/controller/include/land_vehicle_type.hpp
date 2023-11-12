#ifndef __LAND_VEHICLE_TYPE_HPP__
#define __LAND_VEHICLE_TYPE_HPP__

#include <cmath>

#define OFSET 0.1f

#define VEHICLE_RADIUS (0.6f)
#define MAX_DISTANCE (3.5f)
#define MIN_DISTANCE (0.0f)

#define F2P(x) (1000/x)
#define DEG2RAD (M_PI/180.0f)
#define RAD2DEG (180.0f/M_PI)

#define VERTICAL   181 
#define HORIZONTAL 360
#define ERROR (-1)
#define OK    (1)

#define CAL_YAW(X) (X < 0 ? 0.1 : -0.1)
#define UPDATE_DATA(x) (abs(x) > OFSET ? x : 0.0f) 

typedef enum { X, Y, Z, ALL_CC } cc_mode_t;
typedef enum { RADIUS, THETA, PHI, ALL_SC } sc_mode_t;
typedef enum {
  THRESHOLD_DIS, SAFETY_DIS, VEHICLE_RAD, 
  OBJECT_WIDTH, OBS_ALL
} obs_config_t;

typedef struct{
    union{
        struct{
            float x;
            float y;
            float z;
        };
        struct{
            float radius;
            float theta;
            float phi;
        };
        float pos[3];
    };
}Coordinate_t;

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