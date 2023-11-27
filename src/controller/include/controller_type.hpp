#ifndef __LAND_VEHICLE_TYPE_HPP__
#define __LAND_VEHICLE_TYPE_HPP__

#include <cmath>

#define OFSET 0.1f

#define F2P(x) (1000/x)
#define DEG2RAD (M_PI/180.0f)
#define RAD2DEG (180.0f/M_PI)

#define VERTICAL   181 
#define HORIZONTAL 360

#define ERROR (-1)
#define OK    (1)

#define COEF(X) (X < 0 ? 0.1 : -0.1)
#define DETECT_RANGE(X) abs(std::pow((cosf(DEG2RAD * X)), 3))

#define OFFSET_EXCEPTION(x) (abs(x) > OFSET ? x : 0.0f) 

typedef enum { 
  X, Y, Z, ALL_CC 
} cc_mode_e;

typedef enum { 
  RADIUS, THETA, PHI, ALL_SC 
} sc_mode_e;

typedef enum {
  THRESHOLD_DIS, SAFETY_DIS, VEHICLE_RAD, 
  VEHICLE_WIDTH, OBS_ALL
} rules_e;

typedef enum {
  MAX_DIS, MIN_DIS, SENSOR_ALL
} sensor_rules_e;

typedef enum{ 
  LINEAR_V, ANGULAR_V, RESULT_V, ALL_V 
} velocity_e;

typedef enum { 
  ROLL, PITCH, YAW, R_ALL
} rotation_angle_t;

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

typedef union{
  struct{
    float linear;
    float angular;
    float result;
  };
  float data[ALL_V];
}Joy_t;

#endif