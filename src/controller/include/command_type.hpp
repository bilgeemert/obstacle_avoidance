#ifndef __COMMAND_TYPE_HPP__
#define __COMMAND_TYPE_HPP__

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