#ifndef __COMMAND_TYPE_HPP__
#define __COMMAND_TYPE_HPP__

#include <string>
#include <cstdint>

#define HEADER_     27
#define FOOTHER_    71
#define HEADER_LEN  1
#define FOOTHER_LEN 1
#define PAYLOAD_LEN 4

#define KEYCODE_W 87 
#define KEYCODE_A 65
#define KEYCODE_S 83
#define KEYCODE_D 68
#define KEYCODE_X 88

#define BAUDRATE B9600

typedef enum {
    HEADER, JOY_X, JOY_Y, OBS_FLAG, 
    ARM_FLAG, FOOTHER, ALL_DATA
} data_t;

typedef struct{
    std::string file_name;
    int port;
} device_t;

typedef struct{
    int button[2];
    float joy[2];
    int state = 0;
    uint8_t curr_byte;
    uint8_t prev_byte;
    uint8_t buffer[ALL_DATA];
} rx_t;

double mapValues(double, double, double, double, double);

#endif