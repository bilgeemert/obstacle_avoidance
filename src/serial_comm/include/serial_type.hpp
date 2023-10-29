#ifndef __SERIAL_COMM_SERIAL_TYPE_HPP__
#define __SERIAL_COMM_SERIAL_TYPE_HPP__

#include <cstring>

#define HEADER_     27
#define FOOTHER_    71
#define HEADER_LEN  1
#define FOOTHER_LEN 1
#define PAYLOAD_LEN 4

typedef enum {HEADER, JOY_X, JOY_Y, OBS_FLAG, 
                ARM_FLAG, FOOTHER, ALL_DATA} data_t;

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

#endif