#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>
#include <RemoteXY.h>

#define REMOTEXY_WIFI_SSID "Obstacle_Avoidance"
#define REMOTEXY_WIFI_PASSWORD "obstacle_avoidance"
#define REMOTEXY_SERVER_PORT 6377

#define HEADER_BYTE   27
#define FOOTHER_BYTE  71
#define RX_DELAY      10

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 62 bytes
  { 255,4,0,1,0,55,0,16,24,0,5,52,241,16,46,46,6,28,31,2,
  1,81,40,22,7,6,27,31,31,79,78,0,79,70,70,0,70,16,96,4,
  8,8,12,37,0,2,1,81,49,22,7,6,27,31,31,79,78,0,79,70,
  70,0 };
  
struct {
  int8_t joystick_1_x; // from -100 to 100  
  int8_t joystick_1_y; // from -100 to 100  

  uint8_t switch_1; // =1 if switch ON and =0 if OFF 
  uint8_t switch_2; // =1 if switch ON and =0 if OFF 
  uint8_t led_1; // led state 0 .. 1 
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

typedef enum {HEADER, JOY_X, JOY_Y, OBS_FLAG, ARM_FLAG, FOOTHER, ALL_DATA} data_t;

uint8_t  buffer[ALL_DATA];
uint64_t prev_time;

void transmitBuffer();
void updateData();

void setup(){
  RemoteXY_Init (); 
  Serial.begin(9600);
  buffer[HEADER]  = HEADER_BYTE;
  buffer[FOOTHER] = FOOTHER_BYTE;
  prev_time = millis();
}

void loop(){ 
  RemoteXY_Handler ();
  updateData();
  transmitBuffer();
}

void transmitBuffer(){
  if((millis() - prev_time) > RX_DELAY){
    prev_time = millis();
    Serial.write(buffer, ALL_DATA);
  }
}

void updateData(){
  buffer[JOY_X] = (uint8_t)map(RemoteXY.joystick_1_x ,-100, 100, 0, 200);
  buffer[JOY_Y] = (uint8_t)map(RemoteXY.joystick_1_y ,-100, 100, 0, 200);
  buffer[OBS_FLAG] = RemoteXY.switch_1;
  buffer[ARM_FLAG] = RemoteXY.switch_2;
}
