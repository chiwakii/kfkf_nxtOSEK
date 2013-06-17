#ifndef _SENSORMANAGER_H_
#define _SENSORMANAGER_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
//#include <stdlib.h>

//#define LIGHT_BUFFER_LENGTH_MAX 250
//#define GYRO_BUFFER_LENGTH_MAX 250


typedef struct tag_Sensor {
//////sensor values////////////////////////////
U16 light;
//U16 light_buffer[LIGHT_BUFFER_LENGTH_MAX];
U16 light_ave;
U16 black;
U16 white;
U16 target_gray;
U16 target_gray_base;

U16 gyro;
U16 gyro_offset;
U16 gyro_offset_base;

boolean touch;
S32 distance;
S32 count_left;
S32 count_right;
S32 count_tail;
U16 battery;

boolean bottle_is_left;
boolean bottle_is_right;

//S32 distance;
//S32 threshold_gray;
//S32 calib_gray;
//S32 white_gray_threshold;
//S32 wood;
//S32 sonar_value;

//U16 prev_light_value;
//U16 prev_gyro_value;

//U16 light_buffer[LIGHT_BUFFER_LENGTH_MAX];
//U16 gyro_buffer[GYRO_BUFFER_LENGTH_MAX];
//U16 light_max;
//U16 light_min;
//U16 light_ave;
//U16 light_diff;

//U16 LIGHT_BUFFER_LENGTH;
//U16 GYRO_BUFFER_LENGTH;

#define V_LIGHT_BUFFER_LENGTH 10
#define V_GYRO_BUFFER_LENGTH 10

//S8 light_V;
//S8 gyro_V;
//byte bottle_is_left;
//byte bottle_is_right;

} Sensor_t;

#endif

