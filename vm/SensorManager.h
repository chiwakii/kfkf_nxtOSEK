#ifndef _SENSORMANAGER_H_
#define _SENSORMANAGER_H_

#include "Common.h"

//#define LIGHT_BUFFER_LENGTH_MAX 250
//#define GYRO_BUFFER_LENGTH_MAX 250


typedef struct tag_Sensor
{
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

	U8 touch;
	S32 distance;
	int count_left;
	int count_right;
	int count_tail;
	U16 battery;

	U8 bottle_is_left;
	U8 bottle_is_right;
}Sensor_t;

#endif

