#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "Common.h"


typedef struct tag_Sensor
{
	U16 light;
	U16 light_ave;

	U16 gyro;

	U8 touch;
	S32 distance;
	int count_left;
	int count_right;
	int count_tail;
	U16 battery;

	U8 bottle_is_left;
	U8 bottle_is_right;

	U8 BTstart;
}Sensor_t;

#endif

