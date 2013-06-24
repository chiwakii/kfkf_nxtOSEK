/*
####################################################################################################
	name: logger.h
	Description: ??
	---
	update: 2013.06.13
####################################################################################################
*/
#ifndef _LOGGER_H_
#define _LOGGER_H_

typedef enum LogType
{
	LOG_NO = -1,
	LOG_STATE = 0,
	LOG_TURN = 1,
	LOG_PWM = 2,
	LOG_TARGET_ANGLE = 3,
	LOG_LIGHT_MIN = 4,
	LOG_MOTOR_COUNT = 5,
	LOG_SONAR = 6,
	LOG_DT = 7,
	LOG_BALANCE_TAIL = 8,
	LOG_LOOP = 9
}LogType_e;

typedef struct tag_Logger{

	enum LogType type;

} Logger_t;


#endif
