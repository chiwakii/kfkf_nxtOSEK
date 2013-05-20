/**
 *******************************************************************************
 **	FILE NAME : nxt_config.h
 **
 **	ABSTRUCT  : NXT device configration
 *******************************************************************************
 **/

#ifndef _NXT_CONFIG_H_
#define _NXT_CONFIG_H_

#include "ecrobot_interface.h"

/* NXT sensor port configuration */
#define GYRO_SENSOR NXT_PORT_S1
#define SONAR_SENSOR NXT_PORT_S2
#define LIGHT_SENSOR NXT_PORT_S3
#define TOUCH_SENSOR NXT_PORT_S4

/* NXT motor port configuration */
#define TAIL_MOTOR NXT_PORT_A
#define LEFT_MOTOR NXT_PORT_B
#define RIGHT_MOTOR NXT_PORT_C

#endif
