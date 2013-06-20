/*
####################################################################################################
	name: NXT_Config.h
	Description: NXT device configration
	---
	update: 2013.06.13
####################################################################################################
*/

#ifndef _NXT_CONFIG_H_
#define _NXT_CONFIG_H_

#include "Common.h"

/* NXT sensor port configuration */
#define GYRO_SENSOR NXT_PORT_S1
#define SONAR_SENSOR NXT_PORT_S2
#define LIGHT_SENSOR NXT_PORT_S3
#define TOUCH_SENSOR NXT_PORT_S4

/* NXT motor port configuration */
#define TAIL_MOTOR NXT_PORT_A
#define LEFT_MOTOR NXT_PORT_B
#define RIGHT_MOTOR NXT_PORT_C

#define BT_PASS_KEY		"0753"	/* Pass Key for bluetooth */

#endif
