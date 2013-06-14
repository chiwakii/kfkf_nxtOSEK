#ifndef _CONTOROLLER_H_
#define _CONTOROLLER_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

typedef struct tag_Controller_{

	float speed;

	byte pid_on;
	byte wg_pid_on;

	byte balance_on;
	byte tail_on;
	
	S8 forward;
	S8 turn;

	//gyro sensor offset
	F32 gyro_offset;
	U16 gyro_offset_base;

	//light sensor
	U32 light_gray;
	U32 light_gray_base;
	U32 light_white;
	U32 light_black;

	S16 gray_offset;
	int color_threshold;
	F32 P_gain;
	F32 I_gain;
	F32 D_gain;
	F32 Tail_gain;

	F32 integral;
	
	U16 step_offset;
	U32 tail_ang;
	S16 tail_run_speed;
	S8 tail_speed_offset;

}Controller_t;

#endif
