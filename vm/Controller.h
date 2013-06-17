#ifndef _CONTOROLLER_H_
#define _CONTOROLLER_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "Common.h"

typedef enum PIDMode
{
	NO_MODE = 0,
	WB_PID = 1,
	WG_PID = 2
}PIDMode_e;

typedef enum StandMode
{
	NO_MODE = 0,
	BALANCE = 1,
	TAIL = 2
}StandMode_e;

typedef struct tag_Controller{

	//F32 speed;

	PIDMode_e PIDmode;
	//boolean pid_flag;
	//boolean wg_pid_flag;

	StandMode_e standmode;
	//boolean balance_flag;
	//boolean tail_flag;
	
	S8 forward;
	S8 turn;


	//light sensor
	//U32 light_gray;
	//U32 light_gray_base;
	//U32 light_white;
	//U32 light_black;

	S16 gray_offset;
	int color_threshold;
	F32 P_gain;
	F32 I_gain;
	F32 D_gain;

	S32 dif;
	S32 pre_dif;
	S32 differential;
	F32 integral;
	
	U16 step_offset;

	U32 target_tail;
	U32 tail_dif;
	U32 tail_pre_dif;
	F32 TP_gain;
	F32 TD_gain;

	//S16 tail_run_speed;
	//S8 tail_speed_offset;

}Controller_t;

#endif
