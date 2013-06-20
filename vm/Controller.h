#ifndef _CONTOROLLER_H_
#define _CONTOROLLER_H_

#include "Common.h"

typedef enum _PIDMode
{
	NO_PID_MODE = 0,
	WB_PID = 1,
	WG_PID = 2
}PIDMode_e;

typedef enum _StandMode
{
	NO_STAND_MODE = 0,
	BALANCE = 1,
	TAIL = 2
}StandMode_e;

typedef struct tag_Controller
{
	PIDMode_e PIDmode;
	//U8 pid_flag;
	//U8 wg_pid_flag;

	StandMode_e StandMode;
	//U8 balance_flag;
	//U8 tail_flag;
	
	S8 forward;
	S8 turn;

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

	U8 target_tail;
	S16 tail_dif;
	S16 tail_pre_dif;
	F32 TP_gain;
	F32 TD_gain;

}Controller_t;

#endif
