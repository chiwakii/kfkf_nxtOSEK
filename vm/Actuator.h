#ifndef _CONTOROLLER_H_
#define _CONTOROLLER_H_

#include "Common.h"

typedef struct tag_Actuator
{
	/* Traceモード */
	/* 0:トレースなし / 1:白色と黒色の境目をトレース / 2:白色と灰色の境目をトレース */
	U8 TraceMode;

	/* 倒立モード */
	/* 0:倒立なし / 1:バランスとって立つ / 2:しっぽで立つ */
	U8 StandMode;
	
	S8 forward;
	S8 turn;

	U16 black;
	U16 white;
	U16 target_gray;
	U16 target_gray_base;

	S16 gray_offset;
	int color_threshold;

	U16 gyro_offset;
	U16 gyro_offset_base;

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

}Actuator_t;
//}Controller_t;

#endif
