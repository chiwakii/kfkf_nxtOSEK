/*
####################################################################################################
	name: Actuator.h
	Description: アクチュエータ構造体
####################################################################################################
*/

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
	
	S8 forward;		/* 前後進 */
	S8 turn;		/* 旋回 */

	U16 black;				/* 黒色検知用 */
	U16 white;				/* 白色検知用 */
	U16 target_gray;		/* ライントレース目標 */
	U16 target_gray_base;	/* ライントレース目標(保存用) */

	S16 gray_offset;		/*  */
	int color_threshold;	/*  */

	U16 gyro_offset;		/* ジャイロオフセット */
	U16 gyro_offset_base;	/* ジャイロオフセット(保存用) */

	F32 P_gain;
	F32 I_gain;
	F32 D_gain;

	S16 dif;
	S16 pre_dif;
	S16 differential;
	F32 integral;
	
	U16 step_offset;

	U8 target_tail;
	S16 tail_dif;
	S16 tail_pre_dif;
	F32 TP_gain;

}Actuator_t;
//}Controller_t;

#endif
