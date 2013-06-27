/*
####################################################################################################
	name: Controller.h
	Description: コントローラ構造体
####################################################################################################
*/

#ifndef CONTROLLER_H_
#define CONTROLLER_H_


#include "Common.h"

typedef struct tag_Controller
{
	/* 光の状態 */
	/* 0:不明 / 1:白色上 / 2:黒色上 */
	U8 light_status;
	//S32 gray_marker_count;

	U8 target_distance;

	U8 timer_flag;
	U32 start_time;
	U32 target_time;

	U8 motor_counter_flag;
	int start_motor_count;
	int target_motor_count;

	U8 pivot_turn_flag;
	int start_pivot_turn_encoder_R;
	int target_pivot_turn_angle_R;

	U8 object_left_flag;
	U8 object_right_flag;
	U8 object_left_length;
	U8 object_right_length;
	U8 object_judge;

}Controller_t;


#endif /* CONTROLLER_H_ */
