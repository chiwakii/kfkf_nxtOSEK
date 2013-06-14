/*
####################################################################################################
	name: kfkfModel.h
	Description: ??
	---
	update: 2013.06.13
####################################################################################################
*/


#ifndef _KFKFMODEL_ELEMENTS_H_
#define _KFKFMODEL_ELEMENTS_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include <stdlib.h>

//typedef struct tag_Event {
	//S16 event_no;
//} Event_t;

typedef struct tag_State {
	S16 state_no;
	S16 action_no;
	S16 value0;
	S16 value1;
	S16 value2;
	S16 value3;
} State_t;

typedef struct tag_StateMachine {
	S16 num_of_events;
	S16 num_of_states;
	S16 *events;
	State_t *states;
	S16 current_state;
} StateMachine_t;

//event manager
typedef struct tag_EventStatus {
#define LIGHT_STATUS_UNDEFINED 0
#define LIGHT_STATUS_WHITE 1
#define LIGHT_STATUS_BLACK 2
#define TOUCH_STATUS_NOTPRESSED 0
#define TOUCH_STATUS_PRESSED 1
#define TIMER_PROCESSING 1
#define STARTED 1
	int light_status;
	int gray_marker_count;
	int touch_status;
	int start_motor_count;
	int motor_count;
	int start_timer;
	int timer_flag;
	S16 limit_time;
	byte started;
	int circling_start_encoder_R;
	int circling_target_angle_R;
	byte circling_on;
	byte bottle_left_length;
	byte bottle_right_length;
	byte bottle_judge;
	S8 num_to_loop;
	S8 loop_count;

} EventStatus_t;




#endif
