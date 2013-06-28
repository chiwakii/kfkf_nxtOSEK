/*
####################################################################################################
	name: kfkfModel.h
	Description: "ETロボコンkfkf"用プログラム
	---
	update: 2013.06.22
####################################################################################################
*/

#ifndef _KFKFMODEL_H_
#define _KFKFMODEL_H_

#include "../Common.h"


/*
===============================================================================================
	列挙型の定義
===============================================================================================
*/
/*--------------------------*/
/* 	ETロボコンkfkfのaction	*/
/*--------------------------*/
enum ActType
{
	DO_NOTHING = 0,
	BALANCE_STOP = 1,
	BALANCE_LINETRACE = 2,
	CHANGE_GRAY = 4,
	TAIL_RUN_FREEDOM = 5,
	TIMER_SET = 8,
	MOTOR_SET = 9,
	PID_SET = 10,
	STEP_OFFSET_SET = 12,
	RAISE_TAIL = 13,
	TAIL_LINETRACE = 14,
	PIVOT_TURN = 15,
	SELECT_LOGTYPE = 16,
	GRAY_MARKER_OFFSET = 17,
	BALANCE_RUN_FREEDOM = 18,
	SONAR_SET = 20,
	SERACH_BOTTLE_RIGHT = 21,
	SEARCH_BOTTLE_LEFT = 22,
	SEARCH_BOTTLE_END = 23,
	SEARCH_BOTTLE_JUDGE = 24
};

/*--------------------------*/
/* 	ETロボコンkfkfのevent	*/
/*--------------------------*/
enum EvtType
{
	NO_TRANSITION = 0,
	AUTO = 1,
	TOUCH = 2,
	WHITE = 3,
	BLACK = 4,
	GRAY_MARKER = 5,
	STEP = 6,
	SONAR = 9,
	TIMER = 10,
	MOTOR_COUNT = 11,
	BT_START = 12,
	PIVOT_TURN_END = 13,
	BOTTLE_LEFT = 14,
	BOTTLE_RIGHT = 15
};

/*
===============================================================================================
	構造体の定義
===============================================================================================
*/
/*--------------------------*/
/* ETロボコンkfkf用構造体	*/
/*--------------------------*/
typedef struct tag_State
{
	S16 state_no;
	U16 action_no;
	S16 value0;
	S16 value1;
	S16 value2;
	S16 value3;
}State_t;


//--------------------------------------------------------------------
//	functions
//--------------------------------------------------------------------
//
void InitKFKF(void);
// Receive kfkf model data.
U8 ReceiveBT(void);
// Return current kfkf model state.
S16 getCurrentStateNum(void);
//
State_t getCurrentState(void);
//
void setEvent(U16 event_id);
//
void clearEvent(void);
// ??
void setNextState(void);
//
S8 BluetoothStart(void);

#endif
