/*
####################################################################################################
	name: kfkfModel.c
	Description: "ETロボコンkfkf"用プログラム
	---
	update: 2013.06.22
####################################################################################################
*/

#include <stdlib.h>

#include "kfkfModel.h"

/*
===============================================================================================
	Definition
===============================================================================================
*/
/* Bluetooth用バッファのサイズ */
#define BT_RCV_BUF_SIZE 32
/* event一時保管用配列のサイズ */
#define RESERVED_EVENT_SIZE 3000
/* state一時保管用配列のサイズ */
#define RESERVED_STATES_SIZE 900

/* ETロボコンkfkf管理用構造体 */
typedef struct tag_StateMachine {
	S16 num_of_events;
	S16 num_of_states;
	U16 *events;
	State_t *states;
	S16 current_state;
	U8 *event_flag;
} StateMachine_t;

/*
===============================================================================================
	Variables
===============================================================================================
*/
/* Bluetooth用バッファ */
S16 bt_receive_buf[BT_RCV_BUF_SIZE];
/* ETロボコンkfkf管理用変数 */
StateMachine_t g_StateMachine;

/*
===============================================================================================
	Functions
===============================================================================================
*/

/*
===============================================================================================
	name: ReceiveBT
	Description: ETロボコンkfkfの受信処理
	Parameter: no
	Return Value: no
===============================================================================================
*/
/*------------------*/
/* グローバル変数	*/
/*------------------*/
static U16 g_PacketCnt;
static U16 g_Eptr;
static U16 g_Sptr;

/*------------------*/
/* 一時保管用配列	*/
/*------------------*/
static S16 events[RESERVED_EVENT_SIZE];
static S16 states[RESERVED_STATES_SIZE];

/*------------------*/
/*	  関数			*/
/*------------------*/
U8 ReceiveBT(void){
	
    U16 i = 0;
    U16 j = 0;
    U8 comm_end = 0;

    ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);

/*======================================================================================*/
/*  最後の受信パケットを受信したら														*/
/*======================================================================================*/
    if( bt_receive_buf[1] == 255 )
    {
    	/*--------------------------*/
		/*	eventの割り当て			*/
    	/*--------------------------*/
        g_StateMachine.events = (U16 *)calloc( g_Eptr, sizeof(U16) );
        if( g_StateMachine.events == NULL )
        {
            display_clear(0);
            display_goto_xy(0, 1);
            display_string("Pre:Bluetooth");
            display_goto_xy(0, 2);
            display_string("Malloc Err:event");
            display_update();
            ecrobot_sound_tone(880, 50, 30);
        }
        else
        {
        	for(i=0;i<g_Eptr;i++)
        	{
        		g_StateMachine.events[i] = (U16)events[i];
        	}

        	comm_end++;
        }

    	/*--------------------------*/
		/*	stateの割り当て			*/
    	/*--------------------------*/
        g_StateMachine.states = (State_t *)calloc( g_StateMachine.num_of_states, sizeof(State_t) );
        if( g_StateMachine.states == NULL )
        {
            display_clear(0);
            display_goto_xy(0, 1);
            display_string("Pre:Bluetooth");
            display_goto_xy(0, 2);
            display_string("Malloc Err:state");
            display_update();
            ecrobot_sound_tone(880, 50, 30);
        }
        else
        {

        	i = 0;
        	for(j=0;j<g_StateMachine.num_of_states;j++)
        	{
        		g_StateMachine.states[j].state_no = states[i];
        		g_StateMachine.states[j].action_no = (U16)states[i+1];
       			g_StateMachine.states[j].value0 = states[i+2];
       			g_StateMachine.states[j].value1 = states[i+3];
       			g_StateMachine.states[j].value2 = states[i+4];
       			g_StateMachine.states[j].value3 = states[i+5];
       			i = i + 6;
       		}

        	comm_end++;
        }

        if(comm_end >= 2)
        {
        	comm_end = 1;

        	g_StateMachine.current_state = 0;

        	g_StateMachine.event_flag = (U8 *)calloc( g_StateMachine.num_of_events, sizeof(U8) );
        	clearEvent();

        	g_PacketCnt = 0;
        }
    }

/*======================================================================================*/
/*  受信パケットの種類が"1"																*/
/*======================================================================================*/
    if(bt_receive_buf[0] == g_PacketCnt && bt_receive_buf[1] == 1)
    {
    	g_StateMachine.num_of_states = bt_receive_buf[2];
    	g_StateMachine.num_of_events = bt_receive_buf[3];
		
    	g_PacketCnt++;
    }

/*======================================================================================*/
/*  受信パケットの種類が"2"																*/
/*======================================================================================*/
    if(bt_receive_buf[0] == g_PacketCnt && bt_receive_buf[1] == 2)
    {
    	for(i=2;i<16;i++)
    	{
    		//*(events + g_Eptr) = *(bt_receive_buf + i);
    		events[g_Eptr] = bt_receive_buf[i];
    		g_Eptr++;
    	}

    	g_PacketCnt++;
    }

/*======================================================================================*/
/*  受信パケットの種類が"3"																*/
/*======================================================================================*/
    if(bt_receive_buf[0] == g_PacketCnt && bt_receive_buf[1] == 3)
    {
    	for(i=2;i<16;i++)
    	{
    		//*(states + g_Sptr) = *(bt_receive_buf + i);
    		states[g_Sptr] = bt_receive_buf[i];
    		g_Sptr++;
    	}

    	g_PacketCnt++;
    }
    
	/*----------------------------------*/
	/*	パケット受信の終了を知らせる	*/
    /*	------------------------------	*/
    /*	終了してない:0 / 終了した:1		*/
	/*----------------------------------*/
    return comm_end;
}


/*
===============================================================================================
	name: getCurrentStateNum
	Description: 現在の状態の番号をわたす
	Parameter: no
	Return Value: 状態の番号(S16)
===============================================================================================
*/
S16 getCurrentStateNum()
{
	return g_StateMachine.current_state;
}

/*
===============================================================================================
	name: getCurrentState
	Description: 現在の状態をわたす
	Parameter: no
	Return Value: 状態(State_t)
===============================================================================================
*/
State_t getCurrentState(void)
{
	return g_StateMachine.states[g_StateMachine.current_state];
}

/*
===============================================================================================
	name: setEvent
	Description: 発生したイベントのフラグを立てる
	Parameter: イベントの種類(U16)
	Return Value: no
===============================================================================================
*/
void setEvent(U16 event_id)
{
	g_StateMachine.event_flag[(U16)event_id] = 1;
}

/*
===============================================================================================
	name: clearEvent
	Description: イベントフラグの初期化
	Parameter: no
	Return Value: no
===============================================================================================
*/
void clearEvent(void)
{
	U16 i = 0;

	for(i=0;i<g_StateMachine.num_of_events;i++){
		g_StateMachine.event_flag[i] = 0;
	}
}

/*
===============================================================================================
	name: setNextState
	Description: 遷移先の状態番号をセットする
	Parameter: no
	Return Value: no
===============================================================================================
*/
void setNextState(void) {
	S8 next_state = -1;
	S16 i = 0;

	for( i=0;i<g_StateMachine.num_of_events;i++ ){

		if( g_StateMachine.event_flag[i] == 1)
		{
			if( g_StateMachine.events[i + g_StateMachine.current_state * g_StateMachine.num_of_events] != -1 )
			{
				next_state = g_StateMachine.events[i + g_StateMachine.current_state * g_StateMachine.num_of_events];
			}
		}
	}

	if( next_state != -1 )
	{
		g_StateMachine.current_state = next_state;
	}

    display_clear(0);
	display_goto_xy(0, 0);
	display_string("Act:");
	display_int(g_StateMachine.states[g_StateMachine.current_state].action_no,4);
    display_update();
	clearEvent();

}

/*
===============================================================================================
	name: BluetoothStart
	Description: Bluetooth送信機の"START"ボタンの押下を知らせる
	Parameter: no
	Return Value: 押下していない:0 / 押下した:1 (S8)
===============================================================================================
*/
S8 BluetoothStart(void)
{
	ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);

	if( bt_receive_buf[1] == 254 && bt_receive_buf[0] != g_PacketCnt )
	{
		g_PacketCnt = bt_receive_buf[0];
		return 1;
	}

	return 0;
}


/*
===============================================================================================
	name: InitKFKF
	Description: ETロボコンkfkfに関する変数の初期化
	Parameter: no
	Return Value: no
===============================================================================================
*/
void InitKFKF(void)
{
	int i = 0;

	/*--------------------------*/
	/*	StateMachine			*/
	/*--------------------------*/
	free( g_StateMachine.events );

	free( g_StateMachine.states );

	free( g_StateMachine.event_flag );

	g_StateMachine.num_of_events = 0;
	g_StateMachine.num_of_states = 0;
	g_StateMachine.current_state = 0;


	/*--------------------------*/
	/*	others					*/
	/*--------------------------*/
	g_PacketCnt = 1;
	g_Eptr = 0;
	g_Sptr = 0;

	for(i=0;i<RESERVED_EVENT_SIZE;i++)
	{
		events[i] = 0;
	}
	for(i=0;i<RESERVED_STATES_SIZE;i++)
	{
		states[i] = 0;
	}
}

