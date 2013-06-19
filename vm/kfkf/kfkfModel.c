/*
####################################################################################################
	name: kfkfModel.c
	Description: ???
	---
	update: 2013.06.13
####################################################################################################
*/

#include <stdlib.h>
#include "kfkfModel.h"

typedef struct tag_StateMachine {
	S16 num_of_events;
	S16 num_of_states;
	EvtType_e *events;
	State_t *states;
	S16 current_state;
} StateMachine_t;

/* NXT Bluetooth configuration */
#define BT_RCV_BUF_SIZE 32		/* Buffer size for bluetooth */

#define RESERVED_MATRIX_SIZE 3000
#define RESERVED_STATES_SIZE 900

S16 bt_receive_buf[BT_RCV_BUF_SIZE];	/* bluetooth */

static StateMachine_t g_StateMachine;

void InitStateMachine(void)
{
	g_StateMachine.num_of_events = 0;
	g_StateMachine.num_of_states = 0;
	g_StateMachine.current_state = 0;

	free( g_StateMachine.events );
	g_StateMachine.events = NULL;

	free( g_StateMachine.states );
	g_StateMachine.states = NULL;
}

/*
===============================================================================================
	name: receive_BT
	Description: ??
	Parameter: no
	Return Value: no
	---
	update: 2013.06.13
===============================================================================================
*/
void receive_BT(void){
	
	S16 matrix[RESERVED_MATRIX_SIZE];
	S16 states[RESERVED_STATES_SIZE];
	
    int packet_no = 1,	//packet number.
    int ptr = 0;

    display_clear(0);
    display_goto_xy(0, 1);
    display_string("BT Communication");
    display_update();
    
// packet type:1 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    while(1){
    	// Receive first packet.
        ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
		
        if(bt_receive_buf[0] == packet_no && bt_receive_buf[1] == 1)
        {
        	g_StateMachine.num_of_states = bt_receive_buf[2];
        	g_StateMachine.num_of_events = bt_receive_buf[3];
			
            packet_no++;
            break;
        }
    }

// packet type:2 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    display_clear(0);
    display_goto_xy(0, 1);
    display_string("BT Communication");
    display_goto_xy(1, 1);
    display_string("receive packet:event");
    display_update();

    ptr = 0;

    while(1/*ptr+14 <num_of_states*num_of_events*/)
    {
        int i = 0;
        systick_wait_ms(100);
        
        ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);

        if(bt_receive_buf[1] == 3)
        {
            break;
        }

        if(bt_receive_buf[0] == packet_no && bt_receive_buf[1] == 2)
        {
            for(i=2;i<16;i++)
            {
                *(matrix+ptr) = *(bt_receive_buf+i);
                ptr++;
            }
            packet_no++;
        }
    }

    g_StateMachine.events = (EvtType_e *)malloc(ptr);
    if(g_StateMachine.events == NULL)
    {
        display_clear(0);
        display_goto_xy(0, 1);
        display_string("BT Communication");
        display_goto_xy(1, 1);
        display_string("Malloc Error:event");
        display_update();
    }

    for(i=0;i<ptr;i++)
    {
    	g_StateMachine.events[i] = (EvtType_e)matrix[i];
    }
    

// packet type:3 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    display_clear(0);
    display_goto_xy(0, 1);
    display_string("BT Communication");
    display_goto_xy(1, 1);
    display_string("end packet:event");
    display_goto_xy(2, 1);
    display_string("receive packet:state");
    display_update();

    ptr = 0;

    g_StateMachine.states = (State_t *)malloc(g_StateMachine.num_of_states);
    if(g_StateMachine.states == NULL)
    {
        display_clear(0);
        display_goto_xy(0, 1);
        display_string("BT Communication");
        display_goto_xy(1, 1);
        display_string("end packet:event");
        display_goto_xy(2, 1);
        display_string("Malloc Error:state");
        display_update();
    }

    while(1/*ptr+14 <(2+4)*num_of_states*/){

        int i=0;
        systick_wait_ms(100);

        if(ptr!=0)
        {
            ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
        }
        if(bt_receive_buf[1] == 255)
        {
            break;
        }
        
        if(bt_receive_buf[0] == packet_no && bt_receive_buf[1] == 3)
        {
            for(i=2;i<16;i++)
            {
                *(states+ptr) = *(bt_receive_buf+i);
                ptr++;
            }
            packet_no++;
        }
    }
    
    for(i=0;i<ptr;i=i+6)
    {
    	g_StateMachine[i].states.state_no = states[i];
    	g_StateMachine[i+1].states.action_no = (ActType_e)states[i+1];
    	g_StateMachine[i+2].states.value0 = states[i+2];
    	g_StateMachine[i+3].states.value1 = states[i+3];
    	g_StateMachine[i+4].states.value2 = states[i+4];
    	g_StateMachine[i+5].states.value3 = states[i+5];
    }

    display_clear(0);
    display_goto_xy(0, 1);
    display_string("BT Communication");
    display_goto_xy(1, 1);
    display_string("end packet:event");
    display_goto_xy(2, 1);
    display_string("end packet:state");
    display_update();

    g_StateMachine.current_state = 0;

}

/*
===============================================================================================
	name: get_CurrentState
	Description: ??
	Parameter: no
	Return Value: g_StateMachine.current_state
	---
	update: 2013.06.13
===============================================================================================
*/
S16 getCurrentState()
{
	return g_StateMachine.current_state;
}

/*
===============================================================================================
	name: set_NextState(befote:sendevent)
	Description: ??
	Parameter: event_id:S8
	Return Value:
	---
	update: 2013.06.13
===============================================================================================
*/
#define NO_STATE -1

State_t setNextState(EvtType_e event_id) {
	S8 next_state = NO_STATE;
	//S16 i = 0;
	State_t state = {-1,-1,0,0,0,0};

	next_state = g_StateMachine.events[event_id + g_StateMachine.current_state * g_StateMachine.num_of_events];

	if(next_state != NO_STATE)
	{
		g_StateMachine.current_state = next_state;
		state = g_StateMachine.states[g_StateMachine.current_state];
	}

/*
	for(i = 0;i < g_StateMachine.num_of_states;i++) {
		if(i == g_StateMachine.current_state) {
			ControllerSet(&g_StateMachine.states[i]);
			return 1;
		}
	}
*/
	return state;
}

/*
===============================================================================================
	name: BluetoothStart
	Description: ??
	Parameter: no
	Return Value: S8
	---
	update: 2013.06.17
===============================================================================================
*/
S8 BluetoothStart(void)
{
	U8 btstart = OFF;

	ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
	if(bt_receive_buf[1] == 254 )
	{
		btstart = ON;
	}
	else
	{
		btstart = OFF;
	}

	return btstart;
}
