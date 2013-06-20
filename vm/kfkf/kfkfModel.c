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

/*
===============================================================================================
	Definition
===============================================================================================
*/
/* Buffer size for Bluetooth */
#define BT_RCV_BUF_SIZE 32
/* The number of event */
#define RESERVED_EVENT_SIZE 3000
/* The number of state */
#define RESERVED_STATES_SIZE 900

/* Definition of Structure of state machine */
typedef struct tag_StateMachine {
	S16 num_of_events;
	S16 num_of_states;
	EvtType_e *events;
	State_t *states;
	S16 current_state;
	U8 *event_array;
} StateMachine_t;

/*
===============================================================================================
	Variables
===============================================================================================
*/
/* Buffer for Bluetooth */
S16 bt_receive_buf[BT_RCV_BUF_SIZE];
/* State Machine for kfkf Model */
static StateMachine_t g_StateMachine;

/*
===============================================================================================
	Functions
===============================================================================================
*/
/*
===============================================================================================
	name: receive_BT
	Description: ??
	Parameter: no
	Return Value: no
===============================================================================================
*/
static U16 g_PacketCnt = 1;
static U16 g_Eptr = 0;
static U16 g_Sptr = 0;

static S16 events[RESERVED_EVENT_SIZE];
static S16 states[RESERVED_STATES_SIZE];

U8 ReceiveBT(void){
	
    U16 i = 0;
    U16 j = 0;
    U8 comm_end = OFF;

    ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);

//-------------------------------------------------------------------------------------
//  If packet end
//-------------------------------------------------------------------------------------
    if(bt_receive_buf[1] == 255)
    {
		//==========================================
		//	Allocation for events
		//==========================================
        g_StateMachine.events = (EvtType_e *)malloc( sizeof(EvtType_e) * g_Eptr );
        if(g_StateMachine.events == NULL)
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
        		g_StateMachine.events[i] = (EvtType_e)events[i];
        	}

        	comm_end++;
        }


		//==========================================
		//	Allocation for states
		//==========================================
        g_StateMachine.states = (State_t *)malloc( sizeof(State_t) * g_StateMachine.num_of_states );
        if(g_StateMachine.states == NULL)
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
        		g_StateMachine.states[j].action_no = (ActType_e)states[i+1];
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
        	comm_end = ON;

        	g_StateMachine.current_state = 0;

        	g_StateMachine.event_array = (U8 *)malloc( sizeof(U8) * g_StateMachine.num_of_events );
        	clearEvent();
        }
    }

//-------------------------------------------------------------------------------------
//  If packet type:1
//-------------------------------------------------------------------------------------
    if(bt_receive_buf[0] == g_PacketCnt && bt_receive_buf[1] == 1)
    {
    	g_StateMachine.num_of_states = bt_receive_buf[2];
    	g_StateMachine.num_of_events = bt_receive_buf[3];
		
    	g_PacketCnt++;
    }

//-------------------------------------------------------------------------------------
//  If packet type:2
//-------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------
//  If packet type:3
//-------------------------------------------------------------------------------------
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
    

	//==========================================
	//	End of BT communication
    // -------
    // Not END:OFF / END:ON
	//==========================================
    return comm_end;
}


/*
===============================================================================================
	name: getCurrentStateNum
	Description: ??
	Parameter: no
	Return Value: g_StateMachine.current_state
===============================================================================================
*/
S16 getCurrentStateNum()
{
	return g_StateMachine.current_state;
}

State_t getCurrentState(void)
{
	return g_StateMachine.states[g_StateMachine.current_state];
}


void setEvent(EvtType_e event_id)
{
	g_StateMachine.event_array[(U16)event_id] = ON;
}

void clearEvent(void)
{
	U16 i = 0;

	for(i=0;i<g_StateMachine.num_of_events;i++){
		g_StateMachine.event_array[i] = OFF;
	}
}

/*
===============================================================================================
	name: set_NextState(befote:sendevent)
	Description: ??
	Parameter: event_id:S8
	Return Value:
===============================================================================================
*/
#define NO_STATE -1

void setNextState(void) {
	S8 next_state = NO_STATE;
	S16 i = 0;

	for(i=0;i<g_StateMachine.num_of_events;i++){

		if( g_StateMachine.event_array[i] == ON)
		{
			next_state = g_StateMachine.events[i + g_StateMachine.current_state * g_StateMachine.num_of_events];

			if(next_state != NO_STATE)
			{
				g_StateMachine.current_state = next_state;
			}

		}

	}

    display_clear(0);
	display_goto_xy(0, 0);
	display_string("Prep:TRUE");
	display_int(g_StateMachine.states[g_StateMachine.current_state].action_no,4);
    display_update();
	clearEvent();

}

/*
===============================================================================================
	name: BluetoothStart
	Description: ??
	Parameter: no
	Return Value: S8
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


/*
===============================================================================================
	name: InitKFKF
	Description: ??
	Parameter: no
	Return Value: no
===============================================================================================
*/
void InitKFKF(void)
{
	U16 i = 0;

	//==========================================
	//	Initialization of StateMachine
	//==========================================
	free( g_StateMachine.events );
	//g_StateMachine.events = NULL;

	free( g_StateMachine.states );
	//g_StateMachine.states = NULL;

	for(i=0;i<g_StateMachine.num_of_events;i++){
		g_StateMachine.event_array[i] = OFF;
	}
	free( g_StateMachine.event_array );

	g_StateMachine.num_of_events = 0;
	g_StateMachine.num_of_states = 0;
	g_StateMachine.current_state = 0;

	//==========================================
	//	Initialization of others
	//==========================================
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
