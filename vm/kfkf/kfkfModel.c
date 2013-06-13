/*
####################################################################################################
	name: kfkfModel.c
	Description: ???
	---
	update: 2013.06.13
####################################################################################################
*/

#include "kfkfModel_Elements.h"
#include "kfkfModel.h"


/* NXT Bluetooth configuration */
#define BT_RCV_BUF_SIZE 32		/* Buffer size for bluetooth */

#define RESERVED_MATRIX_SIZE 3000
#define RESERVED_STATES_SIZE 900

S16 bt_receive_buf[BT_RCV_BUF_SIZE];	/* bluetooth */

static StateMachine_t statemachine;

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
void receive_BT(/* StateMachine_t statemachine*/){
	
	S16 matrix[RESERVED_MATRIX_SIZE];
	S16 states[RESERVED_STATES_SIZE];
	
    short num_of_events,num_of_states;
    int packet_no = 1,	//packet number.
    int ptr;
    ///////bluetooth
    //wait for　bluetooth
    ////////////////////////
    display_clear(0);
    display_goto_xy(0, 1);
    display_string("BT Communication");
    display_update();
    
    while(1){
    	// Receive first packet.
        ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
		
        if(bt_receive_buf[0] == packet_no && bt_receive_buf[1] == 1)
        {
        	statemachine.num_of_states = bt_receive_buf[2];
        	statemachine.num_of_events = bt_receive_buf[3];
            //num_of_states = bt_receive_buf[2];
            //num_of_events = bt_receive_buf[3];
			
            packet_no++;
            break;
        }
    }

    ptr = 0;

    while(1/*ptr+14 <num_of_states*num_of_events*/){
        int i=0;
        systick_wait_ms(100);
        
        ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
        if(bt_receive_buf[1] == 3)
        {
            break;
        }
        
        display_clear(0);
        display_goto_xy(0, 1);
        display_string("E0_1");
        display_int(packet_no,4);
        display_int(ptr,6);
        display_update();
        
        if(bt_receive_buf[0]==packet_no && bt_receive_buf[1]==2){
            for(i=2;i<16;i++){
                *(matrix+ptr) = *(bt_receive_buf+i);
                ptr++;
            }
            packet_no++;
        }
        
        
    }
	
    
    display_clear(0);
    display_goto_xy(0, 1);
    display_string("E0_1");
    display_int(packet_no,4);
    display_int(ptr,6);
    display_update();
    
    display_clear(0);
    display_goto_xy(0, 1);
    display_string("matrix end");
    display_update();

    ptr=0;
    while(1/*ptr+14 <(2+4)*num_of_states*/){
        int i=0;
        display_clear(0);
        display_goto_xy(0, 1);
        display_string("S0_1");
        display_int(packet_no,4);
        display_int(ptr,6);
        display_update();
        systick_wait_ms(100);

        if(ptr!=0)
        {
            ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
        }
        if(bt_receive_buf[1]==255)
        {
            break;
        }
        
        
        if(bt_receive_buf[0]==packet_no && bt_receive_buf[1]==3){
            for(i=2;i<16;i++){
                *(states+ptr) = *(bt_receive_buf+i);
                ptr++;
            }
            packet_no++;
        }
    }
    
    //statemachine.num_of_events = num_of_events;
    //statemachine.num_of_states = num_of_states;
    statemachine.current_state = 0;
    statemachine.matrix = matrix;
    statemachine.states = (State_t *)states;

}

/*
===============================================================================================
	name: get_CurrentState
	Description: ??
	Parameter: no
	Return Value: statemachine.current_state
	---
	update: 2013.06.13
===============================================================================================
*/
S16 get_CurrentState()
{
	return statemachine.current_state;
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
int set_NextState(S8 event_id) {
	S16 next_state = -1;
	S16 i = 0;


	next_state = statemachine.matrix[event_id + statemachine.current_state * statemachine.num_of_events];

	if(next_state == -1) return 0;
	ecrobot_sound_tone(500,10,100);

	statemachine.current_state = next_state;

	for(i = 0;i < statemachine.num_of_states;i++) {
		if(i == statemachine.current_state) {
			StateMachine_action(&statemachine.states[i]);
			return 1;
		}
	}
	return 2;
}
