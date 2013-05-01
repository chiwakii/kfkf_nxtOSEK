#include "bt.h"

void receive_BT(StateMachine_t *sm){
		S16 num_of_events,num_of_states;
		int packet_no=1;
		int ptr=0;
		S16 matrix[3000];
		S16 states[900];
		///////bluetooth
		//wait forÅ@bluetooth
		////////////////////////
		display_clear(0);
		display_goto_xy(0, 1);
		display_string("waiting for BT");
		display_update();

		while(1){
			ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
		
			if(bt_receive_buf[0]==packet_no && bt_receive_buf[1]==1){
				num_of_states = bt_receive_buf[2];
				num_of_events = bt_receive_buf[3];
			
				packet_no++;
				break;
			
			}
		}


		//values send by BT is assigned to the matrix
/*
		matrix = ( S16 * )malloc( sizeof( S16 )*((int)num_of_events*(int)num_of_states/14+1)*14);
		if( matrix == NULL ) {
			
			display_clear(0);
			display_goto_xy(0, 1);
			display_string("malloc error matrix");
			display_update();
			systick_wait_ms(10000);
		}
		
*/
		;
		while(1/*ptr+14 <num_of_states*num_of_events*/){
			int i=0;
			systick_wait_ms(100);
			
			 ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
			if(bt_receive_buf[1]==3)
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

		////values send by BT is assigned to the states
/*		
		states = ( S16 * )malloc( (( sizeof( State_t)*((int)num_of_states+1)/14)+1 )*14);
		if( states == NULL ) {
			
			display_clear(0);
			display_goto_xy(0, 1);
			display_string("malloc error state");
			display_update();
			systick_wait_ms(10000);
			
		}
*/		
		
		//state

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
			if(ptr!=0){
					 ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
			}
			if(bt_receive_buf[1]==255){
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
		sm->num_of_events = num_of_events;
		sm->num_of_states = num_of_states;
		sm->current_state = 0;
		sm->matrix = matrix;
		sm->states = (State_t *)states;






}
