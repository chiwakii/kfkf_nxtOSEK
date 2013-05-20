#include "StateMachine.h"

StateMachine_t* StateMachine_create(S16 nevents, S16 nstates, S16 *_matrix, State_t *_states) {
	StateMachine_t *stm;
	int i=0;
	stm = (StateMachine_t*)malloc(sizeof(StateMachine_t));
	stm->num_of_events = nevents;
	stm->num_of_states = nstates;
	stm->matrix = _matrix;
	stm->states = _states;
	stm->current_state = 1;
	return stm;
}


void StateMachine_sendEvent(StateMachine_t *stm, Event_t *event) {
	StateMachine_execute(stm, event);
}

int StateMachine_execute(StateMachine_t *stm, Event_t *event) {
	S16 next_state = -1;
	S16 i = 0;
	next_state = stm->matrix[event->event_no + stm->current_state * stm->num_of_events];
//	next_state = stm->matrix[1 + stm->current_state * stm->num_of_events];
	
	if(next_state == -1) return 0;
	stm->current_state = next_state;
	
	for(i = 0;i < stm->num_of_states;i++) {
		if(i == stm->current_state) {
			StateMachine_action(&stm->states[i]);
			return 1;
		}
	}
	return 2;
}


/**
 *action_noに対応するアクションを記述する
 *//*
void StateMachine_action(State_t state) {
	switch(state.action_no) {
		case 0:
			//set_motor_pwr(MOTOR_RIGHT, state.value0)
			//set_motor_pwr(MOTOR_LEFT, state.value1)
		
			break;
		case 1:
			//set_motor_pwr(MOTOR_RIGHT, 100)
			//set_motor_pwr(MOTOR_LEFT, 100)

	ecrobot_sound_tone(100,10,50);
			break;
		case 2:
			//forward();	//balancer
			break;
		case 3:
			//turn();	//balancer
			break;
		case 4:
			//do_linetracer();	//pid
			break;
	}
}
*/

