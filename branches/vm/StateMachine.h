#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include <stdlib.h>

typedef struct tag_Event {
	S16 event_no;
} Event_t;

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
	S16 *matrix;
	State_t *states;
	S16 current_state;
} StateMachine_t;

StateMachine_t* StateMachine_create(S16 nevents, S16 nstates, S16 *_matrix, State_t *_states);

/**
 * public
 */
void StateMachine_sendEvent(StateMachine_t *stm, Event_t *event);

/**
 * public
 */
int StateMachine_execute(StateMachine_t *stm, Event_t *evevnt);

/**
 * private
 */
void StateMachine_action(State_t *state);

#endif
