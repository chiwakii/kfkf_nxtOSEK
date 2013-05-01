#include <math.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

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




