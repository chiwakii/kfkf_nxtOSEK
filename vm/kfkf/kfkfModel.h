/*
####################################################################################################
	name: kfkfModel.h
	Description: ??
	---
	update: 2013.06.13
####################################################################################################
*/

#ifndef _KFKFMODEL_H_
#define _KFKFMODEL_H_


// Receive kfkf model data.
void receive_BT(StateMachine_t statemachine);
// Return current kfkf model state.
S16 get_CurrentState();
// ??
int set_NextState(S8 event_id);

#endif
