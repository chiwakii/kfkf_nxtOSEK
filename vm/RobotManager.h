/*
####################################################################################################
	name: RobotManaget.h
	Description: ???
	---
	update: 2013.06.13
####################################################################################################
*/

#ifndef _ROBOTMANAGER_H_
#define _ROBOTMANAGER_H_

/*
=================================
	prototype declaration
=================================
*/
//
void InitNXT();
//
void EventSensor();
//
void ControllerSet(State_t* state);


void gyro_calibration();
void calibration(int *black,int *white,int *gray);
void tail_run_turn2pwm(S16 _tail_run_speed ,float _turn ,S8 *_pwm_L, S8 *_pwm_R);
S16 calc_angle2encoder(S16 angle);
S8 calc_variance(U16 *buf,int _len);

#endif /* ROBOTMANAGER_H_ */
