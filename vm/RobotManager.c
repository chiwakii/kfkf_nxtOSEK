/*
####################################################################################################
	name: ManageActivity.c
	Description: ??
	---
	update: 2013.06.13
####################################################################################################
*/

#include <math.h>
#include "ManagementActivity.h"
#include "kfkf/kfkfModel.h"


/*
===============================================================================================
	name: InitNXT
	Description: ??
 	Parameter: no
	Return Value: no
 	---
	update: 2013.06.13
===============================================================================================
*/
void InitNXT()
{
	balance_init();
	nxt_motor_set_count(RIGHT_MOTOR,0);
	nxt_motor_set_count(LEFT_MOTOR,0);

	sensor.light = ecrobot_get_light_sensor(LIGHT_SENSOR);
	sensor.gyro= ecrobot_get_gyro_sensor(GYRO_SENSOR);
	sensor.touched = ecrobot_get_touch_sensor(TOUCH_SENSOR);
	sensor.sonar = ecrobot_get_sonar_sensor(SONAR_SENSOR);
	//sensor.distance = (S32)getDistance();

	sensor.prev_light_value = sensor.light;

	balance_init();


    //receive_BT(statemachine);   //receiveing BT


	//gyro_calibration();
	//calibration(&sensor.black,&sensor.white,&sensor.gray);

	controller.speed = 0;
	controller.forward_power=0;
	controller.turn=0;
	controller.balance_on=0;
	controller.pid_on=0;
	controller.wg_pid_on=0;
	controller.tail_on=0;
	controller.tail_ang=0;
	controller.tail_run_speed=0;
		//controller.gyro_offset=610;
	controller.step_offset=10000;
	controller.gray_offset = 10000;
	controller.color_threshold = 660;
	controller.P_gain=1.0;
	controller.I_gain=1.0;
	controller.D_gain=1.0;
	controller.right_motor_rate=1.0;
	controller.left_motor_rate=1.0;

	sensor.light_min=1000;
	sensor.light_max=0;
	sensor.bottle_is_left=0;
	sensor.bottle_is_right=0;

	sensor.threshold_gray=sensor.gray;
	controller.color_threshold = sensor.gray;
	sensor.prev_light_value=controller.color_threshold;

	for(int m=0 ; m < sensor.LIGHT_BUFFER_LENGTH ; m++)
	{
		sensor.light_buffer[m]=sensor.gray;
	}
	for(int m=0;m<sensor.GYRO_BUFFER_LENGTH;m++)
	{
		sensor.gyro_buffer[m] = controller.gyro_offset;
	}

}

/*
===============================================================================================
	name: EventSensor
	Description: ??
		Event
		ID description
		00	transition with no event
		01	touch
		02	white (not correctly implemented)
		03	black (not correctly implemented)
		04	gray marker
		05	step
		06	seesaw tilts
		07	dropped from the seesaw
		08	sonar sensor
		09	set time is up
		10 reach the set motor encoder count
		11 receive the bluetooth start signal
		12 finish circling
		13 (some event1)
		14 (some event2)
 	Parameter: no
	Return Value: no
 	---
	update: 2013.06.13
===============================================================================================
*/
void EventSensor(){
	display_clear(0);
	display_goto_xy(0, 1);
	display_string("LIGHT=");
	display_int(ecrobot_get_light_sensor(LIGHT_SENSOR), 4);
	display_goto_xy(0, 2);
	display_string("S=");
	display_int(get_CurrentState(), 4);
	display_update();


	sendevent(0); //always send event0


	if(sensor.touched > 0 && eventStatus.touch_status != TOUCH_STATUS_PRESSED)
	{
		//Touch Event!!
		sendevent(1);
		eventStatus.touch_status = TOUCH_STATUS_PRESSED;
	}
	else if(sensor.touched == 0 && eventStatus.touch_status != TOUCH_STATUS_NOTPRESSED)
	{
		eventStatus.touch_status = TOUCH_STATUS_NOTPRESSED;
	}

	if(sensor.light>sensor.black-50 && eventStatus.light_status != LIGHT_STATUS_BLACK)
	{
		//Black Event!!
		sendevent(3);
		eventStatus.light_status = LIGHT_STATUS_BLACK;
	}
	else if(sensor.light<sensor.white+50 && eventStatus.light_status != LIGHT_STATUS_WHITE)
	{
		//White Event!!
		sendevent(2);
		eventStatus.light_status = LIGHT_STATUS_WHITE;
	}
/*
	sensor.light_min=10000;
	sensor.light_max=0;
	for(int k=0;k<sensor.LIGHT_BUFFER_LENGTH;k++){
		if(sensor.light_buffer[k]<sensor.light_min){
			sensor.light_min = sensor.light_buffer[k];
		}
		if(sensor.light_buffer[k]>sensor.light_max){
			sensor.light_max = sensor.light_buffer[k];
		}
	}
	sensor.light_diff = abs(sensor.light_max - sensor.light_min);
	if( sensor.light_diff > controller.gray_offset ){
		sendevent(4);
	}
*/


	if( sensor.light-sensor.gray > controller.gray_offset ){
		sendevent(4);
	}

/*
	U16 sensor_ave=0;
	long ave=0;
	for(int n=0;n<sensor.LIGHT_BUFFER_LENGTH;n++){
		ave+=sensor.light_buffer[n];
	}
	sensor_ave=ave/sensor.LIGHT_BUFFER_LENGTH;
	if(abs(sensor.light-sensor_ave) > controller.gray_offset){
		sendevent(4);
	}
*/
	//step identify
	U16 sensor_gyro_ave=0;
	long gyro_ave=0;
	for(int m=0;m<sensor.GYRO_BUFFER_LENGTH;m++){
		gyro_ave+=sensor.gyro_buffer[m];
	}
	sensor_gyro_ave = gyro_ave/sensor.GYRO_BUFFER_LENGTH;
	if( abs(sensor.gyro - sensor_gyro_ave) > controller.step_offset	){
		sendevent(5);
	}

   	if(sensor.sonar < sensor.sonar_value ){
		sendevent(8);
	}
	if(systick_get_ms()-eventStatus.start_timer>eventStatus.limit_time && eventStatus.timer_flag==TIMER_PROCESSING){
		sendevent(9);
		eventStatus.timer_flag=0;
	}
	int motor_count = (nxt_motor_get_count(LEFT_MOTOR) + nxt_motor_get_count(RIGHT_MOTOR)) / 2;
	if(eventStatus.motor_count != 0 && abs(motor_count - eventStatus.start_motor_count) > abs(eventStatus.motor_count)) {
	sendevent(10);
		eventStatus.motor_count = 0;

	}


   //bt_start
	if(eventStatus.started != STARTED){
		ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
		if(bt_receive_buf[1]==254 ){
			sendevent(11);
			eventStatus.started = STARTED;
		}
	}

	//finshed_turn_still
	if(eventStatus.circling_on==1 && abs(nxt_motor_get_count(RIGHT_MOTOR)-eventStatus.circling_start_encoder_R) >abs(eventStatus.circling_target_angle_R)){
		sendevent(12);
		eventStatus.circling_on=0;
	}

	/////////////////////
	//you can use sendevent(13),sendevent(14)	for some events
	//////////////////////

	if(sensor.gyro_V>100){


	}

	if(eventStatus.bottle_judge!=0){
		//sendevent turn left or right
		if(sensor.bottle_is_right!=0 && sensor.bottle_is_left!=0){
			//////
		}else if(sensor.bottle_is_right==0 && sensor.bottle_is_left==0){

		}else if(sensor.bottle_is_right!=0){
			sendevent(14);
		}else if(sensor.bottle_is_left!=0){
			sendevent(13);
		}



		eventStatus.bottle_judge=1;
	}


	if(eventStatus.num_to_loop>0){
		if(eventStatus.loop_count>eventStatus.num_to_loop){
			eventStatus.loop_count=0;
			eventStatus.num_to_loop=0;
			sendevent(15);

		}else{
			sendevent(16);
		}
	}




}



/*
===============================================================================================
	name: ControllerSet
	Description: ??
		Action
		ID description
		00	do nothing
		01	balanced stop
		02	run at the balanced linetrace
		03	change the gyro offset
		04	change the gray threshold
		05	run with tail (do NOT linetrace)
		06	down the tail at speed 15
		07	NOT USED (currently same as action 3)
		08	set timer
		09	set motor encoder count
		10 set pid values
		11 NOT USED (currently almost same as action 5)
		12 set the offset to identify the step
		13 up the tail at speed -15
		14 run with the tail at the linetrace
		15 circling
		16 select the logger type
		17 set the gray marker offset
		18 run at the balanced (do NOT linetrace)
		19 (some action1)
		20 set sonar value
		21 look for bottle while turning right before drift turn
		22 look for bottle while turning left before dift turn
  	Parameter: no
	Return Value: no
 	---
	update: 2013.06.13
===============================================================================================
*/

void ControllerSet(State_t *state) {
	switch(state->action_no) {
		case 0://do nothing
			controller.speed = 0;
			controller.forward_power=0;//state->value0;
			controller.turn = 0;//state->value1;
			controller.pid_on=0;
			controller.wg_pid_on=0;
			controller.balance_on=1;
			controller.tail_on=0;
			break;
		case 1://stop
			controller.speed = 0;
			controller.forward_power=0;//state->value0;

			nxt_motor_set_speed(TAIL_MOTOR,0,1);
			controller.pid_on=1;
			controller.wg_pid_on=0;
			controller.tail_on=0;

			if(controller.balance_on==0){
				controller.balance_on=1;
				balance_init();
				nxt_motor_set_count(LEFT_MOTOR,0);
				nxt_motor_set_count(RIGHT_MOTOR,0);

			}


			break;
		// linetrace
		//@param speed:=value0
		//@param gyro_offset:=value1
		case 2:
			controller.speed = state->value0;//state->value0;
			controller.gyro_offset = controller.base_gyro_offset + state->value1;

			controller.pid_on = 1;

			if(controller.balance_on == 0){
				controller.balance_on = 1;
				balance_init();
				nxt_motor_set_count(LEFT_MOTOR,0);
				nxt_motor_set_count(RIGHT_MOTOR,0);

			}

			sensor.gray = sensor.threshold_gray;
			nxt_motor_set_speed(TAIL_MOTOR,0,1);

			break;
		//go up the steps
		//@param gyro-offset:=value0
		//need implementation
		case 3:
			controller.gyro_offset = controller.base_gyro_offset + state->value0;

			controller.pid_on = 0;
			controller.balance_on=1;
			break;
		//change the gray threshold
		//@param new threshold:=value0
		case 4:
//			controller.color_threshold=state->value0;
//			sensor.gray=state->value0;
			sensor.gray = sensor.calib_gray;

			controller.pid_on = 0;
			controller.speed = 0;
			controller.forward_power = 0;
			controller.turn = 0;
			break;
		//run with no linetrace without balance
		//@param tail_ang:=value0
		//@param tail_run_speed :=value1
		//@param turn :=value2
		case 5:
			controller.tail_ang = state->value0;
			controller.tail_run_speed = state->value1;
			controller.turn = state->value2;
			controller.tail_speed_offset = state->value3;

			controller.pid_on = 0;
			controller.wg_pid_on = 0;
			controller.balance_on = 0;
			controller.tail_on = 1;
			break;

		//down the tail
		case 6:

			nxt_motor_set_speed(TAIL_MOTOR,15,1);
			break;
		//
		case 7:
			controller.gyro_offset = state->value0;
			controller.pid_on=0;
			break;
		//set timer
		//@param limit_timer:=value0 i.e. 20 = 2.0sec
		case 8:
			eventStatus.timer_flag = TIMER_PROCESSING;
			eventStatus.start_timer = systick_get_ms();
			eventStatus.limit_time = state->value0 *100;

			break;
		//set motor count
		case 9:
			eventStatus.start_motor_count = (nxt_motor_get_count(LEFT_MOTOR) + nxt_motor_get_count(RIGHT_MOTOR)) / 2;
			eventStatus.motor_count = state->value0;


			//turn();	//balancer
			break;
		//set PID
		//@param P_gain:=value0/100
		//@param I_gain:=value1/100
		//@param D_gain:=value2/100
		case 10:

			controller.P_gain = (float)state->value0 / 100;
			controller.I_gain = (float)state->value1 / 100;
			controller.D_gain = (float)state->value2 / 100;


			break;

		//NOT USED
		case 11:
			controller.forward_power=state->value0;
			controller.turn = state->value1;

			controller.pid_on=0;
			break;
		//set gyro offset for steps
		//@param step_offset := value0
		case 12:
			controller.step_offset=state->value0;
			sensor.GYRO_BUFFER_LENGTH=state->value1;
			break;
		//up the tail
		case 13:
			nxt_motor_set_speed(TAIL_MOTOR,-15,1);
			break;
		// tail run
		//@param angle
		//@param speed
 		case 14:
			controller.tail_on=1;
			controller.pid_on=1;
			controller.balance_on=0;
			controller.tail_ang=state->value0;
			controller.tail_run_speed=state->value1;
			controller.tail_speed_offset=state->value2;
			break;
		//circling
		//@param angle to turn
		case 15:
			//controller.tail_on=1;
			//controller.balance_on=0;
			controller.pid_on=0;
			controller.forward_power=0;
			controller.tail_run_speed=1;
			if(controller.balance_on==1){
				controller.turn = state->value1;
			}else{
				controller.turn = -(state->value1);
			}

			if(state->value0 < 0){
				controller.turn *= -1;
			}
			eventStatus.circling_start_encoder_R = nxt_motor_get_count(RIGHT_MOTOR);
			eventStatus.circling_target_angle_R = calc_angle2encoder(state->value0);
			eventStatus.circling_on = 1;
			break;
		//selecting logger
		//@param log_type
		case 16:
			logger.type = state->value0;
			break;
		//set the gray_market offset
		//@param gray_offset
		case 17:
			controller.gray_offset=state->value0;
			sensor.LIGHT_BUFFER_LENGTH=state->value1;

			break;
		//free balance
		case 18:
			controller.tail_on=0;
			controller.pid_on=0;
			controller.wg_pid_on=0;

			if(controller.balance_on==0){
				controller.balance_on=1;
				nxt_motor_set_count(LEFT_MOTOR,0);
				nxt_motor_set_count(RIGHT_MOTOR,0);
				balance_init();

			}
			controller.forward_power=state->value0;
			controller.turn=state->value1;
			controller.gyro_offset = controller.base_gyro_offset + state->value2;

			nxt_motor_set_speed(TAIL_MOTOR,0,1);

			break;

		//white_gray_linetrace(use tail)
		case 19:
			controller.tail_on=1;
			controller.wg_pid_on=1;
			controller.balance_on=0;
			controller.tail_ang=state->value0;
			controller.tail_run_speed=state->value1;
			controller.tail_speed_offset=state->value2;
			break;

		//set_sonar_sensor
		case 20:
			sensor.sonar_value=state->value0;
			break;

		case 21:
			eventStatus.bottle_right_length=state->value0;

			break;
		case 22:
			eventStatus.bottle_left_length=state->value0;

			break;
		case 23:
			eventStatus.bottle_right_length=0;
			eventStatus.bottle_left_length=0;
			break;
		case 24:
			eventStatus.bottle_judge=1;
			break;

		case 25:
			init_nxt();
			break;
		case 26:
			controller.right_motor_rate = (float)state->value0/1000;
			controller.left_motor_rate = (float)state->value1/1000;
			break;
		case 27:
			eventStatus.num_to_loop = state->value0;
			break;
		case 28:
			eventStatus.loop_count=eventStatus.loop_count+1;
			break;

	}
}


//*******************************************
//	calibration the white and black value
//******************************************
void calibration(int *black,int *white,int *gray){


	while( ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0 ){

		sensor.calib_gray = ecrobot_get_light_sensor(LIGHT_SENSOR);

		display_clear(0);
		display_goto_xy(0, 1);
		display_string("CALIB_GRAY=");
		display_int(sensor.calib_gray, 4);
		display_update();

		systick_wait_ms(10);
	}
	ecrobot_sound_tone(200,20,50);
	systick_wait_ms(1000);


	while( ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0 ){

		*white = ecrobot_get_light_sensor(LIGHT_SENSOR);

		display_clear(0);
		display_goto_xy(0, 1);
		display_string("WHITE=");
		display_int(*white, 4);
		display_update();

		systick_wait_ms(10);
	}
	ecrobot_sound_tone(100,10,50);
	systick_wait_ms(1000);



	while(ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0){

		*black = ecrobot_get_light_sensor(LIGHT_SENSOR);

		display_clear(0);
		display_goto_xy(0, 1);
		display_string("BLACK=");
		display_int(*black, 4);
		display_update();

		systick_wait_ms(10);

	}
	ecrobot_sound_tone(100,10,50);
	systick_wait_ms(1000);


	*gray=( *black + *white ) / 2;
	sensor.white_gray_threshold = (sensor.calib_gray + *white) / 2;

	display_clear(0);
	display_goto_xy(0, 1);
	display_string("gray=");
	display_int(*gray, 4);
	display_update();

	systick_wait_ms(1000);

}

//***********************************
//	calibration the base gyro value
//**********************************

void gyro_calibration(){

	int i;
	int sum=0;
	byte calibration_times =1;
	for(i=0;i<calibration_times;i++){
		U16 _gyro=0;
		while( ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0 ){

			 _gyro = ecrobot_get_gyro_sensor(GYRO_SENSOR);

			display_clear(0);
			display_goto_xy(0, 1);
			display_string("gyro=");
			display_int(i, 4);
			display_int(_gyro, 6);
			display_update();

			systick_wait_ms(10);

		}
		sum += (int)_gyro;
		ecrobot_sound_tone(100,10,50);
		systick_wait_ms(1000);
	};

	controller.gyro_offset=sum/calibration_times;
	sensor.prev_gyro_value = controller.gyro_offset;
	controller.base_gyro_offset = controller.gyro_offset;

	display_clear(0);
	display_goto_xy(0, 1);
	display_string("sum=");
	display_int(sum, 6);
	display_update();
	//systick_wait_ms(1000);

	display_clear(0);
	display_goto_xy(0, 1);
	display_string("gyro_ave=");
	display_int(controller.gyro_offset, 6);
	display_update();
	//systick_wait_ms(1000);



	return;

}

//****************************************
//return the Left and Right motor power
// from speed and turn for tail running
//***************************************
void tail_run_turn2pwm(S16 _tail_run_speed ,float _turn ,S8 *_pwm_L, S8 *_pwm_R){
	if(_tail_run_speed!=0){

		if(_tail_run_speed+_turn>100){
			float _turn_overflow = _tail_run_speed+_turn -100;
			*_pwm_R = 100;
			*_pwm_L = (S8)(_tail_run_speed-_turn-_turn_overflow);

		}else if(_tail_run_speed-_turn > 100){
			float _turn_overflow = _tail_run_speed-_turn -100;
			*_pwm_L = 100;
			*_pwm_R = (S8)(_tail_run_speed+_turn-_turn_overflow);

		}else{
			*_pwm_R = (S8)(_tail_run_speed+_turn);
			*_pwm_L = (S8)(_tail_run_speed-_turn);
		}

		/////
	//	*_pwm_L = (S8)(_tail_run_speed+_turn);
	//	*_pwm_R = (S8)(_tail_run_speed-_turn);
		//////
	}else{
		*_pwm_L = 0;
		*_pwm_R = 0;

	}

}

//**********************
//calculate the motor encoder value
//from the angle for cicling
//**********************

S16 calc_angle2encoder(S16 _ang){
	S16 ret=((float)_ang*16.3/8.1);
	if(ret<0){ret*=-1;}
	return ret;


}

/*
***************************************************************
	name:calc_variance
	description:

	Parameter:
		U16* buf: An array of sensor data.
		int _len: length of an array
	Return Value:

***************************************************************
*/
S8 calc_variance(U16 *buf,int _len){
	//calculate light valiance
	long sum=0;
	long sumv=0;
	int ave=0;
	int squ=0;
	for(int k=0;k<_len;k++){

			if(buf[k]==0){return 0;}
			sum += buf[k];
			sumv +=buf[k]*buf[k];

	}
	ave = sum / _len;
	squ = sumv-sum*sum/_len;
	float V = (float)squ /((float)_len -1);
	//V *=10;
	//V /=5;  //may not need to divide by 5
	//int V =(float)sum/(float)_len*100;
	if(V>127){
		V=127;
	}else if(V<-128){
		V= -128;
	}

	return (S8)V;


}

