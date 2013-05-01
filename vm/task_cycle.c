#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "statics/balancer.h"
#include "statics/port_interface.h"
#include "bluetooth_interface.h"
#include "statics/StateMachine.h"
#include "SensorManager.h"
#include "Controller.h"
#include "Logger.h"
#include <math.h>

DeclareCounter(SysTimerCnt);
DeclareTask(TaskInit);
DeclareTask(TaskBalance);				/* Task to controll balance*/
DeclareTask(TaskSensor);				/* Task to keep sensoring */
DeclareTask(TaskLogger);				/*Task to send log*/

////////////////////////////////////////
///prototype declaration
/////////////////////////////////
void event_manager();
void gyro_calibration();
void init_nxt();
void calibration(int *black,int *white,int *gray);
void tail_run_turn2pwm(S16 _tail_run_speed ,float _turn ,S8 *_pwm_L, S8 *_pwm_R);
S16 calc_angle2encoder(S16 angle);
S8 calc_variance(U16 *buf,int _len);
void receive_BT(StateMachine_t *sm);

/////////////////////////////////
//variables
////////////////////////////////



#define BLUETOOTH
//#define SEND_STATE
//#define DEBUG

#ifdef BLUETOOTH
S16 num_of_events,num_of_states;
//S16 *matrix;
//S16 *states;




#endif

//we should make these valiables dynamic valiables

int i;
int count = 1;
int g_count = 1;
///////state machine//////////////////////



////////balance_control//////////////////////

	S8 init=0;
	S8 turn;		  	 			
	S8 pwm_L, pwm_R;				
	
//////sensor/////////////////////
Sensor_t *sensor;
StateMachine_t statemachine;
Event_t *events;
//Controller_t *controller;
Controller_t controller[1];


EventStatus_t eventStatus = {LIGHT_STATUS_UNDEFINED, 0, TOUCH_STATUS_NOTPRESSED, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0};

Logger_t logger={0};

//end of event manager


//*****************************************************************************
// ecrobot_device_initialize
//*****************************************************************************

void ecrobot_device_initialize(){
	nxt_motor_set_speed(LEFT_MOTOR,0,0);			
	nxt_motor_set_speed(RIGHT_MOTOR,0,0);			
	nxt_motor_set_count(TAIL_MOTOR,0);
	ecrobot_set_light_sensor_active(LIGHT_SENSOR);		
	ecrobot_init_sonar_sensor(SONAR_SENSOR);		/*sonar*/
	ecrobot_init_bt_slave(BT_PASS_KEY);			

}

//*****************************************************************************
// ecrobot_device_terminate
//*****************************************************************************

void ecrobot_device_terminate(){
	nxt_motor_set_speed(LEFT_MOTOR,0,0);
	nxt_motor_set_speed(RIGHT_MOTOR,0,0);
	ecrobot_term_bt_connection();		
	ecrobot_set_light_sensor_inactive(LIGHT_SENSOR);	
	ecrobot_term_sonar_sensor(SONAR_SENSOR);		/*sonar*/
//	free(matrix);
//	free(states);
//	free(controller);
	free(sensor);
}


//*****************************************************************************
// user_1ms_isr_type2
//*****************************************************************************

void user_1ms_isr_type2(void){
	SignalCounter(SysTimerCnt);   
}

Task(TaskInit){
	TerminateTask();
}


/////////////////////
//TaskMain
////////////////////

TASK(TaskMain){
	event_manager();	

	TerminateTask();					
}


//////////////////////////////////////
///TaskBalance
///
//////////////////////////////////


TASK(TaskBalance)
{

	if(init==0){

		balance_init();							
		nxt_motor_set_count(NXT_PORT_C,0);
		nxt_motor_set_count(NXT_PORT_B,0);
		init =1;
		

		sensor = (Sensor_t *)malloc(sizeof(Sensor_t));
		if(sensor==NULL){
			
			display_clear(0);
			display_goto_xy(0, 1);
			display_string("malloc error sensor");
			display_update();
		}
		
		sensor->light = ecrobot_get_light_sensor(LIGHT_SENSOR);
		sensor->gyro= ecrobot_get_gyro_sensor(GYRO_SENSOR);
		sensor->touched = ecrobot_get_touch_sensor(TOUCH_SENSOR);
		sensor->sonar = ecrobot_get_sonar_sensor(SONAR_SENSOR);
		//sensor->distance = (S32)getDistance();
		
		sensor->prev_light_value = sensor->light;

		balance_init();


#ifdef BLUETOOTH


	receive_BT(&statemachine);


#endif



		gyro_calibration();
		calibration(&sensor->black,&sensor->white,&sensor->gray);
		
		init_nxt();


	}
	

	int before=0, standard=0;	// beforeAEAE?A§Aac?AEAAeA∑A?AAstandardAEoAeaAuR?AAAAeA∑A 
	float integral=0;

	if(controller->pid_on==1){
		sensor->light = ecrobot_get_light_sensor(LIGHT_SENSOR);
		before = standard;
		standard = sensor->light - sensor->gray;					// AAeA∑A?CiAenAaAE
		controller->integral += (standard - before)/2.0 * 0.004;	

		//	controller->turn = 1 * standard *100 / (black-white) + 1 * controller->integral / (black-white) * 100 + 1 * (sensor->light-sensor->prev_light_value) / (black-white) * 100;	//EoaAouAA§ERaAAAE

		controller->turn = controller->P_gain * standard *100 / (sensor->black-sensor->white) + controller->I_gain * integral / (sensor->black-sensor->white) * 100 + controller->D_gain * (sensor->light-sensor->prev_light_value) / (sensor->black-sensor->white) * 100;	// EoaAouAA§ERaAAAE
		sensor->prev_light_value = sensor->light;		//1?A§Aac?AEEoe?Ca?Ai?CiE†oA\AE
		controller->forward_power=controller->speed;
	}
	else if(controller->wg_pid_on==1){
		sensor->light = ecrobot_get_light_sensor(LIGHT_SENSOR);
		before = standard;
		standard = sensor->light - sensor->white_gray_threshold;
		controller->integral += (standard - before)/2.0 * 0.004;

		controller->turn = controller->P_gain * standard *100 / (sensor->calib_gray-sensor->white) + controller->I_gain * integral / (sensor->calib_gray-sensor->white) * 100 + controller->D_gain * (sensor->light-sensor->prev_light_value) / (sensor->calib_gray-sensor->white) * 100;	// 旋回値計?E
		sensor->prev_light_value = sensor->light;
		controller->forward_power=controller->speed;
	}
	else
	{
		//controller->turn=0;
		//controller->forward_power=controller->speed;
	}

	
	if(controller->balance_on==1){
		balance_control(					//?Ee?Ec?E??CμAPI?AAAeo?A?Aa∫?AAE
			(float)controller->forward_power,		//AacEA?AEeAaaEAAAeΩ‰a§(-100(Ae?EoaAouEuAA§s)?100(A∑¶EoaAouEuAA§s))
			(float)controller->turn,			//EoaAouAeΩ‰a§(-100?100)
			(float)ecrobot_get_gyro_sensor(NXT_PORT_S1),	//?C??E￡?C§?E≠?Ca?E??CμAA§
			(float)controller->gyro_offset,				//?C??E￡?C§?E≠?Ca?E??Cμ?C??Ei?Ca?EAEEaAA§
			(float)nxt_motor_get_count(NXT_PORT_C),		//A∑¶?E￠?Eo?Co?CR?E??C??Eo?EAAA§
			(float)nxt_motor_get_count(NXT_PORT_B),		//Ae??E￠?Eo?Co?CR?E??C??Eo?EAAA§
			(float)ecrobot_get_battery_voltage(),		//?Ee?EE?EAEE?EoaAusAA§[mV]

			&pwm_L,						//A∑¶?E￠?Eo?CoPWMAa∫AaoAA§AEaEaa?CaAA§AEAE
			&pwm_R						//Ae??E￠?Eo?CoPWMAa∫AaoAA§AEaEaa?CaAA§AEAE

		);

		nxt_motor_set_speed(NXT_PORT_C, (S8)(pwm_L*controller->left_motor_rate), 1);		//?E￠?Eo?Co?A´EaAEa§?CiAAE?AAE
		nxt_motor_set_speed(NXT_PORT_B, (S8)(pwm_R*controller->right_motor_rate), 1);
	
	}else if(controller->tail_on==1){
		
		U32 _tail_ang = nxt_motor_get_count(TAIL_MOTOR);
		U8 _tail_speed;
	
		if(_tail_ang>=0){
			_tail_speed = (U8)(10.0/9.0*(controller->tail_ang - _tail_ang)+controller->tail_speed_offset);
		}else{
			_tail_speed=10;
		}
	
		

		if(_tail_ang > controller->tail_ang+1){
			nxt_motor_set_speed(TAIL_MOTOR,-50,1);
		
			
		}else if(_tail_ang < controller->tail_ang-1){
			nxt_motor_set_speed(TAIL_MOTOR,_tail_speed,1);
		
		
		}else{
			
			nxt_motor_set_speed(TAIL_MOTOR,0,1);
				
		}
		tail_run_turn2pwm(
				(S16) controller->tail_run_speed,
				(float) controller->turn,
				&pwm_L,
				&pwm_R
		);
		nxt_motor_set_speed(RIGHT_MOTOR,(S8)(pwm_R*controller->right_motor_rate),1);
		nxt_motor_set_speed(LEFT_MOTOR,(S8)(pwm_L*controller->left_motor_rate),1);

	}




	TerminateTask();


}

//////////////////////
//TaskSensor
//////////////////////
int sonar_cnt=0;

TASK(TaskSensor)
{

		sensor->prev_light_value = sensor->light;
	sensor->prev_gyro_value= sensor->gyro;

	sensor->light_buffer[count]=sensor->prev_light_value;
	count++;
	if(count>=sensor->LIGHT_BUFFER_LENGTH){
		count=0;
	}
	sensor->gyro_buffer[g_count]=sensor->prev_gyro_value;
	g_count++;
	if(g_count>=sensor->GYRO_BUFFER_LENGTH){
		g_count=0;
	}

	sensor->light = ecrobot_get_light_sensor(LIGHT_SENSOR);
	if(sonar_cnt>10){
		sensor->sonar = ecrobot_get_sonar_sensor(SONAR_SENSOR);
		sonar_cnt=0;
	}else{
		sonar_cnt++;	
	}
	sensor->touched = ecrobot_get_touch_sensor(TOUCH_SENSOR);
	sensor->gyro= ecrobot_get_gyro_sensor(GYRO_SENSOR);	


	
	//calculate light valiance

	sensor->light_V = 1.0*calc_variance(sensor->light_buffer,V_LIGHT_BUFFER_LENGTH);
	
	//calculate gyro valiance

	sensor->gyro_V = 1.0*calc_variance(sensor->gyro_buffer,V_GYRO_BUFFER_LENGTH);


	if(eventStatus.bottle_right_length>0){
		if(sensor->sonar <eventStatus.bottle_right_length){
			sensor->bottle_is_right=1;
		}
	}
	if(eventStatus.bottle_left_length>0){
		if(sensor->sonar <eventStatus.bottle_left_length){
			sensor->bottle_is_left=1;
		}
	}



	TerminateTask();

}

////////////////////////
//TaskLogger
////////////////////////
TASK(TaskLogger){

	S8 _ang = nxt_motor_get_count(RIGHT_MOTOR)-eventStatus.circling_start_encoder_R -eventStatus.circling_target_angle_R;
	int rest_motor_count = (nxt_motor_get_count(LEFT_MOTOR) + nxt_motor_get_count(RIGHT_MOTOR)) / 2 - eventStatus.start_motor_count - eventStatus.motor_count;
	
		switch(logger.type){
			case LOG_STATE:
				ecrobot_bt_data_logger((S8)statemachine.current_state,111);
				break;
			case LOG_TURN:
				ecrobot_bt_data_logger((S8)controller->turn,112);		
				break;
			case LOG_PWM:
				ecrobot_bt_data_logger((S8)pwm_L,pwm_R);
				break;
			case LOG_TARGET_ANGLE:
				
				ecrobot_bt_data_logger((S8)(_ang/100),(S8)(_ang%100));
				break;
			case LOG_LIGHT_MIN:
				ecrobot_bt_data_logger((S8)(sensor->light_min%100),(S8)(sensor->light_min/100));
				//ecrobot_bt_data_logger((S8)(light_ave%100),(S8)(light_ave/100));
				break;
			case LOG_MOTOR_COUNT:
				ecrobot_bt_data_logger((S8)(rest_motor_count%100),(S8)(rest_motor_count/100));
				break;
			case LOG_SONAR:
				ecrobot_bt_data_logger((S8)(sensor->sonar%100),(S8)(sensor->sonar/100));
				break;
			case LOG_DT:
				ecrobot_bt_data_logger((S8)(sensor->bottle_is_right),(S8)(sensor->bottle_is_left));
				break;
			case LOG_BALANCE_TAIL:
				ecrobot_bt_data_logger((S8)(controller->tail_on),(S8)(controller->balance_on));
				break;
			case LOG_LOOP:
				ecrobot_bt_data_logger((S8)(eventStatus.loop_count),(S8)(eventStatus.num_to_loop));
				break;

			default:
				ecrobot_bt_data_logger(sensor->gyro_V,sensor->light_V);
				break;

		}
		TerminateTask();
}


int sendevent(event_id);

/*-------------------------
 
 Event
 ID description
 0	transition with no event
 1	touch
 2	white (not correctly implemented) 
 3	black (not correctly implemented)
 4	gray marker
 5	step
 6	seesaw tilts 
 7	dropped from the seesaw 
 8	sonar sensor
 9	set time is up
 10 reach the set motor encoder count
 11 receive the bluetooth start signal
 12 finish circling
 13 (some event1)
 14 (some event2)
 
 -------------------------*/
void event_manager(){
	display_clear(0);
	display_goto_xy(0, 1);
	display_string("LIGHT=");
	display_int(ecrobot_get_light_sensor(LIGHT_SENSOR), 4);
	display_goto_xy(0, 2);
	display_string("S=");
	display_int(statemachine.current_state, 4);
	display_update();

	//StateMachine_sendEvent(&statemachine,&events[0]);





	
	sendevent(0); //always send event0 


	if(sensor->touched > 0 && eventStatus.touch_status != TOUCH_STATUS_PRESSED) {
		//Touch Event!!
	sendevent(1);
		eventStatus.touch_status = TOUCH_STATUS_PRESSED;
	}else if(sensor->touched == 0 && eventStatus.touch_status != TOUCH_STATUS_NOTPRESSED){
		eventStatus.touch_status = TOUCH_STATUS_NOTPRESSED;
	}
	
	if(sensor->light>sensor->black-50 && eventStatus.light_status != LIGHT_STATUS_BLACK){
		//Black Event!!
		sendevent(3);
			eventStatus.light_status = LIGHT_STATUS_BLACK;
	}else if(sensor->light<sensor->white+50 && eventStatus.light_status != LIGHT_STATUS_WHITE){
		//White Event!!
		sendevent(2);
			 eventStatus.light_status = LIGHT_STATUS_WHITE;
	}
/*
	sensor->light_min=10000;
	sensor->light_max=0;
	for(int k=0;k<sensor->LIGHT_BUFFER_LENGTH;k++){
		if(sensor->light_buffer[k]<sensor->light_min){
			sensor->light_min = sensor->light_buffer[k];
		}
		if(sensor->light_buffer[k]>sensor->light_max){
			sensor->light_max = sensor->light_buffer[k];
		}
	}
	sensor->light_diff = abs(sensor->light_max - sensor->light_min);
	if( sensor->light_diff > controller->gray_offset ){
		sendevent(4);
	}
*/





	if(sensor->light-sensor->gray >controller->gray_offset){
		sendevent(4);
	}
	
/*
	U16 sensor_ave=0;
	long ave=0;
	for(int n=0;n<sensor->LIGHT_BUFFER_LENGTH;n++){
		ave+=sensor->light_buffer[n];
	}
	sensor_ave=ave/sensor->LIGHT_BUFFER_LENGTH;
	if(abs(sensor->light-sensor_ave) > controller->gray_offset){
		sendevent(4);
	}
*/	
	//step identify
	U16 sensor_gyro_ave=0;
	long gyro_ave=0;
	for(int m=0;m<sensor->GYRO_BUFFER_LENGTH;m++){
		gyro_ave+=sensor->gyro_buffer[m];
	}
	sensor_gyro_ave=gyro_ave/sensor->GYRO_BUFFER_LENGTH;
	if( abs(sensor->gyro - sensor_gyro_ave) > controller->step_offset	){
		sendevent(5);
	}

   	if(sensor->sonar < sensor->sonar_value ){
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
	
	if(sensor->gyro_V>100){
	
	
	}

	if(eventStatus.bottle_judge!=0){
		//sendevent turn left or right
		if(sensor->bottle_is_right!=0 && sensor->bottle_is_left!=0){
			//////
		}else if(sensor->bottle_is_right==0 && sensor->bottle_is_left==0){
			
		}else if(sensor->bottle_is_right!=0){
			sendevent(14);	
		}else if(sensor->bottle_is_left!=0){
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



/*-----------------------------
 Action
 ID description
 0	do nothing
 1	balanced stop
 2	run at the balanced linetrace 
 3	change the gyro offset
 4	change the gray threshold
 5	run with tail (do NOT linetrace)
 6	down the tail at speed 15
 7	NOT USED (currently same as action 3)
 8	set timer
 9	set motor encoder count
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
  ------------------------------*/

void StateMachine_action(State_t *state) {
	switch(state->action_no) {
		case 0://do nothing
			controller->speed = 0;
			controller->forward_power=0;//state->value0;
			controller->turn = 0;//state->value1;
			controller->pid_on=0;
			controller->wg_pid_on=0;
			controller->balance_on=1;
			controller->tail_on=0;
			break;
		case 1://stop
			 controller->speed = 0;
			controller->forward_power=0;//state->value0;
			
			nxt_motor_set_speed(TAIL_MOTOR,0,1);
			controller->pid_on=1;
			controller->wg_pid_on=0;
			controller->tail_on=0;
			
			if(controller->balance_on==0){
				controller->balance_on=1;
				balance_init();
				nxt_motor_set_count(LEFT_MOTOR,0);
				nxt_motor_set_count(RIGHT_MOTOR,0);
				
			}


			break;
		// linetrace
		//@param speed:=value0
		//@param gyro_offset:=value1
		case 2: 
			controller->speed = state->value0;//state->value0;
			controller->gyro_offset = controller->base_gyro_offset + state->value1; 
		
			controller->pid_on=1;
			
			if(controller->balance_on==0){
				controller->balance_on=1;
				balance_init();
				nxt_motor_set_count(LEFT_MOTOR,0);
				nxt_motor_set_count(RIGHT_MOTOR,0);
				
			}			

			sensor->gray=sensor->threshold_gray;
			nxt_motor_set_speed(TAIL_MOTOR,0,1);			

			break;
		//go up the steps 
		//@param gyro-offset:=value0
		//need implementation
		case 3:
			controller->gyro_offset = controller->base_gyro_offset + state->value0;
		
			controller->pid_on=0;
			controller->balance_on=1;
			break;
		//change the gray threshold
		//@param new threshold:=value0
		case 4:
//			controller->color_threshold=state->value0;
//			sensor->gray=state->value0;
			sensor->gray=sensor->calib_gray;

			controller->pid_on=0;
			controller->speed=0;
			controller->forward_power=0;
			controller->turn=0;
			break;
		//run with no linetrace without balance
		//@param tail_ang:=value0
		//@param tail_run_speed :=value1
		//@param turn :=value2
		case 5:
			
			controller->tail_ang=state->value0;
			controller->tail_run_speed=state->value1;
			controller->turn =state->value2;
			controller->tail_speed_offset=state->value3;
		
			controller->pid_on=0;
			controller->wg_pid_on=0;
			controller->balance_on=0;
			controller->tail_on=1;
			break;

		//down the tail	
		case 6:
			
			nxt_motor_set_speed(TAIL_MOTOR,15,1);
			break;
		//
		case 7: 
			controller->gyro_offset = state->value0;
			controller->pid_on=0;
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

			controller->P_gain = (float)state->value0 / 100;
			controller->I_gain = (float)state->value1 / 100;
			controller->D_gain = (float)state->value2 / 100;


			break;
		
		//NOT USED
		case 11:
			controller->forward_power=state->value0;
			controller->turn = state->value1;
				
			controller->pid_on=0;
			break;
		//set gyro offset for steps
		//@param step_offset := value0
		case 12:
			controller->step_offset=state->value0;
			sensor->GYRO_BUFFER_LENGTH=state->value1;
			break;
		//up the tail
		case 13:
			nxt_motor_set_speed(TAIL_MOTOR,-15,1);
			break;
		// tail run
		//@param angle 
		//@param speed
 		case 14:
			controller->tail_on=1;
			controller->pid_on=1;
			controller->balance_on=0;
			controller->tail_ang=state->value0;
			controller->tail_run_speed=state->value1;
			controller->tail_speed_offset=state->value2;
			break;
		//circling
		//@param angle to turn
		case 15:
			//controller->tail_on=1;
			//controller->balance_on=0;
			controller->pid_on=0;
			controller->forward_power=0;
			controller->tail_run_speed=1;
			if(controller->balance_on==1){
				controller->turn=state->value1;
			}else{
				controller->turn=-(state->value1);
			}

			if(state->value0<0){
				controller->turn *=-1;
			}
			eventStatus.circling_start_encoder_R=nxt_motor_get_count(RIGHT_MOTOR);
			eventStatus.circling_target_angle_R=calc_angle2encoder(state->value0);
			eventStatus.circling_on=1;
			break;
		//selecting logger
		//@param log_type
		case 16:
			logger.type = state->value0;
			break;
		//set the gray_market offset
		//@param gray_offset
		case 17:
			controller->gray_offset=state->value0;
			sensor->LIGHT_BUFFER_LENGTH=state->value1;
			
			break;
		//free balance
		case 18:
			controller->tail_on=0;
			controller->pid_on=0;
			controller->wg_pid_on=0;
			
			if(controller->balance_on==0){
				controller->balance_on=1;
				nxt_motor_set_count(LEFT_MOTOR,0);
				nxt_motor_set_count(RIGHT_MOTOR,0);
				balance_init();
				
			}
			controller->forward_power=state->value0;
			controller->turn=state->value1;
			controller->gyro_offset = controller->base_gyro_offset + state->value2; 
		
			nxt_motor_set_speed(TAIL_MOTOR,0,1);
			
			break;

		//white_gray_linetrace(use tail)
		case 19:
			controller->tail_on=1;
			controller->wg_pid_on=1;
			controller->balance_on=0;
			controller->tail_ang=state->value0;
			controller->tail_run_speed=state->value1;
			controller->tail_speed_offset=state->value2;
			break;

		//set_sonar_sensor
		case 20:
			sensor->sonar_value=state->value0;
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
			controller->right_motor_rate = (float)state->value0/1000;
			controller->left_motor_rate = (float)state->value1/1000;
			break;
		case 27:
			eventStatus.num_to_loop = state->value0;
			break;
		case 28:
			eventStatus.loop_count=eventStatus.loop_count+1;
			break;

	}
}

int sendevent(event_id) {
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

//*******************************************
//calibration the white and black value
//******************************************
void calibration(int *black,int *white,int *gray){	


	while( ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0 ){		

		sensor->calib_gray = ecrobot_get_light_sensor(LIGHT_SENSOR);	

		display_clear(0);				
		display_goto_xy(0, 1);
		display_string("CALIB_GRAY=");
		display_int(sensor->calib_gray, 4);
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
	sensor->white_gray_threshold = (sensor->calib_gray + *white) / 2;

	display_clear(0);
	display_goto_xy(0, 1);
	display_string("gray=");
	display_int(*gray, 4);
	display_update();

	systick_wait_ms(1000);

}

//***********************************
//calibration the base gyro value
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

	controller->gyro_offset=sum/calibration_times;
	sensor->prev_gyro_value = controller->gyro_offset;
	controller->base_gyro_offset = controller->gyro_offset;
	
	display_clear(0);				
	display_goto_xy(0, 1);
	display_string("sum=");
	display_int(sum, 6);
	display_update();
	//systick_wait_ms(1000);

	display_clear(0);				
	display_goto_xy(0, 1);
	display_string("gyro_ave=");
	display_int(controller->gyro_offset, 6);
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

void init_nxt(){

		controller->speed = 0;
		controller->forward_power=0;
		controller->turn=0;
		controller->balance_on=0;
		controller->pid_on=0;
		controller->wg_pid_on=0;
		controller->tail_on=0;
		controller->tail_ang=0;
		controller->tail_run_speed=0;
		//controller->gyro_offset=610;
		controller->step_offset=10000;
		controller->gray_offset = 10000;
		controller->color_threshold = 660;
		controller->P_gain=1.0;
		controller->I_gain=1.0;
		controller->D_gain=1.0;
		controller->right_motor_rate=1.0;
		controller->left_motor_rate=1.0;

		sensor->light_min=1000;
		sensor->light_max=0;
		sensor->bottle_is_left=0;
		sensor->bottle_is_right=0;

		sensor->threshold_gray=sensor->gray;
		controller->color_threshold = sensor->gray;
		sensor->prev_light_value=controller->color_threshold;

		for(int m=0 ; m < sensor->LIGHT_BUFFER_LENGTH ; m++){
			sensor->light_buffer[m]=sensor->gray;
		}
		for(int m=0;m<sensor->GYRO_BUFFER_LENGTH;m++){
			sensor->gyro_buffer[m] = controller->gyro_offset;
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


void receive_BT(StateMachine_t *sm){
		int packet_no=1;
		int ptr=0;
		S16 matrix[3000];
		S16 states[900];
		///////bluetooth
		//wait for　bluetooth
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
