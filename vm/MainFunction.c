#include <math.h>
//
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
//
#include "NXT_Config.h"
//#include "kfkf/kfkfModel.h"
#include "Logger.h"
#include "SensorManager.h"
#include "Controller.h"

/*
===============================================================================================
	macro
===============================================================================================
*/
#define BLUETOOTH
#define OFF 0
#define ON 1

/*
===============================================================================================
	task declaration
===============================================================================================
*/
DeclareCounter(SysTimerCnt);
DeclareTask(TaskMain);					/* Task to manage behavior of robot */
DeclareTask(TaskActuator);				/* Task to actuate */
DeclareTask(TaskSensor);				/* Task to sense */
DeclareTask(TaskLogger);				/* Task to send logging data */

/*
===============================================================================================
	prototype declaration
===============================================================================================
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



/*
===============================================================================================
	variables
===============================================================================================
*/
static U8 count = 0;
static U32 sum = 0;
static byte flag_calib = 0;;

int i;
int rest;
int i_value = 0;
///////state machine//////////////////////

//StateMachine_t statemachine;
Sensor_t sensor;
Controller_t controller;

Event_t *events;
Controller_t controller;

////////balance_control//////////////////////

	S8 init=0;
	S8 turn;		  	 			
	S8 pwm_L, pwm_R;				
	
//////sensor/////////////////////
Sensor_t sensor;  //initialize later
EventStatus_t eventStatus = {LIGHT_STATUS_UNDEFINED, 0, TOUCH_STATUS_NOTPRESSED, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0};
Logger_t logger={LOG_NO};

/*
===============================================================================================
	ecrobot_device_initialize
===============================================================================================
*/
void ecrobot_device_initialize(){
	nxt_motor_set_speed(LEFT_MOTOR,0,0);			
	nxt_motor_set_speed(RIGHT_MOTOR,0,0);			
	nxt_motor_set_count(TAIL_MOTOR,0);
	ecrobot_set_light_sensor_active(LIGHT_SENSOR);		
	ecrobot_init_sonar_sensor(SONAR_SENSOR);		/*sonar*/
	ecrobot_init_bt_slave(BT_PASS_KEY);			
}

/*
===============================================================================================
	ecrobot_device_terminate
===============================================================================================
*/
void ecrobot_device_terminate(){
	nxt_motor_set_speed(LEFT_MOTOR,0,0);
	nxt_motor_set_speed(RIGHT_MOTOR,0,0);
	ecrobot_term_bt_connection();		
	ecrobot_set_light_sensor_inactive(LIGHT_SENSOR);	
	ecrobot_term_sonar_sensor(SONAR_SENSOR);		/*sonar*/
}

/*
===============================================================================================
	user_1ms_isr_type2
===============================================================================================
*/
void user_1ms_isr_type2(void){
	SignalCounter(SysTimerCnt);   
}



/****************************************************************************************************************************
	Task
****************************************************************************************************************************/
/*
===============================================================================================
	name: TaskMain
	Description: ??
 	---
	update: 2013.06.13
===============================================================================================
*/
TASK(TaskMain)
{

	switch(/*robot state variable*/)
	{
	case START:	 /* Start State */
		break;
	case WAIT:	 /* Wait and Initialize */
		break;
	case BTCOMM: /* Communication by Bluetooth */
		receive_BT();
		break;
	case GYROCALIB:	 /* Do calibration */
		if(sensor.touched == 1) flag_calib = 1;

		if(flag_calib == 1)
		{
			count++;
			sum = sensor.gyro;
		}

		if(count >= 100)
		{
			controller.gyro_offset = (F32)sum/count;
			flag_calib = 0;
			sum = 0;
			count = 0;
			//to GRAYCALIB
		}
		break;
	case GRAYCALIB:
		if(sensor.touched == 1) flag_calib = 1;

		if(flag_calib == 1)
		{
			count++;
			sum = sensor.light;
		}

		if(count >= 100)
		{
			controller.light_gray = (F32)sum/count;
			controller.light_gray_base = controller.light_gray;
			flag_calib = 0;
			sum = 0;
			count = 0;
			//to WHITECALIB
		}
		break;
	case WHITECALIB:
		if(sensor.touched == ON) flag_calib = ON;

		if(flag_calib == ON)
		{
			count++;
			sum = sensor.light;
		}

		if(count >= 100)
		{
			controller.light_white = (F32)sum/count;
			flag_calib = OFF;
			sum = 0;
			count = 0;
			//to BLACKCALIB
		}
		break
	case BLACKCALIB:
		if(sensor.touched == ON) flag_calib = ON;

		if(flag_calib == ON)
		{
			count++;
			sum = sensor.light;
		}

		if(count >= 100)
		{
			controller.light_black = (F32)sum/count;
			flag_calib = OFF;
			sum = 0;
			count = 0;
			//to ACTION
		}
		break;
	case ACTION: /* Challenge contest */
		event_manager();
		break;
	}

	TerminateTask();					
}


/*
===============================================================================================
	name: TaskActuator
	Description: ??
 	---
	update: 2013.06.13
===============================================================================================
*/
TASK(TaskActuator)
{

	int before=0, standard=0;	// beforeAEAE?AﾂｧAac?AEAAeA竏羨?AAstandardAEoAeaAuR?AAAAeA竏羨
	float integral=0;

	if(controller.pid_on == ON)	// Linetrace border between black and white
	{
		before = standard;
		standard = sensor.light - sensor.gray;					// AAeA竏羨?CiAenAaAE
		controller.integral += (standard - before)/2.0 * 0.004;	

		controller.turn = controller.P_gain * standard *100 / (sensor.black-sensor.white)
				+ controller.I_gain * integral / (sensor.black-sensor.white) * 100
				+ controller.D_gain * (sensor.light - sensor.pre_light) / (sensor.black-sensor.white) * 100;	// EoaAouAAﾂｧERaAAAE

		sensor.prev_light_value = sensor.light;		//1?AﾂｧAac?AEEoe?Ca?Ai?CiE窶�A\AE
		controller.forward = controller.speed;
	}
	else if(controller.wg_pid_on == ON)	// Linetrace border between gray and white
	{
		//sensor.light = ecrobot_get_light_sensor(LIGHT_SENSOR);
		before = standard;
		standard = sensor.light - sensor.white_gray_threshold;
		controller.integral += (standard - before)/2.0 * 0.004;

		controller.turn = controller.P_gain * standard *100 / (sensor.calib_gray-sensor.white)
				+ controller.I_gain * integral / (sensor.calib_gray-sensor.white) * 100
				+ controller.D_gain * (sensor.light-sensor.prev_light_value) / (sensor.calib_gray-sensor.white) * 100;	// 譌句屓蛟､險�E

		sensor.prev_light_value = sensor.light;
		controller.forward = controller.speed;
	}

	
	if(controller.balance_on == ON)
	{
		balance_control(
			(F32)controller.forward,
			(F32)controller.turn,
			(F32)sensor.gyro,
			(F32)controller.gyro_offset,
			(F32)sensor.count_left,
			(F32)sensor.count_right,
			(F32)sensor.battery,
			&pwm_L,
			&pwm_R
		);

		//
		nxt_motor_set_speed(LEFT_MOTOR, (S8)(pwm_L), 1);
		nxt_motor_set_speed(RIGHT_MOTOR, (S8)(pwm_R), 1);
	
	}
	else if(controller.tail_on == ON)
	{
		S8 tail_speed;

		//tail_speed = (U8)(10.0/9.0*(controller.tail_ang - sensor.count_tail) + controller.tail_speed_offset);
		tail_speed = (U8)(controller.Tail_gain*(controller.tail_ang - sensor.count_tail));

		if(tail_speed > 100)
		{
			tail_speed = 100;
		}
		else if(tail_speed < -100)
		{
			tail_speed = -100;
		}
		
		nxt_motor_set_speed(TAIL_MOTOR,tail_speed,1);

		balance_control(
			(F32)controller.forward,
			(F32)controller.turn,
			(F32)controller.gyro_offset,
			(F32)controller.gyro_offset,
			(F32)sensor.count_left,
			(F32)sensor.count_right,
			(F32)sensor.battery,
			&pwm_L,
			&pwm_R
		);

		nxt_motor_set_speed(RIGHT_MOTOR,(S8)(pwm_R,1);
		nxt_motor_set_speed(LEFT_MOTOR,(S8)(pwm_L,1);
	}

	TerminateTask();
}

/*
===============================================================================================
	name: TaskSensor
	Description: ??
 	---
	update: 2013.06.13
===============================================================================================
*/

int g_SonarCnt = 0;

TASK(TaskSensor)
{
	//==========================================
	//	Data Update of Sensor
	//==========================================
	//--------------------------------
	//	Light Data
	//--------------------------------
	// Update Data of Light Sensor
	sensor.pre_light = sensor.light;
	sensor.light = ecrobot_get_light_sensor(LIGHT_SENSOR);

	//--------------------------------
	//	Gyro Data
	//--------------------------------
	// Update Data of Gyro Sensor
	sensor.gyro = ecrobot_get_gyro_sensor(GYRO_SENSOR);

	//--------------------------------
	//	Sonar Data
	//--------------------------------
	if(g_SonarCnt > 10)
	{
		// Update Data of Sonar Sensor
		sensor.sonar = ecrobot_get_sonar_sensor(SONAR_SENSOR);
		g_SonarCnt = 0;
	}
	else
	{
		g_SonarCnt++;
	}

	//--------------------------------
	//	Touch Data
	//--------------------------------
	// Update Data of Touch Sensor
	sensor.touched = ecrobot_get_touch_sensor(TOUCH_SENSOR);


	//--------------------------------
	//	Motor Data
	//--------------------------------
	sensor.count_left = nxt_motor_get_count(LEFT_MOTOR);
	sensor.count_right = nxt_motor_get_count(RIGHT_MOTOR);
	sensor.count_tail = nxt_motor_get_count(TAIL_MOTOR);

	//--------------------------------
	//	battery Data
	//--------------------------------
	sensor.battery = ecrobot_get_battery_voltage();

	//==========================================
	//	calculation
	//==========================================
	
	//calculate light valiance
	sensor.light_V = 1.0*calc_variance(sensor.light_buffer,V_LIGHT_BUFFER_LENGTH);
	
	//calculate gyro valiance
	sensor.gyro_V = 1.0*calc_variance(sensor.gyro_buffer,V_GYRO_BUFFER_LENGTH);


	//Bottle Detecting
	if(eventStatus.bottle_right_length > 0){
		if(sensor.sonar < eventStatus.bottle_right_length){
			sensor.bottle_is_right=1;
		}
	}
	if(eventStatus.bottle_left_length > 0){
		if(sensor.sonar < eventStatus.bottle_left_length){
			sensor.bottle_is_left=1;
		}
	}

	TerminateTask();
}

/*
###################################################################
	Task
	name: TaskLogger
	description:

###################################################################
*/
TASK(TaskLogger){

	S8 _ang = nxt_motor_get_count(RIGHT_MOTOR) - eventStatus.circling_start_encoder_R -eventStatus.circling_target_angle_R;
	int rest_motor_count = (nxt_motor_get_count(LEFT_MOTOR) + nxt_motor_get_count(RIGHT_MOTOR)) / 2 - eventStatus.start_motor_count - eventStatus.motor_count;
	
		switch(logger.type){
			case LOG_STATE:
				ecrobot_bt_data_logger((S8)get_CurrentState(),111);
				break;
			case LOG_TURN:
				ecrobot_bt_data_logger((S8)controller.turn,112);		
				break;
			case LOG_PWM:
				ecrobot_bt_data_logger((S8)pwm_L,pwm_R);
				break;
			case LOG_TARGET_ANGLE:
				ecrobot_bt_data_logger((S8)(_ang/100),(S8)(_ang%100));
				break;
			case LOG_LIGHT_MIN:
				//ecrobot_bt_data_logger((S8)(sensor.light_min%100),(S8)(sensor.light_min/100));
				break;
			case LOG_MOTOR_COUNT:
				ecrobot_bt_data_logger((S8)(rest_motor_count%100),(S8)(rest_motor_count/100));
				break;
			case LOG_SONAR:
				ecrobot_bt_data_logger((S8)(sensor.sonar%100),(S8)(sensor.sonar/100));
				break;
			case LOG_DT:
				ecrobot_bt_data_logger((S8)(sensor.bottle_is_right),(S8)(sensor.bottle_is_left));
				break;
			case LOG_BALANCE_TAIL:
				ecrobot_bt_data_logger((S8)(controller.tail_on),(S8)(controller.balance_on));
				break;
			case LOG_LOOP:
				ecrobot_bt_data_logger((S8)(eventStatus.loop_count),(S8)(eventStatus.num_to_loop));
				break;
			default:
				ecrobot_bt_data_logger(sensor.gyro,sensor.light);
				break;

		}
		TerminateTask();
}

/****************************************************************************************************************************
	Function
****************************************************************************************************************************/
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
	sensor.gyro = ecrobot_get_gyro_sensor(GYRO_SENSOR);
	sensor.touched = ecrobot_get_touch_sensor(TOUCH_SENSOR);
	sensor.sonar = ecrobot_get_sonar_sensor(SONAR_SENSOR);
	//sensor.distance = (S32)getDistance();

	sensor.pre_light = sensor.light;

	controller.speed = 0;
	controller.forward = 0;
	controller.turn = 0;
	controller.balance_on = 0;
	controller.pid_on = 0;
	controller.wg_pid_on = 0;
	controller.tail_on = 0;
	controller.tail_ang = 0;
	controller.tail_run_speed = 0;
		//controller.gyro_offset=610;
	controller.step_offset = 10000;
	controller.gray_offset = 10000;
	controller.color_threshold = 660;
	controller.P_gain=1.0;
	controller.I_gain=1.0;
	controller.D_gain=1.0;

	sensor.light_min=1000;
	sensor.light_max=0;
	sensor.bottle_is_left=0;
	sensor.bottle_is_right=0;

	sensor.threshold_gray = sensor.gray;
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


	if(sensor.touched == ON && eventStatus.touch_status != ON)
	{
		//Touch Event!!
		sendevent(1);
		eventStatus.touch_status = ON;
	}
	else if(sensor.touched == OFF && eventStatus.touch_status != OFF)
	{
		eventStatus.touch_status = OFF;
	}

	if(sensor.light > sensor.black-50 && eventStatus.light_status != LIGHT_STATUS_BLACK)
	{
		//Black Event!!
		sendevent(3);
		eventStatus.light_status = LIGHT_STATUS_BLACK;
	}
	else if(sensor.light < sensor.white+50 && eventStatus.light_status != LIGHT_STATUS_WHITE)
	{
		//White Event!!
		sendevent(2);
		eventStatus.light_status = LIGHT_STATUS_WHITE;
	}

	if( sensor.light-sensor.gray > controller.gray_offset ){
		sendevent(4);
	}

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
			controller.forward=0;//state->value0;
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
	}
}


//****************************************
//return the Left and Right motor power
// from speed and turn for tail running
//***************************************
void tail_run_turn2pwm(S16 tail_run_speed ,float turn ,S8 *_pwm_L, S8 *_pwm_R)
{
	if(tail_run_speed != 0)
	{
		if((tail_run_speed + turn) > 100)
		{
			float _turn_overflow = tail_run_speed + turn - 100;
			*_pwm_R = 100;
			*_pwm_L = (S8)(tail_run_speed - turn - turn_overflow);

		}
		else if(_tail_run_speed-_turn > 100)
		{
			float _turn_overflow = _tail_run_speed-_turn -100;
			*_pwm_L = 100;
			*_pwm_R = (S8)(_tail_run_speed+_turn-_turn_overflow);
		}
		else
		{
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
*/

