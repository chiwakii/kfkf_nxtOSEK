#include <math.h>
//
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
//
#include "Common.h"
#include "NXT_Config.h"
#include "kfkf/kfkfModel.h"
#include "Logger.h"
#include "SensorManager.h"
#include "Controller.h"

/*
===============================================================================================
	macro
===============================================================================================
*/
#define BLUETOOTH

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
State_t EventSensor();
//
void setController(const State_t state);


void gyro_calibration();
void calibration(int *black,int *white,int *gray);
//void tail_run_turn2pwm(S16 _tail_run_speed ,float _turn ,S8 *_pwm_L, S8 *_pwm_R);
S16 calc_angle2encoder(S16 ang);
//S8 calc_variance(U16 *buf,int _len);
void tailstand();



/*
===============================================================================================
	variables
===============================================================================================
*/
static U8 count = 0;
static U32 sum = 0;
static boolean flag_calib = OFF;

int i;
int rest;
int i_value = 0;

///////state machine//////////////////////

Sensor_t sensor;
Controller_t controller;

Event_t *events;
Controller_t controller;

////////balance_control//////////////////////

	S8 init=0;
	S8 turn;
	
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
	case START:	 /* Initialization */
	    display_clear(0);
	    display_goto_xy(0, 1);
	    display_string("Robot Start");
	    display_update();
		InitNXT();
		//to WAIT
		break;
	case WAIT:	 /* Wait */
		if( sensor.touch == ON ) //to BTCOMM
		break;
	case BTCOMM: /* Communication by Bluetooth */
		tailstand();
		receive_BT();
		//to GYROCALIB
		break;
	case GYROCALIB:	 /* Do calibration */
		if( sensor.touch == ON ) flag_calib = 1;

		if(flag_calib == ON)
		{
			count++;
			sum += sensor.gyro;
		}

		if(count >= 100)
		{
			sensor.gyro_offset = (F32)sum/count;
			flag_calib = OFF;
			sum = 0;
			count = 0;
			//to GRAYCALIB
		}
		break;

	case GRAYCALIB:
		if( sensor.touch == ON ) flag_calib = 1;

		if(flag_calib == 1)
		{
			count++;
			sum += sensor.light;
		}

		if(count >= 100)
		{
			sensor.target_gray = (F32)sum/count;
			sensor.target_gray_base = controller.target_gray;
			flag_calib = OFF;
			sum = 0;
			count = 0;
			//to WHITECALIB
		}
		break;

	case WHITECALIB:
		if( sensor.touch == ON ) flag_calib = ON;

		if(flag_calib == ON)
		{
			count++;
			sum += sensor.light;
		}

		if(count >= 100)
		{
			sensor.white = (F32)sum/count;
			flag_calib = OFF;
			sum = 0;
			count = 0;
			//to BLACKCALIB
		}
		break;

	case BLACKCALIB:
		if( sensor.touch == ON ) flag_calib = ON;

		if( flag_calib == ON )
		{
			count++;
			sum += sensor.light;
		}

		if(count >= 100)
		{
			sensor.black = (F32)sum/count;
			flag_calib = OFF;
			sum = 0;
			count = 0;
			//to ACTION
		}
		break;

	case ACTION: /* Challenge contest */
		setController( EventSensor() );
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
	S8 pwm_L = 0;
	S8 pwm_R = 0;
	S8 pwm_T = 0;

	if( controller.PIDmode != NO_MODE )
	{
		controller.pre_dif = controller.dif;

		if( controller.PIDmode == WB_PID )
		{
			controller.dif = sensor.light - sensor.target_gray;
		}
		else if( controller.PIDmode == WG_PID )
		{
			controller.dif = sensor.light - sensor.white_gray_threshold;//?
		}

		controller.differential = controller.dif - controller.pre_dif;
		controller.integral += (controller.dif + controller.pre_dif)/2.0 * 0.004;

		controller.turn = controller.P_gain * controller.dif
				+ controller.I_gain * controller.integral
				+ controller.D_gain * controller.differential;
	}

	controller.tail_pre_dif = controller.tail_dif;
	controller.tail_dif = controller.target_tail - sensor.count_tail;

	//pwm_T = (U8)( controller.TP_gain * controller.tail_dif + controller.TD_gain * (controller.tail_pre_dif - controller.tail_dif) );
	pwm_T = (U8)( controller.TP_gain * controller.tail_dif );

	if(pwm_T > 100)
	{
		pwm_T = 100;
	}
	else if(pwm_T < -100)
	{
		pwm_T = -100;
	}
	
	if( controller.standmode == BALANCE )
	{
		balance_control(
			(F32)controller.forward,
			(F32)controller.turn,
			(F32)sensor.gyro,
			(F32)sensor.gyro_offset,
			(F32)sensor.count_left,
			(F32)sensor.count_right,
			(F32)sensor.battery,
			&pwm_L,
			&pwm_R
		);
	
	}
	else if(controller.standmode == TAIL)
	{
		
		balance_control(
			(F32)controller.forward,
			(F32)controller.turn,
			(F32)sensor.gyro_offset,
			(F32)sensor.gyro_offset,
			(F32)sensor.count_left,
			(F32)sensor.count_right,
			(F32)sensor.battery,
			&pwm_L,
			&pwm_R
		);

	}

	nxt_motor_set_speed( TAIL_MOTOR, pwm_T, 1 );
	nxt_motor_set_speed( LEFT_MOTOR, pwm_L, 1 );
	nxt_motor_set_speed( RIGHT_MOTOR, pwm_R, 1 );

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

#define LIGHT_BUFFER_LENGTH_MAX 250

static U8 g_SonarCnt = 0;
static U8 g_LightCnt = 0;
static U16 lightbuffer[LIGHT_BUFFER_LENGTH_MAX] = {0};
static U32 lightave = 0;

TASK(TaskSensor)
{
	//==========================================
	//	Data Update of Sensor
	//==========================================
	//--------------------------------
	//	Light Data
	//--------------------------------
	// Update Data of Light Sensor
	sensor.light = ecrobot_get_light_sensor(LIGHT_SENSOR);

	lightbuffer[g_LightCnt] = sensor.light;
	g_LightCnt++;
	if( g_LightCnt >= LIGHT_BUFFER_LENGTH_MAX ) g_LightCnt = 0;

	for(i=0;i >= LIGHT_BUFFER_LENGTH_MAX;i++)
	{
		lightave += lightbuffer[i];
	}
	sensor.light_ave = (U16)(lightave / LIGHT_BUFFER_LENGTH_MAX);

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
		sensor.distance = ecrobot_get_sonar_sensor(SONAR_SENSOR);
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
	sensor.touch = ecrobot_get_touch_sensor(TOUCH_SENSOR);


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
	//sensor.light_V = 1.0*calc_variance(sensor.light_buffer,V_LIGHT_BUFFER_LENGTH);
	
	//calculate gyro valiance
	//sensor.gyro_V = 1.0*calc_variance(sensor.gyro_buffer,V_GYRO_BUFFER_LENGTH);


	//Bottle Detecting
	if(eventStatus.bottle_right_length > 0){
		if(sensor.distance < eventStatus.bottle_right_length){
			sensor.bottle_is_right = ON;
		}
	}
	if(eventStatus.bottle_left_length > 0){
		if(sensor.distance < eventStatus.bottle_left_length){
			sensor.bottle_is_left = ON;
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
	//==========================================
	//	initialize motor & balancer
	//==========================================
	balance_init();
	nxt_motor_set_count(RIGHT_MOTOR,0);
	nxt_motor_set_count(LEFT_MOTOR,0);
	nxt_motor_set_count(TAIL_MOTOR,0);

	//==========================================
	//	initialize sensor variables
	//==========================================
	sensor.light = 600;
	//sensor.pre_light = 600;
	sensor.black = 800;
	sensor.white = 400;
	sensor.target_gray = 600;
	sensor.target_gray_base = sensor.target_gray;
	//sensor.threshold_gray = sensor.target_gray;

	sensor.gyro = 600;
	sensor.gyro_offset = 610;
	sensor.gyro_offset_base = sensor.gyro_offset;

	sensor.touch = OFF;
	sensor.distance = 255;
	sensor.count_left = 0;
	sensor.count_right = 0;
	sensor.battery = 600;

	sensor.bottle_is_left = OFF;
	sensor.bottle_is_right = OFF;


	//==========================================
	//	initialize controller variables
	//==========================================
	//controller.speed = 0;
	controller.forward = 0;
	controller.turn = 0;
	controller.standmode = NO_MODE;
	controller.PIDmode = NO_MODE;
	controller.target_tail = 0;
	//controller.tail_run_speed = 0;
	controller.step_offset = 10000;
	controller.gray_offset = 10000;
	controller.color_threshold = 660;
	controller.P_gain = 1.0;
	controller.I_gain = 0.0;
	controller.D_gain = 0.0;

	controller.TP_gain = 0.5;
	controller.TD_gain = 1.0;

	controller.color_threshold = sensor.target_gray;

	sensor.prev_light_value = controller.color_threshold;
/*
	for(int m=0 ; m < sensor.LIGHT_BUFFER_LENGTH ; m++)
	{
		sensor.light_buffer[m]=sensor.target_gray;
	}
	for(int m=0;m<sensor.GYRO_BUFFER_LENGTH;m++)
	{
		sensor.gyro_buffer[m] = controller.gyro_offset;
	}
*/
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
State_t EventSensor(){

	State_t tmp;

	//--------------------------------
	//	Event:auto
	//--------------------------------
	tmp = setNextState(AUTO);


	//--------------------------------
	//	Event:touch
	//--------------------------------
	if(sensor.touch == ON && eventStatus.touch_status == OFF)
	{
		tmp = setNextState(TOUCH);
		eventStatus.touch_status = ON;
	}
	else if(sensor.touch == OFF && eventStatus.touch_status == ON)
	{
		eventStatus.touch_status = OFF;
	}

	//==========================================
	//	black & white
	//==========================================
	if(sensor.light > (sensor.black - 50) && eventStatus.light_status != LIGHT_STATUS_BLACK)
	{
		//--------------------------------
		//	Event:black
		//--------------------------------
		tmp = setNextState(BLACK);
		eventStatus.light_status = LIGHT_STATUS_BLACK;
	}
	else if(sensor.light < (sensor.white + 50) && eventStatus.light_status != LIGHT_STATUS_WHITE)
	{
		//--------------------------------
		//	Event:white
		//--------------------------------
		tmp = setNextState(WHITE);
		eventStatus.light_status = LIGHT_STATUS_WHITE;
	}
	else
	{
		eventStatus.light_status = LIGHT_STATUS_GRAY;
	}

	//--------------------------------
	//	Event:gray marker
	//--------------------------------
	//if( sensor.light_ave > controller.gray_offset ){
	if( controller.gray_offset - 10 < sensor.light_ave && sensor.light_ave < controller.gray_offset + 10  )
	{
		tmp = setNextState(GRAY_MARKER);
		controller.PIDmode = WG_PID;
	}
	else
	{
		controller.PIDmode = WB_PID;
	}

	//--------------------------------
	//	Event:step
	//--------------------------------
	if( abs(sensor.gyro - sensor.gyro_offset) > controller.step_offset	)
	{
		tmp = setNextState(STEP);
	}

	//--------------------------------
	//	Event:sonar
	//--------------------------------
   	if(sensor.distance < eventStatus.distance )
   	{
   		tmp = setNextState(SONAR);
	}

	//--------------------------------
	//	Event:timer
	//--------------------------------
	if( (systick_get_ms() - eventStatus.start_time) > eventStatus.target_time && eventStatus.timer_flag == ON)
	{
		tmp = setNextState(TIMER);
		eventStatus.target_time = 0;
		eventStatus.start_time = 0;
		eventStatus.timer_flag = OFF;
	}

	//--------------------------------
	//	Event:motor count
	//--------------------------------
	int motor_count = (sensor.count_left + sensor.count_right) / 2;
	//int motor_count = (nxt_motor_get_count(LEFT_MOTOR) + nxt_motor_get_count(RIGHT_MOTOR)) / 2;
	if(abs(motor_count - eventStatus.start_motor_count) > abs(eventStatus.target_motor_count) && eventStatus.motor_counter_flag == ON )
	{
		tmp = setNextState(MOTOR_COUNT);
		eventStatus.target_motor_count = 0;
		eventStatus.start_motor_count = 0;
		eventStatus.motor_counter_flag = OFF;
	}

	//--------------------------------
	//	Event:bluetooth start
	//--------------------------------
	boolean bts = BluetoothStart();
	if(eventStatus.BTstart == OFF && bts == ON)
	{
		tmp = setNextState(BT_START);
		eventStatus.BTstart = ON;
	}
	else if(eventStatus.BTstart == ON && bts == OFF)
	{
		eventStatus.BTstart = OFF;
	}

	//--------------------------------
	//	Event:pivot turn
	//--------------------------------
	if( eventStatus.pivot_turn_flag == ON && abs(sensor.count_right - eventStatus.start_pivot_turn_encoder_R) > eventStatus.target_pivot_turn_angle_R )
	{
		tmp = setNextState(PIVOT_TURN_END);
		eventStatus.pivot_turn_flag = OFF;
	}


	//==========================================
	//	drift turn
	//==========================================
	if(eventStatus.bottle_judge == ON)
	{
		//sendevent turn left or right

		if(sensor.bottle_is_right == ON && sensor.bottle_is_left == ON)
		{

		}
		else if(sensor.bottle_is_right == ON && sensor.bottle_is_left == OFF)
		{
			//--------------------------------
			//	Event:bottle is right
			//--------------------------------
			tmp = setNextState(BOTTLE_RIGHT);
		}
		else if(sensor.bottle_is_right == OFF && sensor.bottle_is_left == ON)
		{
			//--------------------------------
			//	Event:bottle is left
			//--------------------------------
			tmp = setNextState(BOTTLE_LEFT);
		}

		eventStatus.bottle_judge = ON;
	}

	return tmp;
}


/*
===============================================================================================
	name: serController
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
void serController(const State_t state)
{
	switch(state.action_no)
	{
		case NOT_TRANSITION:
			break;

		case DO_NOTHING://do nothing
			//controller.speed = 0;
			controller.forward = 0;
			controller.turn = 0;
			//controller.PIDmode = NO_MODE;
			//controller.standmode = BALANCE;
			break;

		case BALANCE_STOP://stop
			//controller.speed = 0;
			controller.forward = 0;
			controller.turn = 0;

			//nxt_motor_set_speed(TAIL_MOTOR,0,1);
			controller.PIDmode = WB_PID;
			controller.standmode = BALANCE;
			//balance_init();
			//nxt_motor_set_count(LEFT_MOTOR,0);
			//nxt_motor_set_count(RIGHT_MOTOR,0);
			break;

		// linetrace
		//@param foward:=value0
		//@param gyro_offset:=value1
		case BALANCE_LINETRACE:
			//controller.speed = state.value0;
			controller.forward = state.value0;
			sensor.gyro_offset = sensor.gyro_offset_base + state.value1;

			controller.PIDmode = WB_PID;
			controller.standmode = BALANCE;


			//sensor.target_gray = sensor.threshold_gray;
			//nxt_motor_set_speed(TAIL_MOTOR,0,1);

			break;

		//change the gray threshold
		//@param new threshold:=value0
		case CHANGE_GRAY:
			//controller.color_threshold=state.value0;
			//sensor.gray=state.value0;
			//sensor.gray = sensor.calib_gray;
			sensor.target_gray = sensor.target_gray_base + state.value0;

			//controller.PIDmode = WB_PID;
			//controller.speed = 0;
			//controller.forward_power = 0;
			//controller.turn = 0;
			break;

		//run with no linetrace without balance
		//@param target_tail:=value0
		//@param tail_run_speed :=value1
		//@param turn :=value2
		case TAIL_RUN_FREEDOM:
			controller.target_tail = state.value0;
			//controller.tail_run_speed = state.value1;
			controller.forward = state.value1;
			controller.turn = state.value2;
			controller.TP_gain = state.value3;

			controller.PIDmode = NO_MODE;
			controller.standmode = TAIL;
			break;

		//set timer
		//@param limit_timer:=value0 i.e. 20 = 2.0sec
		case TIMER_SET:
			eventStatus.timer_flag = ON;
			eventStatus.start_time = systick_get_ms();
			eventStatus.target_time = state.value0 * 100;

			break;
		//set motor count
		case MOTOR_SET:
			//eventStatus.start_motor_count = (nxt_motor_get_count(LEFT_MOTOR) + nxt_motor_get_count(RIGHT_MOTOR)) / 2;
			eventStatus.start_motor_count = (sensor.count_left + sensor.count_right) / 2;
			eventStatus.target_motor_count = state.value0;

			break;

		//set PID
		//@param P_gain:=value0/100
		//@param I_gain:=value1/100
		//@param D_gain:=value2/100
		case PID_SET:

			controller.P_gain = (F32)state.value0 / 100;
			controller.I_gain = (F32)state.value1 / 100;
			controller.D_gain = (F32)state.value2 / 100;

			break;

		//set gyro offset for steps
		//@param step_offset := value0
		case STEP_OFFSET_SET:
			controller.step_offset = state.value0;
			//sensor.GYRO_BUFFER_LENGTH = state.value1;
			break;

		//up the tail
		case RAISE_TAIL:
			controller.target_tail = 0;
			//nxt_motor_set_speed(TAIL_MOTOR,-15,1);
			break;

		// tail run
		//@param angle
		//@param speed
 		case TAIL_LINETRACE:
			controller.PIDmode = WB_PID;
			controller.standmode = TAIL;
			controller.target_tail = state.value0;
			//controller.tail_run_speed = state.value1;
			controller.forward = state.value1;
			controller.TP_gain = state.value2;
			break;

		//circling
		//@param angle to turn
		case PIVOT_TURN:
			//controller.tail_on=1;
			//controller.balance_on=0;
			controller.PIDmode = NO_MODE;
			controller.forward = 0;
			//controller.tail_run_speed=1;
			/*if(controller.balance_flag == ON)
			{
				controller.turn = state.value1;
			}
			else
			{
				controller.turn = -(state.value1);
			}*/

			if( state.value0 > 0 )
			{
				controller.turn = state.value1;
			}
			else
			{
				controller.turn = -(state.value1);
			}

			/*if(state.value0 < 0){
				controller.turn *= -1;
			}*/
			eventStatus.start_pivot_turn_encoder_R = sensor.count_right;
			eventStatus.target_pivot_turn_angle_R = calc_angle2encoder(state.value0);
			eventStatus.pivot_turn_flag = ON;
			break;

		//selecting logger
		//@param log_type
		case SELECT_LOGTYPE:
			logger.type = state.value0;
			break;

		//set the gray_market offset
		//@param gray_offset
		case GRAY_MARKER_OFFSET:
			controller.gray_offset = state.value0;
			//sensor.LIGHT_BUFFER_LENGTH = state.value1;

			break;

		//free balance
		case BALANCE_RUN_FREEDOM:
			controller.PIDmode = NO_MODE;
			controller.standmode = BALANCE;

			controller.forward = state.value0;
			controller.turn = state.value1;
			sensor.gyro_offset = sensor.gyro_offset_base + state.value2;

			//nxt_motor_set_speed(TAIL_MOTOR,0,1);

			break;

		//set_sonar_sensor
		case SONAR_SET:
			eventStatus.target_distance = state.value0;
			break;

		case SERACH_BOTTLE_RIGHT:
			controller.forward = 0;

			eventStatus.bottle_right_length = state.value0;
			eventStatus.bottle_right_flag = ON;
			break;

		case SEARCH_BOTTLE_LEFT:
			controller.forward = 0;

			eventStatus.bottle_left_length = state.value0;
			eventStatus.bottle_left_flag = ON;

			break;

		case SEARCH_BOTTLE_END:
			eventStatus.bottle_right_length = 0;
			eventStatus.bottle_left_length = 0;
			break;

		case SEARCH_BOTTLE_JUDGE:
			eventStatus.bottle_judge = ON;
			break;
	}
}


//****************************************
//return the Left and Right motor power
// from speed and turn for tail running
//***************************************
/*
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
*/
/*
===============================================================================================
	name: calc_angle2encoder
	Description: ??
 	---
	update: 2013.06.17
===============================================================================================
*/
S16 calc_angle2encoder(S16 ang){
	S16 ret = (S16)((F32)ang*16.3/8.1);
	if(ret < 0) ret *= -1;

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

/*
===============================================================================================
	name: tailstand
	Description: ??
 	---
	update: 2013.06.17
===============================================================================================
*/
void tailstand(){
	controller.target_tail = 90;
}
