/*
####################################################################################################
	name: MainFunction.c
	Description: ??
	---
	update: 2013.06.13
####################################################################################################
*/

#include <math.h>
//
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
//
#include "NXT_Config.h"
//#include "kfkf/kfkfModel.h"
#include "RobotManager.h"
#include "Logger.h"
#include "SensorManager.h"
#include "Controller.h"


DeclareCounter(SysTimerCnt);
DeclareTask(TaskMain);					/* Task to manage behavior of robot */
DeclareTask(TaskActuator);				/* Task to actuate */
DeclareTask(TaskSensor);				/* Task to sense */
DeclareTask(TaskLogger);				/* Task to send logging data */


#define BLUETOOTH


//we should make these valiables dynamic valiables

int i;
int rest;
int i_value = 0;
///////state machine//////////////////////

//StateMachine_t statemachine;
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
}


//*****************************************************************************
// user_1ms_isr_type2
//*****************************************************************************

void user_1ms_isr_type2(void){
	SignalCounter(SysTimerCnt);   
}


/*
###################################################################
	Task
	name: TaskMain
	description:

###################################################################
*/
TASK(TaskMain)
{

	switch(/*robot state variable*/)
	{
	case START:	 /* Start State */
		break;
	case WAIT:	 /* Wait and Initialize */
		break;
	case BTCOMU: /* Comunication by Bluetooth */
		receive_BT();
		break;
	case CALIB:	 /* Do calibration */
		gyro_calibration();
		calibration(&sensor.black,&sensor.white,&sensor.gray);
		break;
	case ACTION: /* Challenge contest */
		event_manager();
		break;
	}

	TerminateTask();					
}


/*
###################################################################
	Task
	name: TaskActuator
	description:

###################################################################
*/
TASK(TaskActuator)
{

	int before=0, standard=0;	// beforeAEAE?AﾂｧAac?AEAAeA竏羨?AAstandardAEoAeaAuR?AAAAeA竏羨
	float integral=0;

	if(controller.pid_on == 1)	// Linetrace border between black and white
	{
		before = standard;
		standard = sensor.light - sensor.gray;					// AAeA竏羨?CiAenAaAE
		controller.integral += (standard - before)/2.0 * 0.004;	

		controller.turn = controller.P_gain * standard *100 / (sensor.black-sensor.white) + controller.I_gain * integral / (sensor.black-sensor.white) * 100 + controller.D_gain * (sensor.light-sensor.prev_light_value) / (sensor.black-sensor.white) * 100;	// EoaAouAAﾂｧERaAAAE
		sensor.prev_light_value = sensor.light;		//1?AﾂｧAac?AEEoe?Ca?Ai?CiE窶�A\AE
		controller.forward_power = controller.speed;
	}
	else if(controller.wg_pid_on == 1)	// Linetrace border between gray and white
	{
		sensor.light = ecrobot_get_light_sensor(LIGHT_SENSOR);
		before = standard;
		standard = sensor.light - sensor.white_gray_threshold;
		controller.integral += (standard - before)/2.0 * 0.004;

		controller.turn = controller.P_gain * standard *100 / (sensor.calib_gray-sensor.white) + controller.I_gain * integral / (sensor.calib_gray-sensor.white) * 100 + controller.D_gain * (sensor.light-sensor.prev_light_value) / (sensor.calib_gray-sensor.white) * 100;	// 譌句屓蛟､險�E
		sensor.prev_light_value = sensor.light;
		controller.forward_power = controller.speed;
	}
	else
	{

	}

	
	if(controller.balance_on == 1)
	{
		balance_control(					//?Ee?Ec?E??CﾎｼAPI?AAAeo?A?Aa竏ｫ?AAE
			(float)controller.forward_power,		//AacEA?AEeAaaEAAAeﾎｩ窶ｰaﾂｧ(-100(Ae?EoaAouEuAAﾂｧs)?100(A竏堕ｶEoaAouEuAAﾂｧs))
			(float)controller.turn,			//EoaAouAeﾎｩ窶ｰaﾂｧ(-100?100)
			(float)ecrobot_get_gyro_sensor(NXT_PORT_S1),	//?C??E�｡?Cﾂｧ?E竕�Ca?E??CﾎｼAAﾂｧ
			(float)controller.gyro_offset,				//?C??E�｡?Cﾂｧ?E竕�Ca?E??Cﾎｼ?C??Ei?Ca?EAEEaAAﾂｧ
			(float)nxt_motor_get_count(NXT_PORT_C),		//A竏堕ｶ?E��Eo?Co?CR?E??C??Eo?EAAAﾂｧ
			(float)nxt_motor_get_count(NXT_PORT_B),		//Ae??E��Eo?Co?CR?E??C??Eo?EAAAﾂｧ
			(float)ecrobot_get_battery_voltage(),		//?Ee?EE?EAEE?EoaAusAAﾂｧ[mV]

			&pwm_L,						//A竏堕ｶ?E��Eo?CoPWMAa竏ｫAaoAAﾂｧAEaEaa?CaAAﾂｧAEAE
			&pwm_R						//Ae??E��Eo?CoPWMAa竏ｫAaoAAﾂｧAEaEaa?CaAAﾂｧAEAE

		);

		nxt_motor_set_speed(NXT_PORT_C, (S8)(pwm_L*controller.left_motor_rate), 1);		//?E��Eo?Co?AﾂｴEaAEaﾂｧ?CiAAE?AAE
		nxt_motor_set_speed(NXT_PORT_B, (S8)(pwm_R*controller.right_motor_rate), 1);
	
	}
	else if(controller.tail_on==1)
	{
		U32 _tail_ang = nxt_motor_get_count(TAIL_MOTOR);
		U8 _tail_speed;
	
		if(_tail_ang>=0)
		{
			_tail_speed = (U8)(10.0/9.0*(controller.tail_ang - _tail_ang)+controller.tail_speed_offset);
		}
		else
		{
			_tail_speed=10;
		}
	
		

		if(_tail_ang > controller.tail_ang+1)
		{
			nxt_motor_set_speed(TAIL_MOTOR,-50,1);
		}
		else if(_tail_ang < controller.tail_ang-1)
		{
			nxt_motor_set_speed(TAIL_MOTOR,_tail_speed,1);
		}
		else
		{
			nxt_motor_set_speed(TAIL_MOTOR,0,1);
		}
		
		tail_run_turn2pwm(
				(S16) controller.tail_run_speed,
				(float) controller.turn,
				&pwm_L,
				&pwm_R
		);

		nxt_motor_set_speed(RIGHT_MOTOR,(S8)(pwm_R*controller.right_motor_rate),1);
		nxt_motor_set_speed(LEFT_MOTOR,(S8)(pwm_L*controller.left_motor_rate),1);
	}

	TerminateTask();


}

/*
###################################################################
	Task
	name: TaskSensor
	description:

###################################################################
*/

int g_LightCnt = 1;
int g_GyroCnt = 1;
int g_SonarCnt = 0;

TASK(TaskSensor)
{
	//==========================================
	//	Data Update of Sensor
	//==========================================
	//--------------------------------
	//	Light Data
	//--------------------------------
	sensor.prev_light_value = sensor.light;
	sensor.light_buffer[g_LightCnt] = sensor.prev_light_value;
	g_LightCnt++;
	if(g_LightCnt >= sensor.LIGHT_BUFFER_LENGTH){
		g_LightCnt = 0;
	}

	// Update Data of Light Sensor
	sensor.light = ecrobot_get_light_sensor(LIGHT_SENSOR);

	//--------------------------------
	//	Gyro Data
	//--------------------------------
	sensor.prev_gyro_value= sensor.gyro;
	sensor.gyro_buffer[g_GyroCnt] = sensor.prev_gyro_value;
	g_GyroCnt++;
	if(g_GyroCnt >= sensor.GYRO_BUFFER_LENGTH){
		g_GyroCnt = 0;
	}

	// Update Data of Gyro Sensor
	sensor.gyro= ecrobot_get_gyro_sensor(GYRO_SENSOR);

	//--------------------------------
	//	Sonar Data
	//--------------------------------
	if(g_SonarCnt > 10){
		// Update Data of Sonar Sensor
		sensor.sonar = ecrobot_get_sonar_sensor(SONAR_SENSOR);
		g_SonarCnt = 0;
	}else{
		g_SonarCnt++;
	}

	//--------------------------------
	//	Touch Data
	//--------------------------------
	// Update Data of Touch Sensor
	sensor.touched = ecrobot_get_touch_sensor(TOUCH_SENSOR);


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

	S8 _ang = nxt_motor_get_count(RIGHT_MOTOR)-eventStatus.circling_start_encoder_R -eventStatus.circling_target_angle_R;
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
				ecrobot_bt_data_logger(sensor.gyro_V,sensor.light_V);
				break;

		}
		TerminateTask();
}

