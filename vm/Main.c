#include <math.h>
#include <stdlib.h>

#include "Common.h"

#include "balancer.h"
#include "NXT_Config.h"
#include "Sensor.h"
#include "Actuator.h"

#include "kfkf_Bluetooth/kfkfModel.h"
#include "kfkf_Bluetooth/Logger.h"

/*======================================*/
/*	タスク宣言							*/
/*======================================*/
DeclareCounter(SysTimerCnt);
DeclareTask(TaskMain);
DeclareTask(TaskSensor);
DeclareTask(TaskActuator);
DeclareTask(TaskLogger);

/*======================================*/
/*	列挙型定義							*/
/*======================================*/
typedef enum _MainTaskState
{
	INIT,
	BTCOMM,
	TARGETCALIB,
	WHITECALIB,
	BLACKCALIB,
	ACTION
}MainTaskState_e;

/*======================================*/
/*	プロトライプ宣言					*/
/*======================================*/
void InitNXT(void);
//
void EventSensor(void);
//
void setController(void);

int calcAngle2Encoder(S16 ang);

/*======================================*/
/*	変数宣言							*/
/*======================================*/
S8 g_pwm_L = 0;
S8 g_pwm_R = 0;
S8 g_pwm_T = 0;

/*--------------------------*/
/*	センサー用				*/
/*--------------------------*/
static Sensor_t g_Sensor;

/*--------------------------*/
/*	コントローラー用		*/
/*--------------------------*/
/* Event status */
static Controller_t g_Controller;

/*--------------------------*/
/*	アクチュエーター用		*/
/*--------------------------*/
static Actuator_t g_Actuator;

/*--------------------------*/
/*	ロガー用				*/
/*--------------------------*/
static LogType_e g_LogType = LOG_NO;

/*==================================================*/
/*	Hook関数: ecrobot_device_initialize				*/
/*==================================================*/
void ecrobot_device_initialize()
{
	/* 初期化:Motor */
	nxt_motor_set_speed( LEFT_MOTOR, 0, 0);
	nxt_motor_set_speed( RIGHT_MOTOR, 0, 0);
	nxt_motor_set_speed( TAIL_MOTOR, 0, 0);
	nxt_motor_set_count( RIGHT_MOTOR, 0);
	nxt_motor_set_count( LEFT_MOTOR, 0);
	nxt_motor_set_count( TAIL_MOTOR, 0);

	/* 初期化:Sensor(Sonar) */
	ecrobot_init_sonar_sensor( SONAR_SENSOR );

	/* 初期化:Bluetooth device */
	ecrobot_init_bt_slave( BT_PASS_KEY );

	/* 赤色LED:ON */
	ecrobot_set_light_sensor_active( LIGHT_SENSOR );

	InitNXT();
}

/*==================================================*/
/*	Hook関数: ecrobot_device_terminate				*/
/*==================================================*/
void ecrobot_device_terminate()
{
	/* 終了処理: Motor */
	nxt_motor_set_speed( LEFT_MOTOR, 0, 0);
	nxt_motor_set_speed( RIGHT_MOTOR, 0, 0);
	nxt_motor_set_speed( TAIL_MOTOR, 0, 0);
	nxt_motor_set_count( RIGHT_MOTOR, 0);
	nxt_motor_set_count( LEFT_MOTOR, 0);
	nxt_motor_set_count( TAIL_MOTOR, 0);

	/* 終了処理: Sensor(Sonar) */
	ecrobot_term_sonar_sensor( SONAR_SENSOR );

	/* 終了処理: Bluetooth device */
	ecrobot_term_bt_connection();

	/* 赤色LED:OFF */
	ecrobot_set_light_sensor_inactive( LIGHT_SENSOR );

	InitNXT();
}

/*==================================================*/
/*	Hook関数: user_1ms_isr_type2					*/
/*==================================================*/
void user_1ms_isr_type2(void)
{
	SignalCounter(SysTimerCnt);   
}



/************************************************************/
/*	Task													*/
/************************************************************/

/*
===============================================================================================
	name: TaskMain
	Description: ロボットのメインタスク
===============================================================================================
*/

/*==================================================*/
/*	変数											*/
/*==================================================*/
static U8 g_CalibCnt = 0;
static U32 g_CalibGyroSum = 0;
static U32 g_CalibLightSum = 0;
static U8 g_CalibFlag = 0;
static MainTaskState_e g_MTState = INIT;

/*==================================================*/
/*	タスク											*/
/*==================================================*/
TASK(TaskMain)
{

	switch(g_MTState)
	{
		/*--------------------------*/
		/*	初期化					*/
		/*--------------------------*/
		case INIT:
			display_clear(0);
			display_goto_xy(0, 0);
			display_string("Prep:FALSE");
	        display_goto_xy(1, 1);
	        display_string("BT:FALSE");
			display_update();
			ecrobot_sound_tone(880, 50, 30);

			InitNXT();
			g_Actuator.target_tail = 110;

			/* 状態遷移: BTCOMM */
			g_MTState = BTCOMM;
			break;

		/*--------------------------*/
		/*	kfkfモデル受信			*/
		/*--------------------------*/
		case BTCOMM:
			if(ReceiveBT() == 1)
			{

		        display_clear(0);
				display_goto_xy(0, 0);
				display_string("Prep:FALSE");
		        display_goto_xy(1, 1);
		        display_string("BT:TRUE");
		        display_goto_xy(1, 2);
		        display_string("CLBGray:FALSE");
		        display_update();
		        ecrobot_sound_tone(880, 50, 30);

		        /* 状態遷移: TARGETCALIB */
				g_MTState = TARGETCALIB;
			}

			break;

		/*----------------------------------------------*/
		/*	キャリブレーション(ライン境目&ジャイロ)		*/
		/*----------------------------------------------*/
		case TARGETCALIB:
			if( g_Sensor.touch == 1 )
			{
				ecrobot_sound_tone(440, 50, 30);
				g_CalibFlag = 1;
			}

			if(g_CalibFlag == 1)
			{
				g_CalibCnt++;
				g_CalibGyroSum += g_Sensor.gyro;
				g_CalibLightSum += g_Sensor.light;
			}

			if(g_CalibCnt >= 100)
			{
				g_Actuator.gyro_offset = (U16)(g_CalibGyroSum / g_CalibCnt);
				g_Actuator.target_gray = (U16)(g_CalibLightSum / g_CalibCnt);
				g_Actuator.target_gray_base = g_Actuator.target_gray;
				g_CalibFlag = 0;
				g_CalibGyroSum = 0;
				g_CalibLightSum = 0;
				g_CalibCnt = 0;

		        display_clear(0);
				display_goto_xy(0, 0);
				display_string("Prep:FALSE");
		        display_goto_xy(1, 1);
		        display_string("BT:TRUE");
		        display_goto_xy(1, 2);
		        display_string("CLBGray:TRUE");
		        display_goto_xy(1, 2);
		        display_string("CLBGray:TRUE");
		        display_goto_xy(1, 3);
		        display_string("CLBWhite:FALSE");
		        display_update();
		        ecrobot_sound_tone(880, 50, 30);

		        /* 状態遷移: WHITECALIB */
				g_MTState = WHITECALIB;
			}
			break;

		/*----------------------------------------------*/
		/*	キャリブレーション(白色)					*/
		/*----------------------------------------------*/
		case WHITECALIB:
			if( g_Sensor.touch == 1 )
			{
				ecrobot_sound_tone(440, 50, 30);
				g_CalibFlag = 1;
			}

			if(g_CalibFlag == 1)
			{
				g_CalibCnt++;
				g_CalibLightSum += g_Sensor.light;
			}

			if(g_CalibCnt >= 100)
			{
				g_Actuator.white = (U16)(g_CalibLightSum / g_CalibCnt);
				g_CalibFlag = 0;
				g_CalibLightSum = 0;
				g_CalibCnt = 0;

		        display_clear(0);
				display_goto_xy(0, 0);
				display_string("Prep:FALSE");
		        display_goto_xy(1, 1);
		        display_string("BT:TRUE");
		        display_goto_xy(1, 2);
		        display_string("CLBGray:TRUE");
		        display_goto_xy(1, 2);
		        display_string("CLBGray:TRUE");
		        display_goto_xy(1, 3);
		        display_string("CLBWhite:TRUE");
		        display_goto_xy(1, 4);
		        display_string("CLBBlack:FALSE");
		        display_update();
		        ecrobot_sound_tone(880, 50, 30);

		        /* 状態遷移: BLACKCALIB */
				g_MTState =  BLACKCALIB;
			}
			break;

		/*----------------------------------------------*/
		/*	キャリブレーション(黒色)					*/
		/*----------------------------------------------*/
		case BLACKCALIB:
			if( g_Sensor.touch == 1 )
			{
				ecrobot_sound_tone(440, 50, 30);
				g_CalibFlag = 1;
			}

			if( g_CalibFlag == 1 )
			{
				g_CalibCnt++;
				g_CalibLightSum += g_Sensor.light;
			}

			if(g_CalibCnt >= 100)
			{
				g_Actuator.black = (U16)(g_CalibLightSum / g_CalibCnt);
				g_CalibFlag = 0;
				g_CalibLightSum = 0;
				g_CalibCnt = 0;

		        display_clear(0);
				display_goto_xy(0, 0);
				display_string("Prep:TRUE");
		        display_goto_xy(1, 1);
		        display_string("BT:TRUE");
		        display_goto_xy(1, 2);
		        display_string("CLBGray:TRUE");
		        display_goto_xy(1, 2);
		        display_string("CLBGray:TRUE");
		        display_goto_xy(1, 3);
		        display_string("CLBWhite:TRUE");
		        display_goto_xy(1, 4);
		        display_string("CLBBlack:TRUE");
		        display_goto_xy(0, 5);
		        display_string("kfkfModel:ON");
		        display_update();
		        ecrobot_sound_tone(880, 50, 30);

		        /* 状態遷移: ACTION */
				g_MTState = ACTION;
			}
			break;

		/*----------------------------------------------*/
		/*	kfkfモデル動作								*/
		/*----------------------------------------------*/
		case ACTION:
			EventSensor();
			setController();

			if( ecrobot_is_ENTER_button_pressed() == 1 )
			{
				/* 状態遷移: INIT */
				g_MTState = INIT;
			}
			break;
	}

	/* タスク終了 */
	TerminateTask();
}

/*
===============================================================================================
	name: TaskSensor
	Description: センシング
===============================================================================================
*/
#define LIGHT_BUFFER_LENGTH_MAX 250

/*==================================================*/
/*	変数											*/
/*==================================================*/
static U8 g_SonarCnt = 0;
static U8 g_LightCnt = 0;
static U16 g_LightBuffer[ LIGHT_BUFFER_LENGTH_MAX ] = {0};
static U32 g_LightAve = 0;

/*==================================================*/
/*	タスク											*/
/*==================================================*/
TASK(TaskSensor)
{
	U8 i = 0;

	/*----------------------*/
	/*	光センサー			*/
	/*----------------------*/
	g_Sensor.light = ecrobot_get_light_sensor(LIGHT_SENSOR);

	/* 平均計算 */
	g_LightBuffer[g_LightCnt] = g_Sensor.light;
	if( ++g_LightCnt >= LIGHT_BUFFER_LENGTH_MAX )
	{
		g_LightCnt = 0;
	}
	for(i=0;i < LIGHT_BUFFER_LENGTH_MAX;i++)
	{
		g_LightAve += g_LightBuffer[i];
	}
	g_Sensor.light_ave = (U16)((F32)g_LightAve / LIGHT_BUFFER_LENGTH_MAX);

	/*----------------------*/
	/*	ジャイロセンサー	*/
	/*----------------------*/
	g_Sensor.gyro = ecrobot_get_gyro_sensor(GYRO_SENSOR);


	/*----------------------*/
	/*	超音波センサー		*/
	/*----------------------*/
	/* 40msec毎に検知 */
	if(++g_SonarCnt >= 10)
	{
		g_Sensor.distance = ecrobot_get_sonar_sensor(SONAR_SENSOR);
		g_SonarCnt = 0;
	}

	/*----------------------*/
	/*	タッチセンサー		*/
	/*----------------------*/
	g_Sensor.touch = ecrobot_get_touch_sensor(TOUCH_SENSOR);


	/*----------------------*/
	/*	モータカウント		*/
	/*----------------------*/
	g_Sensor.count_left = nxt_motor_get_count(LEFT_MOTOR);
	g_Sensor.count_right = nxt_motor_get_count(RIGHT_MOTOR);
	g_Sensor.count_tail = nxt_motor_get_count(TAIL_MOTOR);


	/*----------------------*/
	/*	バッテリー			*/
	/*----------------------*/
	g_Sensor.battery = ecrobot_get_battery_voltage();


	/*----------------------*/
	/*	パケット受信		*/
	/*----------------------*/
	getBluetooth();


	/*----------------------*/
	/*	物体センサー		*/
	/*----------------------*/
	/* 右側物体 */
	if( g_Controller.object_right_flag == 1 )
	{
		if(g_Sensor.distance < g_Controller.object_right_length)
		{
			g_Sensor.object_is_right = 1;
		}
	}

	/* 左側物体 */
	if ( g_Controller.object_left_flag == 1 )
	{
		if(g_Sensor.distance < g_Controller.object_left_length)
		{
			g_Sensor.object_is_left = 1;
		}
	}


	/*----------------------*/
	/*	タスク終了			*/
	/*----------------------*/
	TerminateTask();
}


/*
===============================================================================================
	name: TaskActuator
	Description: ??
===============================================================================================
*/
TASK(TaskActuator)
{

	/*--------------------------*/
	/*	PWMの初期化				*/
	/*--------------------------*/
	g_pwm_L = 0;
	g_pwm_R = 0;
	g_pwm_T = 0;


	/*--------------------------*/
	/*	旋回値(turn)の計算		*/
	/*--------------------------*/
	/* トレースを行う場合 */
	if( g_Actuator.TraceMode != 0 )
	{
		g_Actuator.pre_dif = g_Actuator.dif;

		if( g_Actuator.TraceMode == 1 )
		{
			g_Actuator.dif = g_Sensor.light - g_Actuator.target_gray;
		}
		else if( g_Actuator.TraceMode == 2 )
		{
			/* 未定 */
		}

		g_Actuator.differential = g_Actuator.dif - g_Actuator.pre_dif;
		g_Actuator.integral += (F32)(g_Actuator.dif + g_Actuator.pre_dif)/2.0 * 0.004;

		g_Actuator.turn = (S8)(g_Actuator.P_gain * g_Actuator.dif
				+ g_Actuator.I_gain * g_Actuator.integral
				+ g_Actuator.D_gain * g_Actuator.differential);
	}

	/*--------------------------*/
	/*	PWMの計算				*/
	/*--------------------------*/
	if( g_Actuator.StandMode == 1 )
	{
		balance_control(
			(F32)g_Actuator.forward,
			(F32)g_Actuator.turn,
			(F32)g_Sensor.gyro,
			(F32)g_Actuator.gyro_offset,
			(F32)g_Sensor.count_left,
			(F32)g_Sensor.count_right,
			(F32)g_Sensor.battery,
			&g_pwm_L,
			&g_pwm_R
		);

	}
	else if( g_Actuator.StandMode == 2 )
	{

		balance_control(
			(F32)g_Actuator.forward,
			(F32)g_Actuator.turn,
			(F32)g_Actuator.gyro_offset,
			(F32)g_Actuator.gyro_offset,
			(F32)g_Sensor.count_left,
			(F32)g_Sensor.count_right,
			(F32)g_Sensor.battery,
			&g_pwm_L,
			&g_pwm_R
		);

	}


	/*--------------------------*/
	/*	しっぽの計算			*/
	/*--------------------------*/
	g_Actuator.tail_pre_dif = g_Actuator.tail_dif;
	g_Actuator.tail_dif = g_Actuator.target_tail - g_Sensor.count_tail;

	g_pwm_T = (S8)( g_Actuator.TP_gain * g_Actuator.tail_dif );

	if(g_pwm_T > 100)
	{
		g_pwm_T = 100;
	}
	else if(g_pwm_T < -100)
	{
		g_pwm_T = -100;
	}


	/*--------------------------*/
	/*	PWMのセット				*/
	/*--------------------------*/
	nxt_motor_set_speed( TAIL_MOTOR, g_pwm_T, 1 );
	nxt_motor_set_speed( LEFT_MOTOR, g_pwm_L, 1 );
	nxt_motor_set_speed( RIGHT_MOTOR, g_pwm_R, 1 );


	/*--------------------------*/
	/*	タスクの終了			*/
	/*--------------------------*/
	TerminateTask();
}


/*
###################################################################
	Task
	name: TaskLogger
	description:
###################################################################
*/
TASK(TaskLogger)
{
	//==========================================
	//  Calculation for logging
	//==========================================
	S8 ang = g_Sensor.count_right - g_Controller.start_pivot_turn_encoder_R - g_Controller.target_pivot_turn_angle_R;
	int rest_motor_count = (g_Sensor.count_left + g_Sensor.count_right) / 2 - g_Controller.start_motor_count - g_Controller.target_motor_count;
	
	//==========================================
	//  Select log type
	//==========================================
	switch(g_LogType)
	{
		case LOG_STATE:
			ecrobot_bt_data_logger( (S8)getCurrentStateNum(), 0 );
			break;

		case LOG_TURN:
			ecrobot_bt_data_logger( (S8)g_Actuator.turn, 0 );
			break;

		case LOG_PWM:
			ecrobot_bt_data_logger( (S8)g_pwm_L, (S8)g_pwm_R );
			break;

		case LOG_TARGET_ANGLE:
			ecrobot_bt_data_logger( (S8)(ang/100), (S8)(ang%100) );
			break;

		case LOG_MOTOR_COUNT:
			ecrobot_bt_data_logger( (S8)(rest_motor_count%100), (S8)(rest_motor_count/100) );
			break;

		case LOG_SONAR:
			ecrobot_bt_data_logger( (S8)(g_Sensor.distance%100), (S8)(g_Sensor.distance/100) );
			break;

		case LOG_DT:
			ecrobot_bt_data_logger( (S8)(g_Sensor.object_is_right), (S8)(g_Sensor.object_is_left) );
			break;

		case LOG_BALANCE_TAIL:
			if( g_Actuator.StandMode == 1 )
			{
				ecrobot_bt_data_logger( (S8)0, (S8)1 );
			}
			else if( g_Actuator.StandMode == 2 )
			{
				ecrobot_bt_data_logger( (S8)1, (S8)0 );
			}
			else
			{
				ecrobot_bt_data_logger( (S8)0, (S8)0 );
			}
			break;

		default:
			break;

	}

		//==========================================
		//  Termination of Task
		//==========================================
		TerminateTask();
}


/************************************************/
/*	Function									*/
/************************************************/
/*
===============================================================================================
	name: InitNXT
	Description: ??
 	Parameter: no
	Return Value: no
===============================================================================================
*/
void InitNXT()
{
	U8 i = 0;

	//==========================================
	//	StateMachine
	//==========================================
	InitKFKF();

	//==========================================
	//	Global variables
	//==========================================
	g_CalibCnt = 0;
	g_CalibGyroSum = 0;
	g_CalibLightSum = 0;
	g_CalibFlag = 0;

	g_SonarCnt = 0;
	g_LightCnt = 0;
	g_LightAve = 0;
	for(i=0;i<LIGHT_BUFFER_LENGTH_MAX;i++)
	{
		g_LightBuffer[i] = 0;
	}

	//==========================================
	//	Motor & balancer
	//==========================================
	balance_init();
	nxt_motor_set_speed( LEFT_MOTOR, 0, 0);
	nxt_motor_set_speed( RIGHT_MOTOR, 0, 0);
	nxt_motor_set_speed( TAIL_MOTOR, 0, 0);
	nxt_motor_set_count( RIGHT_MOTOR, 0);
	nxt_motor_set_count( LEFT_MOTOR, 0);
	nxt_motor_set_count( TAIL_MOTOR, 0);

	//==========================================
	//	Sensor variables
	//==========================================
	g_Sensor.light = 600;
	//g_Sensor.pre_light = 600;
	g_Sensor.gyro = 600;
	g_Sensor.touch = 0;
	g_Sensor.distance = 255;
	g_Sensor.count_left = 0;
	g_Sensor.count_right = 0;
	g_Sensor.battery = 600;

	g_Sensor.object_is_left = 0;
	g_Sensor.object_is_right = 0;

	g_Sensor.BTstart = 0;


	//==========================================
	//	Controller variables
	//==========================================
	g_Actuator.forward = 0;
	g_Actuator.turn = 0;
	g_Actuator.black = 800;
	g_Actuator.white = 400;
	g_Actuator.target_gray = 600;
	g_Actuator.target_gray_base = g_Actuator.target_gray;
	//g_Actuator.threshold_gray = g_Actuator.target_gray;
	g_Actuator.gyro_offset = 610;
	g_Actuator.gyro_offset_base = g_Actuator.gyro_offset;
	g_Actuator.StandMode = 0;
	g_Actuator.TraceMode = 0;
	g_Actuator.target_tail = 0;
	g_Actuator.step_offset = 10000;
	g_Actuator.gray_offset = 10000;
	g_Actuator.color_threshold = 660;
	g_Actuator.P_gain = 1.0;
	g_Actuator.I_gain = 0.0;
	g_Actuator.D_gain = 0.0;

	g_Actuator.TP_gain = 0.8;

	g_Actuator.color_threshold = g_Actuator.target_gray;

	//g_Actuator.prev_light_value = g_Actuator.color_threshold;

	//==========================================
	//	Event Status variables
	//==========================================
	g_Controller.touch_status = 0;
	g_Controller.light_status = 0;
	g_Controller.target_distance = 0;
	g_Controller.timer_flag = 0;
	g_Controller.start_time = 0;
	g_Controller.target_time = 0;
	g_Controller.motor_counter_flag = 0;
	g_Controller.start_motor_count = 0;
	g_Controller.target_motor_count = 0;
	g_Controller.BTstart_status = 0;
	g_Controller.pivot_turn_flag = 0;
	g_Controller.start_pivot_turn_encoder_R = 0;
	g_Controller.target_pivot_turn_angle_R = 0;
	g_Controller.object_left_flag = 0;
	g_Controller.object_right_flag = 0;
	g_Controller.object_left_length = 0;
	g_Controller.object_right_length = 0;
	g_Controller.object_judge = 0;
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
===============================================================================================
*/
void EventSensor(){

	//--------------------------------
	//	Event:auto
	//--------------------------------
	setEvent(AUTO);

	//--------------------------------
	//	Event:touch
	//--------------------------------
	if(g_Sensor.touch == 1 && g_Controller.touch_status == 0)
	{
		setEvent(TOUCH);
		g_Controller.touch_status = 1;
	}
	else if(g_Sensor.touch == 0 && g_Controller.touch_status == 1)
	{
		g_Controller.touch_status = 0;
	}

	//==========================================
	//	black & white
	//==========================================
	if(g_Sensor.light > (g_Actuator.black - 50) && g_Controller.light_status != 2)
	{
		//--------------------------------
		//	Event:black
		//--------------------------------
		setEvent(BLACK);
		g_Controller.light_status = 2;
	}
	else if(g_Sensor.light < (g_Actuator.white + 50) && g_Controller.light_status != 1)
	{
		//--------------------------------
		//	Event:white
		//--------------------------------
		setEvent(WHITE);
		g_Controller.light_status = 1;
	}
	else
	{
		g_Controller.light_status = 0;
	}

	//--------------------------------
	//	Event:gray marker
	//--------------------------------
	if( g_Actuator.gray_offset - 10 < g_Sensor.light_ave && g_Sensor.light_ave < g_Actuator.gray_offset + 10  )
	{
		setEvent(GRAY_MARKER);
		g_Actuator.TraceMode = 2;
	}
	else
	{
		g_Actuator.TraceMode = 1;
	}

	//--------------------------------
	//	Event:step
	//--------------------------------
	if( abs((int)(g_Sensor.gyro - g_Actuator.gyro_offset)) > g_Actuator.step_offset	)
	{
		setEvent(STEP);
	}

	//--------------------------------
	//	Event:sonar
	//--------------------------------
   	if(g_Sensor.distance < g_Controller.target_distance )
   	{
   		setEvent(SONAR);
	}

	//--------------------------------
	//	Event:timer
	//--------------------------------
   	if( g_Controller.timer_flag == 1 )
   	{
   		if( (systick_get_ms() - g_Controller.start_time) > g_Controller.target_time )
   		{
   			setEvent(TIMER);
   			g_Controller.target_time = 0;
   			g_Controller.start_time = 0;
   			g_Controller.timer_flag = 0;
   		}
   	}

	//--------------------------------
	//	Event:motor count
	//--------------------------------
   	if( g_Controller.motor_counter_flag == 1 )
   	{
   		int motor_count = (g_Sensor.count_left + g_Sensor.count_right) / 2;
   		if( abs(motor_count - g_Controller.start_motor_count) > abs(g_Controller.target_motor_count) )
   		{
   			setEvent(MOTOR_COUNT);
   			g_Controller.target_motor_count = 0;
   			g_Controller.start_motor_count = 0;
   			g_Controller.motor_counter_flag = 0;
   		}
   	}

	//--------------------------------
	//	Event:bluetooth start
	//--------------------------------
   	g_Sensor.BTstart = BluetoothStart();

	if( g_Sensor.BTstart == 1 && g_Controller.BTstart_status == 0 )
	{
		setEvent(BT_START);
		g_Controller.BTstart_status = 1;
	}
	else if( g_Sensor.BTstart == 0 && g_Controller.BTstart_status == 1 )
	{
		g_Controller.BTstart_status = 0;
	}


	//--------------------------------
	//	Event:pivot turn
	//--------------------------------
	if( g_Controller.pivot_turn_flag == 1  )
	{
		if( abs(g_Sensor.count_right - g_Controller.start_pivot_turn_encoder_R) >= g_Controller.target_pivot_turn_angle_R )
		{
			setEvent(PIVOT_TURN_END);
			g_Controller.pivot_turn_flag = 0;
		}
	}


	//==========================================
	//	drift turn
	//==========================================
	if(g_Controller.object_judge == 1)
	{
		//sendevent turn left or right

		if(g_Sensor.object_is_right == 1 && g_Sensor.object_is_left == 0)
		{
			//--------------------------------
			//	Event:bottle is right
			//--------------------------------
			setEvent(BOTTLE_RIGHT);
		}
		else if(g_Sensor.object_is_right == 0 && g_Sensor.object_is_left == 1)
		{
			//--------------------------------
			//	Event:bottle is left
			//--------------------------------
			setEvent(BOTTLE_LEFT);
		}

		g_Controller.object_judge = 0;
	}

	//
	setNextState();

}


/*
===============================================================================================
	name: setController
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
===============================================================================================
*/
void setController(void)
{
	State_t state = getCurrentState();

	switch( state.action_no )
	{
		case DO_NOTHING://do nothing
			g_Actuator.forward = 0;
			g_Actuator.turn = 0;

			g_Actuator.TraceMode = 1;
			g_Actuator.StandMode = 1;

			break;

		case BALANCE_STOP://stop
			g_Actuator.forward = 0;
			g_Actuator.turn = 0;

			g_Actuator.TraceMode = 1;
			g_Actuator.StandMode = 1;

			break;

		// linetrace
		//@param foward:=value0
		//@param gyro_offset:=value1
		case BALANCE_LINETRACE:
			g_Actuator.forward = state.value0;
			g_Actuator.gyro_offset = g_Actuator.gyro_offset_base + state.value1;

			g_Actuator.TraceMode = 1;
			g_Actuator.StandMode = 1;

			break;

		//change the gray threshold
		//@param new threshold:=value0
		case CHANGE_GRAY:
			g_Actuator.target_gray = g_Actuator.target_gray_base + state.value0;

			break;

		//run with no linetrace without balance
		//@param target_tail:=value0
		//@param tail_run_speed :=value1
		//@param turn :=value2
		case TAIL_RUN_FREEDOM:
			g_Actuator.target_tail = state.value0;
			g_Actuator.forward = state.value1;
			g_Actuator.turn = state.value2;
			g_Actuator.TP_gain = (F32)state.value3 / 100;

			g_Actuator.TraceMode = 0;
			g_Actuator.StandMode = 2;
			break;

		//set timer
		//@param limit_timer:=value0 i.e. 20 = 2.0sec
		case TIMER_SET:
			if( g_Controller.timer_flag == 0 )
			{
				g_Controller.start_time = systick_get_ms();
				g_Controller.target_time = state.value0 * 100;
				g_Controller.timer_flag = 1;
			}

			break;
		//set motor count
		case MOTOR_SET:
			if( g_Controller.motor_counter_flag == 0 )
			{
				g_Controller.start_motor_count = (g_Sensor.count_left + g_Sensor.count_right) / 2;
				g_Controller.target_motor_count = state.value0;
				g_Controller.motor_counter_flag = 1;
			}

			break;

		//set PID
		//@param P_gain:=value0/100
		//@param I_gain:=value1/100
		//@param D_gain:=value2/100
		case PID_SET:

			g_Actuator.P_gain = (F32)state.value0 / 100;
			g_Actuator.I_gain = (F32)state.value1 / 100;
			g_Actuator.D_gain = (F32)state.value2 / 100;

			break;

		//set gyro offset for steps
		//@param step_offset := value0
		case STEP_OFFSET_SET:
			g_Actuator.step_offset = state.value0;
			//g_Sensor.GYRO_BUFFER_LENGTH = state.value1;
			break;

		//up the tail
		case RAISE_TAIL:
			g_Actuator.target_tail = 0;
			//nxt_motor_set_speed(TAIL_MOTOR,-15,1);
			break;

		// tail run
		//@param angle
		//@param speed
 		case TAIL_LINETRACE:
			g_Actuator.TraceMode = 1;
			g_Actuator.StandMode = 2;
			g_Actuator.target_tail = state.value0;
			//g_Actuator.tail_run_speed = state.value1;
			g_Actuator.forward = state.value1;
			g_Actuator.TP_gain = (F32)state.value2 / 100;
			break;

		//circling
		//@param angle to turn
		case PIVOT_TURN:

			if( g_Controller.pivot_turn_flag == 0 )
			{
				g_Actuator.forward = 0;
				g_Actuator.TraceMode = 0;

				if( state.value0 >= 0 )
				{
					g_Actuator.turn = abs( state.value1 );
				}
				else
				{
					g_Actuator.turn = -1 * abs( state.value1 );
				}

				g_Controller.start_pivot_turn_encoder_R = g_Sensor.count_right;
				g_Controller.target_pivot_turn_angle_R = calcAngle2Encoder(state.value0);
				g_Controller.pivot_turn_flag = 1;
			}

			break;

		//selecting logger
		//@param log_type
		case SELECT_LOGTYPE:
			g_LogType = state.value0;
			break;

		//set the gray_market offset
		//@param gray_offset
		case GRAY_MARKER_OFFSET:
			g_Actuator.gray_offset = state.value0;
			//g_Sensor.LIGHT_BUFFER_LENGTH = state.value1;

			break;

		//free balance
		case BALANCE_RUN_FREEDOM:
			g_Actuator.TraceMode = 0;
			g_Actuator.StandMode = 1;

			g_Actuator.forward = state.value0;
			g_Actuator.turn = state.value1;
			g_Actuator.gyro_offset = g_Actuator.gyro_offset_base + state.value2;

			//nxt_motor_set_speed(TAIL_MOTOR,0,1);

			break;

		//set_sonar_sensor
		case SONAR_SET:
			g_Controller.target_distance = state.value0;
			break;

		case SERACH_BOTTLE_RIGHT:
			g_Actuator.forward = 0;

			if( g_Controller.object_right_flag == 0 )
			{
				g_Controller.object_right_length = state.value0;
				g_Controller.object_right_flag = 1;
			}
			break;

		case SEARCH_BOTTLE_LEFT:
			g_Actuator.forward = 0;

			if( g_Controller.object_left_flag == 0 )
			{
				g_Controller.object_left_length = state.value0;
				g_Controller.object_left_flag = 1;
			}

			break;

		case SEARCH_BOTTLE_END:
			g_Controller.object_right_flag = 0;
			g_Controller.object_right_length = 0;
			g_Sensor.object_is_right = 0;
			g_Controller.object_left_flag = 0;
			g_Controller.object_left_length = 0;
			g_Sensor.object_is_left = 0;
			break;

		case SEARCH_BOTTLE_JUDGE:
			g_Controller.object_judge = 1;
			break;
		default:
			break;
	}
}

/*
===============================================================================================
	name: calcAngle2Encoder
	Description: ??
===============================================================================================
*/
int calcAngle2Encoder(S16 ang)
{
	F32 ret = 0;
	ret = (F32)ang * 16.3 / 8.1;

	if(ret < 0)
	{
		ret = -ret;
	}

	return (int)ret;
}


