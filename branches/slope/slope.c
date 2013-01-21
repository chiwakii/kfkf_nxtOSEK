#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"
#include "port_interface.h"
#include "bluetooth_interface.h"


//#define GYRO_OFFSET	610	/* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
//#define WHITE	530		/* 白色の光センサ値 */
//#define BLACK	720		/* 黒色の光センサ値 */
#define DELTA_T		0.004
#define KP		0.0	/* 比例ゲイン */
#define KI		0.0	/* 積分ゲイン */
#define KD		0.0	/* 微分ゲイン */	

DeclareCounter(SysTimerCnt);
DeclareTask(Task1);				/* Task1を宣言 */
DeclareTask(Task2);				/* Task2を宣言 */
DeclareTask(Task_bg);				/* Task_bgを宣言 */

/////////////////////////////////
//variables
////////////////////////////////

static int init=0;
	signed char turn;        	 			/* 旋回命令 */
	signed char pwm_L, pwm_R;				/* 左右モータPWM出力 */
	int sensorvalue;					/* ライトセンサ値を格納する変数を定義 */
	int black, white, gray;					/* 色変数 */
	int light, light_tmp;					/* 1つ前の明るさを格納するlight_tmpを定義 */
	int before=0, standard=0;				/* before：1つ前の偏差、standard：現在の偏差 */
	float integral=0;
	int start_time,current_time;
	float off;
	float p_gain;
	float i_gain;
	float d_gain;



//*****************************************************************************
// 関数名：ecrobot_device_initialize
// 引数：なし
// 戻り値：なし
// 概要：ECROBOTデバイス初期化処理フック関数
//*****************************************************************************

void ecrobot_device_initialize(){
	nxt_motor_set_speed(LEFT_MOTOR,0,0);			// nxt_motor_speed(n,int speed_persent,brake)
	nxt_motor_set_speed(RIGHT_MOTOR,0,0);			// n : NXT_PORT_A , NXT_PORT_B(右) , NXT_PORT_C(左)

	nxt_motor_set_count(TAIL_MOTOR,0);
	ecrobot_set_light_sensor_active(LIGHT_SENSOR);		/* ライトセンサ点灯 */
	ecrobot_init_sonar_sensor(SONAR_SENSOR);		/*sonar*/
	ecrobot_init_bt_slave(BT_PASS_KEY);			/* スレーブとして初期化 */
}

//*****************************************************************************
// 関数名：ecrobot_device_terminate
// 引数：なし
// 戻り値：なし
// 概要：ECROBOTデバイス終了処理フック関数
//*****************************************************************************

void ecrobot_device_terminate(){
	nxt_motor_set_speed(LEFT_MOTOR,0,0);
	nxt_motor_set_speed(RIGHT_MOTOR,0,0);
	ecrobot_term_bt_connection();		/* bluetooth通信終了 */
	ecrobot_set_light_sensor_inactive(LIGHT_SENSOR);	/* ライトセンサ消灯 */
	ecrobot_term_sonar_sensor(SONAR_SENSOR);		/*sonar*/
}


//*****************************************************************************
// 関数名：user_1ms_isr_type2
// 引数：なし
// 戻り値：なし
// 概要：1msec周期割り込みフック関数(OSEK ISR type2カテゴリ)
//*****************************************************************************

void user_1ms_isr_type2(void){
	SignalCounter(SysTimerCnt);    /* カウンタをIncrementする */
}

//*****************************************************************************
// 関数名：calibration
// 引数：black,white,gray
// 戻り値：観測値
// 概要：光センサによる色彩値設定
//*****************************************************************************
void calibration(int *black,int *white,int *gray){			/* 閾値設定用ユーザ関数 */

	while( ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0 ){		/* タッチセンサが押されていなかったら */

		*white = ecrobot_get_light_sensor(LIGHT_SENSOR);	/* 白の明るさ読み取り */

		display_clear(0);					/* 画面表示 */
		display_goto_xy(0, 1);
		display_string("WHITE=");
		display_int(*white, 4);
		display_update();

		systick_wait_ms(10);

	}

	systick_wait_ms(1000);


	while(ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0){		/* タッチセンサが押されていなかったら */

		*black = ecrobot_get_light_sensor(LIGHT_SENSOR);	/* 黒の明るさ読み取り */

		display_clear(0);					/* 画面表示 */
		display_goto_xy(0, 1);
		display_string("BLACK=");
		display_int(*black, 4);
		display_update();

		systick_wait_ms(10);

	}

	systick_wait_ms(1000);

	*gray=( *black + *white ) / 2;					/* 中間値算出 */

	display_clear(0);
	display_goto_xy(0, 1);
	display_string("gray=");
	display_int(*gray, 4);
	display_update();

	systick_wait_ms(1000);

}


TASK(Task1)
{






if(init ==0){
	balance_init();							//バランサAPIの初期化
	nxt_motor_set_count(NXT_PORT_C,0);
	nxt_motor_set_count(NXT_PORT_B,0);
	init =1;
	
	/* キャリブレーション */
	calibration( &black, &white, &gray );			/* 閾値設定用関数呼び出し */

	systick_wait_ms(2000);					/* 2秒待って */
	nxt_motor_set_speed(TAIL_MOTOR,15,1);			/* 尻尾を下ろす */
	
	
	
		/* bluetoothセクション */
	while(1){

		ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);

		int ope = bt_receive_buf[0];
		if(ope==255){
			break;
		}else if(ope==101){
			AUTO_TAIL_POWER = bt_receive_buf[1];
			
		}else if(ope == 102){
			FORWARD_POWER = bt_receive_buf[1];
			
		}else if(ope == 103){
			BACK_POWER = (-1)*bt_receive_buf[1];
			
		}else if(ope == 150){
			ANGLE_TAIL = (int)bt_receive_buf[1];
			
		}else if(ope == 104){
			FORWARD_TIME = 1000*bt_receive_buf[1];
			
		}else if(ope == 105){
			 GYRO_OFFSET = 610+bt_receive_buf[1];
			
		}else if(ope == 106){
			 P_GAIN_TMP = bt_receive_buf[1];
			
		}else if(ope == 107){
			 I_GAIN_TMP = bt_receive_buf[1];
			
		}else if(ope == 108){
			 D_GAIN_TMP = bt_receive_buf[1];
			
		}

	}

	nxt_motor_set_speed(TAIL_MOTOR,-15,0);						/* 尻尾を上げてスタート */
	
	p_gain = P_GAIN_TMP / 100;
	i_gain = I_GAIN_TMP / 50;
	d_gain = D_GAIN_TMP / 50;
}

	
	sensorvalue = ecrobot_get_light_sensor(LIGHT_SENSOR);			/* ライトセンサ値を変数に代入 */

		light_tmp = ecrobot_get_light_sensor(LIGHT_SENSOR);

  		display_goto_xy(0, 1);
  		display_string("light value=");
   		display_int(sensorvalue, 1);						/* sensorvalue変数の中身を表示 */
		display_update();
		
		
		light = ecrobot_get_light_sensor(LIGHT_SENSOR);
			before = standard;
			standard = light - gray;					/* 偏差を取得 */
			integral += (standard - before)/2.0 * DELTA_T;

			turn = p_gain * standard *100 / (black-white) + i_gain * integral / (black-white) * 100 + d_gain * (light-light_tmp) / (black-white) * 100;	/* 旋回値計算 */

			light_tmp = light;						/* 1つ前の明るさを格納 */




		balance_control(					//バランサAPIの呼び出し

			(float)FORWARD_POWER,					//前進／後退命令(-100(右旋回最大)?100(左旋回最大))
			(float)turn,					//旋回命令(-100?100)
			(float)ecrobot_get_gyro_sensor(NXT_PORT_S1),	//ジャイロセンサ値
			(float)610,				//ジャイロセンサオフセット値
			(float)nxt_motor_get_count(NXT_PORT_C),		//左モータエンコーダ値
			(float)nxt_motor_get_count(NXT_PORT_B),		//右モータエンコーダ値
			(float)ecrobot_get_battery_voltage(),		//バッテリ電圧値[mV]

			&pwm_L,						//左モータPWM出力値（戻り値）
			&pwm_R						//右モータPWM出力値（戻り値）

		);

		nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1);		//モータに指令を出す
		nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1);

		TerminateTask();


}


TASK(Task2)
{

	int sonar;

	sonar = ecrobot_get_sonar_sensor(NXT_PORT_S2);

	ecrobot_bt_data_logger(100, 100);

	
	TerminateTask();					/* 処理終了 */

}

TASK(Task_bg)
{


	TerminateTask();					/* 処理終了 */
}




