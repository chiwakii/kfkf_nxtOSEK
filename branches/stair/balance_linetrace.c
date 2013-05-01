#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"
#include "port_interface.h"
#include "bluetooth_interface.h"

//#define GYRO_OFFSET	610	/* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
//#define WHITE	530		/* ���F�̌��Z���T�l */
//#define BLACK	720		/* ���F�̌��Z���T�l */
#define DELTA_T		0.004
#define KP		0.0	/* ���Q�C�� */
#define KI		0.0	/* �ϕ��Q�C�� */
#define KD		0.0	/* �����Q�C�� */

//*****************************************************************************
// �֐��� : ecrobot_device_initialize
// ���� : �Ȃ�
// �߂�l	: �Ȃ�
// �T�v : ECROBOT�f�o�C�X�����������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_initialize(){				// OSEK�N�����̏����i���[�^��~�j
	nxt_motor_set_speed(LEFT_MOTOR,0,0);			// nxt_motor_speed(n,int speed_persent,brake)
	nxt_motor_set_speed(RIGHT_MOTOR,0,0);			// n : NXT_PORT_A , NXT_PORT_B(�E) , NXT_PORT_C(��)
//	nxt_motor_set_speed(TAIL_MOTOR,50,0);			// speed_persent : -100 �` 100
//	systick_wait_ms(10);					// brake : 0(�t���[�g���[�h) , 1(�u���[�L���[�h)
//	nxt_motor_set_speed(TAIL_MOTOR,0,1);

	nxt_motor_set_count(TAIL_MOTOR,0);
	ecrobot_set_light_sensor_active(LIGHT_SENSOR);		/* ���C�g�Z���T�_�� */
	ecrobot_init_sonar_sensor(SONAR_SENSOR);		/*sonar*/
	ecrobot_init_bt_slave(BT_PASS_KEY);			/* �X���[�u�Ƃ��ď����� */

}

//*****************************************************************************
// �֐��� : ecrobot_device_terminate
// ���� : �Ȃ�
// �߂�l	: �Ȃ�
// �T�v : ECROBOT�f�o�C�X�I�������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_terminate(){				// OSEK�I�����̏����i���[�^��~�j
	display_string("DT");
	display_update();	
	nxt_motor_set_speed(LEFT_MOTOR,0,0);
	nxt_motor_set_speed(RIGHT_MOTOR,0,0);
	ecrobot_term_bt_connection();				/* bluetooth�ʐM�I�� */
	ecrobot_set_light_sensor_inactive(LIGHT_SENSOR);	/* ���C�g�Z���T���� */
	ecrobot_term_sonar_sensor(SONAR_SENSOR);		/*sonar*/
}

//*****************************************************************************
// �֐��� : user_1ms_isr_type2
// ���� : �Ȃ�
// �߂�l	: �Ȃ�
// �T�v : 1msec�������荞�݃t�b�N�֐�(OSEK ISR type2�J�e�S��)
//*****************************************************************************
void user_1ms_isr_type2(void){}


//*****************************************************************************
// �֐����Fcalibration
// �����Fblack,white,gray
// �߂�l�F�ϑ��l
// �T�v�F���Z���T�ɂ��F�ʒl�ݒ�
//*****************************************************************************
void calibration(int *black,int *white,int *gray){			/* 臒l�ݒ�p���[�U�֐� */

	while( ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0 ){		/* �^�b�`�Z���T��������Ă��Ȃ������� */

		*white = ecrobot_get_light_sensor(LIGHT_SENSOR);	/* ���̖��邳�ǂݎ�� */

		display_clear(0);					/* ��ʕ\�� */
		display_goto_xy(0, 1);
		display_string("WHITE=");
		display_int(*white, 4);
		display_update();

		systick_wait_ms(10);
	}

	systick_wait_ms(1000);


	while(ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0){		/* �^�b�`�Z���T��������Ă��Ȃ������� */

		*black = ecrobot_get_light_sensor(LIGHT_SENSOR);	/* ���̖��邳�ǂݎ�� */

		display_clear(0);					/* ��ʕ\�� */
		display_goto_xy(0, 1);
		display_string("BLACK=");
		display_int(*black, 4);
		display_update();

		systick_wait_ms(10);

	}

	systick_wait_ms(1000);

	*gray=( *black + *white ) / 2;					/* ���Ԓl�Z�o */

	display_clear(0);
	display_goto_xy(0, 1);
	display_string("gray=");
	display_int(*gray, 4);
	display_update();

	systick_wait_ms(1000);
//

}



//*****************************************************************************
// �^�X�N�� : TaskMain
// �T�v : ���C���^�X�N
//*****************************************************************************
TASK(TaskMain)
{
//	signed char forward;      	/* �O��i���� */
	signed char turn;         	/* ���񖽗� */
	signed char pwm_L, pwm_R;	 /* ���E���[�^PWM�o�� */

	int sensorvalue;		/* ���C�g�Z���T�l���i�[����ϐ����` */
//	int speed=50;			/* ���x�ϐ� */
//	float p_gain=1.0;		/* P�Q�C���ϐ� */
//	float i_gain = 1.0;		/* I�Q�C���ϐ� */
//	float d_gain=1.0;		/* D�Q�C���ϐ� */
	int black, white, gray;		/* �F�ϐ� */
	int light, light_tmp;		/* 1�O�̖��邳���i�[����light_tmp���` */
	int before=0, standard=0;	/* before�F1�O�̕΍��Astandard�F���݂̕΍� */
	float integral=0;


	balance_init();					/* �|���U�q���䏉���� */
	nxt_motor_set_count(LEFT_MOTOR, 0);		/* �����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(RIGHT_MOTOR, 0);		/* �E���[�^�G���R�[�_���Z�b�g */

	/* bluetooth�Z�N�V���� */
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
		//	ANGLE_TAIL = (int)bt_receive_buf[1];
			
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

	float p_gain = P_GAIN_TMP / 10;
	float i_gain = I_GAIN_TMP / 10;
	float d_gain = D_GAIN_TMP / 10;

	while(1)
	{

		sensorvalue = ecrobot_get_light_sensor(LIGHT_SENSOR);	/* ���C�g�Z���T�l��ϐ��ɑ�� */

		calibration( &black, &white, &gray );			/* 臒l�ݒ�p�֐��Ăяo�� */
		systick_wait_ms(2000);					/* 1�b�҂� */
//		nxt_motor_set_speed(TAIL_MOTOR,15,1);

		light_tmp = ecrobot_get_light_sensor(LIGHT_SENSOR);

  		display_goto_xy(0, 1);
  		display_string("light value=");
   		display_int(sensorvalue, 1);						/* sensorvalue�ϐ��̒��g��\�� */
		display_update();


//		while(1){								/* �^�b�`�Z���T�������ăX�^�[�g */
//			if( ecrobot_get_touch_sensor(TOUCH_SENSOR) == 1 ){
//				break;
//			}
//		}

		while( ecrobot_get_touch_sensor(TOUCH_SENSOR) == 0 )
		{
//			forward = 50;							/* �O�i���� */

			light = ecrobot_get_light_sensor(LIGHT_SENSOR);
			before = standard;
			standard = light - gray;					/* �΍����擾 */
			integral += (standard - before)/2.0 * DELTA_T;


			turn = p_gain * standard *100 / (black-white) + i_gain * integral / (black-white) * 100 + d_gain * (light-light_tmp) / (black-white) * 100;	/* ����l�v�Z */

			light_tmp = light;						/* 1�O�̖��邳���i�[ */


			/* �|���U�q����(forward = 0, turn = 0�ŐÎ~�o�����X) */
			balance_control(
				(float)FORWARD_POWER,					/* �O��i����(+:�O�i, -:��i) */
				(float)turn,						/* ���񖽗�(+:�E����, -:������) */
				(float)ecrobot_get_gyro_sensor(GYRO_SENSOR),		/* �W���C���Z���T�l */
				(float)GYRO_OFFSET,					/* �W���C���Z���T�I�t�Z�b�g�l */
				(float)nxt_motor_get_count(LEFT_MOTOR),			/* �����[�^��]�p�x[deg] */
				(float)nxt_motor_get_count(RIGHT_MOTOR),		/* �E���[�^��]�p�x[deg] */
				(float)ecrobot_get_battery_voltage(),			/* �o�b�e���d��[mV] */
				&pwm_L,							/* �����[�^PWM�o�͒l */
				&pwm_R);						/* �E���[�^PWM�o�͒l */
			nxt_motor_set_speed(LEFT_MOTOR, pwm_L, 1);			/* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			nxt_motor_set_speed(RIGHT_MOTOR, pwm_R, 1);			/* �E���[�^PWM�o�̓Z�b�g(-100�`100) */

//			nxt_motor_set_speed( LEFT_MOTOR, forward-turn, 1 );
//			nxt_motor_set_speed( RIGHT_MOTOR, forward+turn, 1 );

			
//			ecrobot_bt_data_logger(100,-100);


			systick_wait_ms(4);
		}

	}

}