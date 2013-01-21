#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

typedef struct tag_Controller_{
	float speed;

	byte pid_on;
	byte balance_on;
	byte tail_on;
	byte wg_pid_on;
	
	float forward_power;
	float turn;

	float gyro_offset;
	S16 gray_offset;
	int color_threshold;
	float P_gain;
	float I_gain;
	float D_gain;
	float integral;
	
	U16 step_offset;
	U16 base_gyro_offset;
	U32 tail_ang;
	S16 tail_run_speed;
	S8 tail_speed_offset;
	float right_motor_rate;
	float left_motor_rate;

}Controller_t;
