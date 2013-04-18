/*
 *  bluetooth_interface.h
 */

/* NXT Bluetooth configuration */
#define BT_PASS_KEY		"0753"	/* bluetoothパスキー */
#define BT_RCV_BUF_SIZE 32		/* bluetooth受信パケットの大きさ */

S16 bt_receive_buf[BT_RCV_BUF_SIZE];	/* bluetooth受信パケットの宣言 */


/*BT_PARAMETER*/

int FORWARD_TIME;		/* 前進速度 */
int AUTO_TAIL_POWER;		/* 尻尾モータ(50くらいが良い) */
int FORWARD_POWER;		/* 前進速度(-100～100) */
int BACK_POWER;			/* 後退速度(-100～100) */
int GYRO_OFFSET;		/* ジャイロオフセット(4くらいが良い) */
int P_GAIN_TMP;			/* Pゲイン(default:1) */
int I_GAIN_TMP;			/* Iゲイン(default:1) */
int D_GAIN_TMP;			/* Dゲイン(default:1) */
int ANGLE_TAIL;//=50;

void receive_BT(S16 *matrix,S16 *states,StateMachine_t statemachine);
