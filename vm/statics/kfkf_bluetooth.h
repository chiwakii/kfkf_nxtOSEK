/*
 *  bluetooth_interface.h
 */

/* NXT Bluetooth configuration */
#define BT_PASS_KEY		"0753"	/* bluetooth�p�X�L�[ */
#define BT_RCV_BUF_SIZE 32		/* bluetooth��M�p�P�b�g�̑傫�� */

S16 bt_receive_buf[BT_RCV_BUF_SIZE];	/* bluetooth��M�p�P�b�g�̐錾 */


/*BT_PARAMETER*/

int FORWARD_TIME;		/* �O�i���x */
int AUTO_TAIL_POWER;		/* �K�����[�^(50���炢���ǂ�) */
int FORWARD_POWER;		/* �O�i���x(-100�`100) */
int BACK_POWER;			/* ��ޑ��x(-100�`100) */
int GYRO_OFFSET;		/* �W���C���I�t�Z�b�g(4���炢���ǂ�) */
int P_GAIN_TMP;			/* P�Q�C��(default:1) */
int I_GAIN_TMP;			/* I�Q�C��(default:1) */
int D_GAIN_TMP;			/* D�Q�C��(default:1) */
int ANGLE_TAIL;//=50;

void receive_BT(S16 *matrix,S16 *states,StateMachine_t statemachine);
