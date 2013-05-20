enum LogType
{
	LOG_NO = 0,
	LOG_TURN = 1,
	LOG_PWM = 2,
	LOG_TARGET_ANGLE = 3,
	LOG_LIGHT_MIN = 4,
	LOG_MOTOR_COUNT = 5,
	LOG_SONAR = 6,
	LOG_STATE = 7,
	LOG_DT = 8,
	LOG_BALANCE_TAIL = 9,
	LOG_LOOP = 10
};

typedef struct tag_Logger{

	enum LogType type;

} Logger_t;


