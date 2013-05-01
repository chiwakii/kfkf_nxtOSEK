typedef struct tag_Logger{


#define LOG_TURN 1
#define LOG_PWM 2
#define LOG_TARGET_ANGLE 3
#define LOG_LIGHT_MIN 4
#define LOG_MOTOR_COUNT 5
#define LOG_SONAR 6
#define LOG_STATE 7
#define LOG_DT 8
#define LOG_BALANCE_TAIL 9
#define LOG_LOOP 10
	byte type;

} Logger_t;