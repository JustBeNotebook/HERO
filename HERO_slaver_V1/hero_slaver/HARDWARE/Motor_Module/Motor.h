#ifndef RM__MOTOR__H
#define RM__MOTOR__H

#include "sys.h"
#include "pid.h"

//LIMIT函数，使用时注意两个变量类型要相同
#define MIN(x, y) ((x)>(y)?(y):(x))
#define MAX(x, y) ((x)>(y)?(x):(y))

typedef short int Motor_speed;
typedef short int Motor_angle;
typedef short int Motor_current;

typedef enum MOTOR_STATE
{
	M_DISABLE = 0,
	M_ENABLE,
	M_DISCONNECT,
	M_CONNECTING
	
}MOTOR_STATE;


typedef struct Motor_Module
{
	Motor_speed tg_speed;
	Motor_speed speed_real;
	Motor_angle angle;
	Motor_angle tg_angle;
	Motor_current current;
	u8 temperature;
	int angle_turns;
	int tg_angle_turns;
	short int output;
	short int output_limit;
	u8 offline_cnt;
	MOTOR_STATE state;									//电机当前状态的设置
	
	Pid_Module speed_PID;
	Pid_Module angle_PID;
}Motor_Module;

//电机初始化
void Motor_Module_Init(Motor_Module *motor, short int output_limit);


//电机属性状态解算
u8 Motor_analyze(Motor_Module *motor, u8 *buf);
void Motor_PID_Update(Motor_Module *Motor);

//////////////////////////////////////////////////////////////////////////////////
//电机接口类函数

//电机目标速度设置函数
void Motor_set_speed(Motor_Module *motor, Motor_speed speed);

//电机目标角度设置函数
void Motor_set_angle_turns(Motor_Module *motor, int turn, short int angle);
void Motor_set_angle(Motor_Module *motor, int angle);
void Motor_angle_add(Motor_Module *motor, int angle);

//电机状态更新函数
void Motor_State_Update(Motor_Module *M_temp);
void Motor_State_Frash(Motor_Module *M_temp);
void Motor_PID_Clear(Motor_Module *Motor);

#endif
