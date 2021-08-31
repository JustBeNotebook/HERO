#ifndef RP__PID__H
#define RP__PID__H

#include "sys.h"

typedef enum
{
	PID_DISABLE,
	PID_ENABLE
}PID_STATE;

typedef struct Pid_Module
{
	float err;	//err值相关计算值
	float err_dz;
	float err_limit;
	
	float d_err;
	float i_err;
	float i_err_limit;
	
	float kp;	//PID参数
	float ki;
	float kd;

	float output;				//输出相关
	float output_limit;//输出限幅
	float p_output;
	float i_output;
	float d_output;
	
	PID_STATE startfalg;
	
}Pid_Module;

//输入参数，
//PID 模块结构体
//err 该轮PID调控中得到的err值
float Pos_PID_CAL_OUT(Pid_Module *PID, float err);

//PID改参数
void PID_Set(Pid_Module *PID, float kp, float ki, float kd, float i_err_limit, float err_dz, float output_limit);

//PID初始化
void PID_Init(Pid_Module *PID);

//PID缓存清空
void PID_Clear(Pid_Module *PID);

#endif

