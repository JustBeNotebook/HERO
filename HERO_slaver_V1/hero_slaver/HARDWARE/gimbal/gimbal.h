#ifndef RP__GIMBAL__H
#define RP__GIMBAL__H

#include "link.h"


typedef struct Gimbal_Module
{
	Motor_Module Yaw_Motor;
	Motor_Module Pitch_Motor;
	short gyrox;
  short	gyroy;
	short	gyroz;//角速度值 x是横滚角速度，y是pitch角速度，z是偏航角速度
	float pitch;
	float roll;
	float yaw;		//角度值   pitch 是俯仰角度 前倾为负， roll是横滚角度 右低为负，yaw是偏航角度 顺时针为负
	
	int YTurns;
	u16 YAngle;
	u16 tg_YAngle;//目标角度
	int tg_YTurns;
	
	float tg_PAngle;
	
	
	Pid_Module Mpu_Yaw_SPID;
	Pid_Module Mpu_Yaw_APID;
	
	Pid_Module Mpu_Pitch_SPID;
	Pid_Module Mpu_Pitch_APID;
	
}Gimbal_Module;

extern Gimbal_Module Gimbal;
extern Gimbal_Module infantry_Gimbal;

//云台电机更新函数
void Gimbal_State_Update(void);
void Gimbal_Config(void);
void Gimbal_PID_Cal(u8 code);
void Gimbal_Set_Output(short int out_1, short int out_2);

void Gimbal_MpuPID_Cal(u8 Motor_code);
void Gimbal_Mpu_Analyze(void);//MPU6050解算结果接收
void Mpu_Yaw_tgAngle_add(int angle);
#endif

