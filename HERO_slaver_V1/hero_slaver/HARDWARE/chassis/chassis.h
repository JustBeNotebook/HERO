#ifndef RP__CLASSIS__H
#define RP__CLASSIS__H

#include "link.h"
#include "motor.h"



typedef struct Chassis_Module
{
	Motor_Module Motor_Num[4];
	
}Chassis_Module;


typedef struct unknow_Module
{
	Motor_Module ID5_F[16];
	
}unknow_Module;


extern u8 start_pid;
extern u8 start_pos_pid;
extern Chassis_Module chassis;//底盘结构体
extern Motor_speed speed2[4];//底盘电机速度缓冲区
extern unknow_Module UNKONW;
extern unknow_Module UNKONW2;

//用CAN发送四个轮子的速度
u8 Chassis_Set_Output(Motor_speed s1, Motor_speed s2, Motor_speed s3, Motor_speed s4);

//从CAN邮箱中读取四个轮子的状态信息
u8 speed_analyze(u16 id, u8 *buf);
u8 Motor_analyze2(u16 id, u8 *buf);


//更新底盘的输出
void Chassis_Speed_Update(void);
//设置底盘目标速度
void Chassis_set_speed(Motor_speed s1, Motor_speed s2, Motor_speed s3, Motor_speed s4);
//设置底盘目标角度
void Chassis_set_angle(u8 motor_code, int angle);
//底盘电机配置初始化
void Chassis_Config(void);
//底盘电机状态更新函数
void Chassis_State_Update(void);//1ms任务








#endif
