#ifndef RM__LAUNCHER__H
#define RM__LAUNCHER__H


#include "link.h"

typedef enum 
{
	OFF,
	ON,
	LOCK
}Binary_State;

typedef struct Friction_Module
{
	u32 PWM_Speed;
	u32 PWM_Max;
	u32 Target_Speed;
	Binary_State state;
}Friction_Module;

typedef struct Launcher_Module
{
	Motor_Module Magazine;	//弹仓电机
	Motor_Module Dial;			//拨盘电机
	Motor_Module Safety;		//限位/保险
	Friction_Module Friction;
	
}Launcher_Module;

extern Launcher_Module Hero_Launcher;


//发射机构初始化
void Launcher_Init(void);

//发射机构的任务
void Launcher_Task(Launcher_Module *Launcher, u8 mode_flag);

//总电机失联判断
void Launcher_state_update(void);

//发射机构填充发送数据
void Launcher_set_output(u16 Magazine, u16 Safety, u16 Dial);


//发射机构保险工作函数
void Launcher_Magazine(void);

//摩擦轮工作函数
void Launcher_Friction(void);

//摩擦轮斜坡函数
void Friction_output(void);

//发射一颗弹丸
void Launcher_Dial(u8 flag);
	
#endif
