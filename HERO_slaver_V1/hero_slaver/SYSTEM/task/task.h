#ifndef __TASK_H
#define __TASK_H

#include "link.h"
#include "supercap.h"

#define STACK_SIZE 9

//任务模块以LOOP的周期为最小单位进行任务周期拓展
//整个任务模块的周期随LOOP的定时时间长短进行相应的倍增或者

typedef enum Task_Status //任务结构体枚举列表
{
	SUSPENDED,		//挂起状态
	READY,				//准备状态
	RUNNING,			//运行态
	BLOCKED				//阻塞态，将来如果有需要外部条件下才使用
}Task_Status;

typedef struct TASK_Module{
	Task_Status flag;			//任务标志位
	void (*function)(void);	//任务执行函数
	u32 cycle;						//任务周期，最小周期单位为100us
	u32 cnt;							//任务计数，看看经过多少个最小单位
}TASK_Module;

typedef struct TASK_Stack
{
	TASK_Module Stack[STACK_SIZE+1];
	u8 stack_pot;
	u8 stack_base;
}TASK_Stack;


//TASK_Init(void)
//执行任务总初始化
void TASK_Init(void);

//xms 任务函数
static void Task_1MS(void);
static void Task_2MS(void);
static void Task_10MS(void);
static void Task_20MS(void);

//Creat_Task
//创建新的任务
//返回1成功，返回0表示堆栈已满，创建失败
u8 Creat_Task(TASK_Stack *stack, u32 cycle, void (*function)(void));


//任务初始化函数
//设置任务的周期以及相关标志位的清零
void Task_Module_init(TASK_Module *pTask, u32 cycle);

//任务堆栈初始化函数
void TASK_Stack_Init(TASK_Stack *stack);

void easy_task(void);//粗糙的任务函数
void Task_roop(void);//任务循环函数

void Task_disapart(void);//任务分发，主要用于主函数
void Task_start(void);
void Task_stop(void);

#endif
