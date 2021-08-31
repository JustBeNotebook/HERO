#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "sys.h"
#include "core_cm4.h"


typedef struct timestamp
{
	u16 day;
	u16 hour;
	u16 min;
	u16 sec;
	u16 msec;
	u32 msec_cnt;
	u16 usec;
}timestamp;

extern timestamp SysTimeStamp;

//滴答定时器初始化函数，滴答定时器的配置
void systick_init(u32 nus);
//滴答定时器代替中断函数
void systick_interrupt(void);

//systick 专用延时函数
void sys_delay_us(u32 nus);
void sys_delay_ms(u32 nms);
u32 timestamp_getms(void);

//sys 时间戳计数函数
//由于一个系统用不到太多时间戳，所以不以结构体指针作为输入了
void timestamp_step_100us(void);
void timestamp_step_ms(void);
void timestamp_init(void);

#endif
