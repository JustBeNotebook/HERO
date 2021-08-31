#include "systick.h"
#include "core_cm4.h"
#include "task.h"

u32 sys_us_cnt = 0;
u32 sys_ms_cnt = 0;

timestamp SysTimeStamp;

RCC_ClocksTypeDef RCC_Clocks;//有生成实体
void systick_init(u32 nus)//按照给定的微秒数进行延时，仅对于系统时钟频率大于1M以上的时钟
{
	u32 reload_val;
	//RCC_ClocksTypeDef* RCC_Clocks;//没生成实体
	
	
	RCC_GetClocksFreq(&RCC_Clocks);//获取系统时钟资源
	
	reload_val = (u32)(RCC_Clocks.HCLK_Frequency /1000000);//1us 计数值
	//reload_val = 168;//1us 计数值
	//reload_val = (u32)(reload_val/8);//8分频
	sys_us_cnt = reload_val;
	sys_ms_cnt = reload_val*1000;
	reload_val = reload_val*nus;     //按照给定值实现systick中断时间
	
	SysTick_Config(reload_val);
	
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Pos   |
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	
}

//根据systick初始化设定的毫秒数执行的中断
void systick_interrupt()
{

	//u16 i = 0; //忘记写static
	static u32 i = 0;
	i++;

	timestamp_step_ms();
	//timestamp_step_100us();
	Task_roop();
	
	if(i>=10000)
		i -= 10000;
}



//系统微秒级延时
void sys_delay_us(u32 nus)
{
	u32 last_temp = 0, now_temp = 0;//
	u32 nus_cnt = 0, temp = 0;
	
	last_temp = SysTick->VAL;
	
	while(nus_cnt<nus)
	{
		now_temp = SysTick->VAL;	//读取当前定时器的计数器值
		
		if(now_temp >= last_temp)  //减法获取经过的时间，溢出判断, 因为是计时器是倒数的，所以需要用last减去now的值才是时钟跑过的值
			temp += last_temp+(SysTick->LOAD) - now_temp;
		else 
			temp += last_temp-now_temp;
		
		last_temp = now_temp;
		if(temp>sys_us_cnt)				//当前计数值大于最小毫米单位
		{

			nus_cnt += temp/sys_us_cnt;
			temp = temp%sys_us_cnt;
		}
	}
}


//系统毫秒级延时
void sys_delay_ms(u32 nms)
{
	u32 nms_cnt = 0;
	
	for(nms_cnt = 0; nms_cnt<nms; nms_cnt++)
	{
		sys_delay_us(1000);
	}
}

//百微秒任务
void timestamp_step_100us(void)
{
	SysTimeStamp.usec += 100;
	if(SysTimeStamp.usec>=1000)
	{
		SysTimeStamp.usec -= 1000;
		timestamp_step_ms();
	}
}

//时间戳步进
//放到1ms任务里面
void timestamp_step_ms(void)
{
	SysTimeStamp.msec++;
	SysTimeStamp.msec_cnt++;//开机经过的总毫秒数
	if(SysTimeStamp.msec >= 1000)
	{
		SysTimeStamp.msec -= 1000;
		SysTimeStamp.sec += 1;
		if(SysTimeStamp.sec >= 60)
		{
			SysTimeStamp.sec -= 60;
			SysTimeStamp.min += 1;
			if(SysTimeStamp.min >= 60)
			{
				SysTimeStamp.min -= 60;
				SysTimeStamp.hour += 1;
				if(SysTimeStamp.hour >= 24)
				{
					SysTimeStamp.hour -= 24;
					SysTimeStamp.day += 1;
					if(SysTimeStamp.day >= 65530)
					{
						SysTimeStamp.day = 0;
					}
				}
			}
		}
	}
}

//TIMESTAMP初始化
void timestamp_init(void)
{
	SysTimeStamp.day = 0;
	SysTimeStamp.hour = 0;
	SysTimeStamp.min = 0;
	SysTimeStamp.sec = 0;
	SysTimeStamp.msec = 0;
	SysTimeStamp.usec = 0;
}


u32 timestamp_getms(void)
{
	return SysTimeStamp.msec_cnt;
}


