#include "task.h"


TASK_Stack TASK_Stack_ms_level;
u8 Task_Start_Flag = 0;
u32 reload1 = 500, reload2 = 500;

u32 n2 = 0;
u32 n20 = 0;

//用于判断任务时间
void Task_roop(void)
{
	u8 i;
	TASK_Module *task_temp;
	
	for(i = 0; i<(TASK_Stack_ms_level.stack_pot); i++)
	{
		task_temp = &(TASK_Stack_ms_level.Stack[i]);
		task_temp->cnt++;//计数+1
		if(task_temp->cnt >= task_temp->cycle)
		{
			switch(task_temp->flag)							//重设任务进行状态
			{
				 //处于挂起状态时将任务转换成就绪态，并且将计数值减掉一个周期
				case SUSPENDED: task_temp->flag = READY;
												(task_temp->cnt -= task_temp->cycle); break;
				case READY: ;break;
				default: ;break;
			}
		}
	}
}

//TASK_Init(void)
//执行任务总初始化
void TASK_Init(void)
{
	TASK_Stack_Init(&TASK_Stack_ms_level);//任务堆栈初始化
	
	//创建任务函数
	Creat_Task(&TASK_Stack_ms_level, 20, Task_20MS);
	Creat_Task(&TASK_Stack_ms_level, 10, Task_10MS);
	Creat_Task(&TASK_Stack_ms_level, 2, Task_2MS);
	Creat_Task(&TASK_Stack_ms_level, 1, Task_1MS);
	
}

short gyrox, gyroy, gyroz;//角速度值 x是横滚角速度，y是pitch角速度，z是偏航角速度
float chip_pitch, chip_roll, chip_yaw;		//角度值   pitch 是俯仰角度 前倾为负， roll是横滚角度 右低为负，yaw是偏航角度 顺时针为负

u8 relife_flag = 0;
//xms 任务函数
	float power = 0;
	u16 power_buf = 0;
	u16 shoot_heat = 0;
	static u32 n1 = 0;
static void Task_1MS(void)
{
	u32 task_1ms_cnt = 0;
	static u8 state_cnt = 0;
	static u8 top_state = 0;
	
	u8 i;
	u8 *pChar;
	u32 temp = 0;	
	IMUsensor_GetData();
	if(Judge_Hero.out_heat_flag == 1)
	{
		pChar = (u8 *)(&Judge_Hero.out_and_heat.chassis_power);
		
		CAN1_0X1af_BUF[0] = *(pChar+3);	//小端存储
		CAN1_0X1af_BUF[1] = *(pChar+2);
		CAN1_0X1af_BUF[2] = *(pChar+1);
		CAN1_0X1af_BUF[3] = *(pChar+0);
		
		CAN1_0X1af_BUF[4] = Judge_Hero.out_and_heat.chassis_power_buffer >> 8;
		CAN1_0X1af_BUF[5] = Judge_Hero.out_and_heat.chassis_power_buffer & 0xff;
		
		CAN1_0X1af_BUF[6] = Judge_Hero.out_and_heat.shooter_heat2_42mm >> 8;
		CAN1_0X1af_BUF[7] = Judge_Hero.out_and_heat.shooter_heat2_42mm & 0xff;
		
		CAN1_Send_Msg2(CAN1_0X1af_BUF, 8, 0X1af);
		//CAN2_Send_Msg2(CAN1_0X1af_BUF, 8, 0X1af);
		
		CAN1_0X1ae_BUF[0] = Judge_Hero.out_and_heat.chassis_volt>>8;
		CAN1_0X1ae_BUF[1] = Judge_Hero.out_and_heat.chassis_volt & 0xff;
		CAN1_0X1ae_BUF[2] = Judge_Hero.out_and_heat.chassis_current>>8;
		CAN1_0X1ae_BUF[3] = Judge_Hero.out_and_heat.chassis_current & 0xff;

		
		if(Judge_Hero.buff_flag == 1)//增益数据
		{
			CAN1_0X1ae_BUF[4] = 1;
			CAN1_0X1ae_BUF[5] = Judge_Hero.buff.power_rune_buff;
			Judge_Hero.buff_flag = 0;
		}
		else
		{
			CAN1_0X1ae_BUF[4] = 0;
			CAN1_0X1ae_BUF[5] = 0;
		}
		
		if(Judge_Hero.hurt_flag == 1)//受伤数据
		{
			CAN1_0X1ae_BUF[6] = 1;
			CAN1_0X1ae_BUF[7] = Judge_Hero.robot_hurt.armor_id<<4 | Judge_Hero.robot_hurt.hurt_type;
			Judge_Hero.hurt_flag = 0;
		}
		else
		{
			CAN1_0X1ae_BUF[6] = 0;
			CAN1_0X1ae_BUF[7] = 0;
		}
		
 		CAN1_Send_Msg2(CAN1_0X1ae_BUF, 8, 0X1ae);
		//CAN2_Send_Msg2(CAN1_0X1ae_BUF, 8, 0X1ae);
		
		Judge_Hero.out_heat_flag = 0;
		
		n1++;
	}
	
	if(Judge_Hero.state_flag == 1)
	{
		
		
		CAN1_0X1ac_BUF[0] = (uint8_t)(Judge_Hero.robot_status.robot_id);
		CAN1_0X1ac_BUF[1] = (uint8_t)(Judge_Hero.robot_status.robot_level);
		CAN1_0X1ac_BUF[2] = (uint8_t)(Judge_Hero.robot_status.remain_HP>>8);
		CAN1_0X1ac_BUF[3] = (uint8_t)(Judge_Hero.robot_status.remain_HP & 0xff);
		CAN1_0X1ac_BUF[4] = (uint8_t)(Judge_Hero.robot_status.max_HP>>8);
		CAN1_0X1ac_BUF[5] = (uint8_t)(Judge_Hero.robot_status.max_HP & 0xff);
		
		
		
		CAN1_0X1ad_BUF[0] = (uint8_t)(Judge_Hero.robot_status.shooter_42mm_speed_limit>>8);
		CAN1_0X1ad_BUF[1] = (uint8_t)(Judge_Hero.robot_status.shooter_42mm_speed_limit & 0xff);
		CAN1_0X1ad_BUF[2] = (uint8_t)(Judge_Hero.robot_status.max_chassis_power>>8);
		CAN1_0X1ad_BUF[3] = (uint8_t)(Judge_Hero.robot_status.max_chassis_power & 0Xff);
		CAN1_0X1ad_BUF[4] = (uint8_t)(Judge_Hero.robot_status.shooter_42mm_cooling_limit>>8);
		CAN1_0X1ad_BUF[5] = (uint8_t)(Judge_Hero.robot_status.shooter_42mm_cooling_limit & 0xff);
		CAN1_0X1ad_BUF[6] = (uint8_t)(Judge_Hero.robot_status.shooter_42mm_cooling_rate>>8);
		CAN1_0X1ad_BUF[7] = (uint8_t)(Judge_Hero.robot_status.shooter_42mm_cooling_rate & 0xff);
		
		if(state_cnt == 0)
		{
			CAN1_Send_Msg2(CAN1_0X1ac_BUF, 8, 0X1ac);
			state_cnt = 1;
		}
		else if(state_cnt == 1)
		{
			CAN1_Send_Msg2(CAN1_0X1ad_BUF, 8, 0X1ad);
			Judge_Hero.state_flag = 0;
			state_cnt = 0;
		}	
		
		
	}
	
	if(Judge_Hero.shoot_flag == 1)
	{
		CAN1_0X1ab_BUF[0] = Judge_Hero.shoot_data.bullet_type;
		CAN1_0X1ab_BUF[1] = Judge_Hero.shoot_data.shooter_id;
		CAN1_0X1ab_BUF[2] = Judge_Hero.shoot_data.bullet_freq;
		
		pChar = (u8 *)(&Judge_Hero.shoot_data.bullet_speed);
		
		CAN1_0X1ab_BUF[3] = *(pChar+3);	//小端存储
		CAN1_0X1ab_BUF[4] = *(pChar+2);
		CAN1_0X1ab_BUF[5] = *(pChar+1);
		CAN1_0X1ab_BUF[6] = *(pChar+0);
		
		CAN1_Send_Msg2(CAN1_0X1ab_BUF, 8, 0X1ab);
		
		Judge_Hero.shoot_flag  = 0;
	}
	
	
	if(Judge_Hero.bullet_flag)
	{
		CAN1_0X1aa_BUF[0] = (Judge_Hero.bullet_remain.bullet_remaining_num_17mm>>8) & 0xff;
		CAN1_0X1aa_BUF[1] = (Judge_Hero.bullet_remain.bullet_remaining_num_17mm) & 0xff;
		CAN1_0X1aa_BUF[2] = (Judge_Hero.bullet_remain.bullet_remaining_num_42mm>>8) & 0xff;
		CAN1_0X1aa_BUF[3] = (Judge_Hero.bullet_remain.bullet_remaining_num_42mm) & 0xff;
		CAN1_0X1aa_BUF[4] = (Judge_Hero.bullet_remain.coin_remain>>8) & 0xff;
		CAN1_0X1aa_BUF[5] = (Judge_Hero.bullet_remain.coin_remain) & 0xff;
		
		CAN1_Send_Msg2(CAN1_0X1aa_BUF, 8, 0X1aa);
		Judge_Hero.bullet_flag = 0;
	}
	
	
	if(Judge_Hero.rfid_flag == 1)
	{
		
		CAN1_0X1a9_BUF[0] = ((Judge_Hero.rfid_status.rfid_status)&0xff000000)>>24;
		CAN1_0X1a9_BUF[1] = ((Judge_Hero.rfid_status.rfid_status)&0x00ff0000)>>16;
		CAN1_0X1a9_BUF[2] = ((Judge_Hero.rfid_status.rfid_status)&0x0000ff00)>>8;
		CAN1_0X1a9_BUF[3] = ((Judge_Hero.rfid_status.rfid_status)&0x000000ff);
		CAN1_0X1a9_BUF[4] = PEin(15);
		CAN1_0X1a9_BUF[5] = 1;
		
		CAN1_Send_Msg2(CAN1_0X1a9_BUF, 8, 0X1a9);
		Judge_Hero.rfid_flag = 0;
	}
	
	//CAN1_Send_Msg2(CAN1_0X401_BUF, 8, 0X401);

	
	
	task_1ms_cnt++;
	//n1++;
}

short int gimbal_out[2] = {0,0};
short int angle_turns[4] = {0, 0, 0, 0};
int angle[4] = {0, 0, 0, 0};

static void Task_2MS(void)
{
	if(n2%20 == 0)
	{
		Client_task();
	}
	n2++;
	n2 = n2 % 20000;
}


float coss;
float sins;
static void Task_10MS(void)
{
	static u32 n10 = 0;
	static u32 i = 0;
	i = n10;
	
	RP_SendToPc(imu_chassis.pitch, chassis.Motor_Num[0].tg_speed, chassis.Motor_Num[0].speed_real, chassis.Motor_Num[1].speed_real, chassis.Motor_Num[2].speed_real, chassis.Motor_Num[3].speed_real);
	
	n10++;
}


u32 ccr1temp, ccr2temp;
static void Task_20MS(void)
{
	super_cap_Transmit_update();
	
	memcpy(CAN2_0X501_BUF, &SC_T_DATA, 8);//发送超电功率信息
	CAN2_Send_Msg2(CAN2_0X501_BUF, 8, 0X501);
	
	if(n20%25 == 0)
		PCout(13) = !PCout(13);
	
	if(n20%50 == 0)
	{
		PCout(14) = !PCout(14);
	}
	
	
	
	n20++;
	n20 = n20%20000;
}


//任务初始化函数
//设置任务的周期以及相关标志位的清零
void Task_Module_init(TASK_Module *pTask, u32 cycle)
{
	pTask->cycle = cycle;
	pTask->flag = SUSPENDED;
}


//任务堆栈初始化函数
void TASK_Stack_Init(TASK_Stack *stack)
{
	u8 i = 0;
	stack->stack_base = 0;
	stack->stack_pot = 0;
	for(i = 0; i<10; i++)
	{
		Task_Module_init(&(stack->Stack[i]), 0);
	}
	
}

//Creat_Task
//创建新的任务
u8 Creat_Task(TASK_Stack *stack, u32 cycle, void (*function)(void))
{
	if(stack->stack_pot >= STACK_SIZE)
		return 0;		//堆栈满了
	
	stack->Stack[stack->stack_pot].cycle = cycle;
	stack->Stack[stack->stack_pot].flag = SUSPENDED;
	stack->Stack[stack->stack_pot].function = function;
	stack->Stack[stack->stack_pot].cnt = 0;
	
	stack->stack_pot++;  //
	
	return 1;
}


//任务分发，主要用于主函数
void Task_disapart(void)
{
	u8 i;
	TASK_Module *task_temp;
	
	if(Task_Start_Flag == 0)
		return;
	
	for(i = 0; i<(TASK_Stack_ms_level.stack_pot); i++)//循环判断标志位是否设置为就绪
	{
		task_temp = &(TASK_Stack_ms_level.Stack[i]);
		if(task_temp->flag == READY)
		{
			task_temp->flag = RUNNING;
			task_temp->function();
			task_temp->flag = SUSPENDED;
		}
	}
	
}


//任务开启函数
void Task_start(void)
{
	Task_Start_Flag = 1;
}

//任务开启函数
void Task_stop(void)
{
	Task_Start_Flag = 0;
}



