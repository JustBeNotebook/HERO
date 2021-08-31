#include "control.h"

/* Exported variables --------------------------------------------------------*/
short CAN1_0X1ff_BUF[4] = { 0 };
short CAN1_0X200_BUF[4] = { 0 };
short CAN1_0X2ff_BUF[4] = { 0 };

short CAN2_0X1ff_BUF[4] = { 0 };
short CAN2_0X200_BUF[4] = { 0 };
short CAN2_0X2ff_BUF[4] = { 0 };

short CAN1_0X150_BUF[4] = { 0 };
short CAN1_0X151_BUF[4] = { 0 };

static void CONTROL_Output(void);
static void CONTROL_Reset(void);
static void CONTROL_SelfProtect(void);

Client_Slave_Flag Slaver_flag;

control_t control = {
	.reset = CONTROL_Reset,
	.output  = CONTROL_Output,
	.self_protect = CONTROL_SelfProtect,
};

void Client_flag_update(void)
{
	Slaver_flag.global_anti_top = (uint8_t)anti_top_flag;
	Slaver_flag.global_auto_aim = (uint8_t)((vision_sensor.info->identify_target)==1)?1:0;
	Slaver_flag.global_clip = (uint8_t)launcher.info->launcher_mgz_flag;
	Slaver_flag.global_fiction = (uint8_t)launcher.info->launcher_friction_flag;
	Slaver_flag.global_spin = (uint8_t)chassis.info->top_gyro;
	Slaver_flag.global_twist = (uint8_t)dial_empty_flag;
	Slaver_flag.shift_rush =	(uint8_t)N2O_Trigger;
	Slaver_flag.user1 = 0;
}

extern uint16_t shoot_heat;

static void CONTROL_Output(void)
{
	uint8_t i = 0;
	uint8_t CAN1_0X150_BUF_temp[8];
	float pFloat[2];
	int shoot_heat_int = shoot_heat;
	
	pFloat[0] = (float)(gimbal_motor[GIMBAL_M_PITCH].info->angle);//pitch角度
	pFloat[1] = (float)(gimbal_motor[GIMBAL_YAW].info->angle);//yaw角度
	pFloat[1] = shoot_heat;
	
	memcpy(CAN1_0X150_BUF_temp, pFloat, 8);
	CAN1_SendData_memcpy(0x150,	CAN1_0X150_BUF_temp);
	
	CAN_SendDataBuff(DRV_CAN2, 0x200,	CAN2_0X200_BUF);
	CAN_SendDataBuff(DRV_CAN1, 0x200,	CAN1_0X200_BUF);
	
	Client_flag_update();
	memcpy(CAN1_0X151_BUF, &Slaver_flag, 8);
	CAN_SendDataBuff(DRV_CAN1, 0x151,	CAN1_0X151_BUF);
	
	CAN_SendDataBuff(DRV_CAN2, 0x1ff,	CAN2_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN1, 0x1ff,	CAN1_0X1ff_BUF);
	
	for(i= 0; i<4; i++)
	{
		CAN1_0X1ff_BUF[i] = 0;
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X1ff_BUF[i] = 0;
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}
}

//保留了云台电机的输出
static void CONTROL_Reset(void)
{
	uint8_t i = 0;

	for(i= 0; i<4; i++)
	{
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}
	
	
	CAN_SendDataBuff(DRV_CAN2, 0x1ff,	CAN2_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN2, 0x200,	CAN2_0X200_BUF);
	
	CAN_SendDataBuff(DRV_CAN1, 0x1ff,	CAN1_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN1, 0x200,	CAN1_0X200_BUF);
	
	for(i= 0; i<4; i++)
	{
		CAN1_0X1ff_BUF[i] = 0;
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X1ff_BUF[i] = 0;
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}
}

static void CONTROL_SelfProtect(void)
{
	uint8_t i = 0;

	for(i= 0; i<4; i++)
	{
		CAN1_0X1ff_BUF[i] = 0;
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X1ff_BUF[i] = 0;
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}
	
	CAN_SendDataBuff(DRV_CAN2, 0x1ff,	CAN2_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN2, 0x200,	CAN2_0X200_BUF);
	
	CAN_SendDataBuff(DRV_CAN1, 0x1ff,	CAN1_0X1ff_BUF);
	CAN_SendDataBuff(DRV_CAN1, 0x200,	CAN1_0X200_BUF);
}



