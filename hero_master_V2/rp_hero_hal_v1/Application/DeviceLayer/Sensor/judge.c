#include "judge.h"
#include "launcher.h"
#include "can_potocol.h"

JUDGE_MODULE_DATA Judge_Hero;
int shoot_big_num = 0;
uint32_t judge_offline_cnt = 0;
uint8_t Judge_Offline_flag = 0;
void CAN_Judge_Analyze(uint16_t id, uint8_t *databuff)
{
	uint8_t i = 0;
	uint32_t temp;
	
	
	if(id == JUDGE_CAN_ID_STATE)
	{
		Judge_Hero.robot_state.shooter_42mm_speed_limit = databuff[0]<<8 |  databuff[1];
		Judge_Hero.robot_state.max_chassis_power = databuff[2]<<8 |  databuff[3];
		Judge_Hero.robot_state.shooter_42mm_cooling_limit = databuff[4]<<8 | databuff[5];
		Judge_Hero.robot_state.shooter_42mm_cooling_rate = databuff[6]<<8 | databuff[7];
//		if(Judge_Hero.robot_state.shooter_42mm_cooling_limit < 100 && Judge_Hero.robot_state.shooter_42mm_cooling_limit > 10)
//		{
//			Judge_Hero.robot_state.shooter_42mm_cooling_limit = 300;
//		}
	}
	else if(id == JUDGE_CAN_ID_OUT_HEAT2)
	{
		
		Judge_Hero.out_and_heat.chassis_volt = databuff[0]<<8 | databuff[1];
		Judge_Hero.out_and_heat.chassis_current = databuff[2]<<8 | databuff[3];
		
	}
	else if(id == JUDGE_CAN_ID_OUT_HEAT1)
	{
		
		for(i = 0; i<4; i++)//这个是读取float数据的，先用U32的temp装数据
		{
			temp = temp<<8;
			temp |= databuff[i];
		}
		Judge_Hero.out_and_heat.chassis_power = *((float *)(&temp));//然后通过指针访问数据，转成float
		
		
		Judge_Hero.out_and_heat.chassis_power_buffer = (databuff[4]<<8) | databuff[5];//缓冲能量
		Judge_Hero.out_and_heat.shooter_heat2_42mm = (databuff[6]<<8) | databuff[7];//枪口热量
		judge_offline_cnt = 0;
	}
	else if(id == JUDGE_CAN_ID_STATE2)
	{
		Judge_Hero.robot_state.robot_id = databuff[0];
		Judge_Hero.robot_state.robot_level = databuff[1];
		Judge_Hero.robot_state.remain_HP = databuff[2]<<8 |  databuff[3];
		Judge_Hero.robot_state.max_HP = databuff[4]<<8 | databuff[5];
	}
	else if(id == JUDGE_CAN_ID_SHOOT)
	{
		Judge_Hero.shoot_data.bullet_type = databuff[0];
		Judge_Hero.shoot_data.shooter_id = databuff[1];
		Judge_Hero.shoot_data.bullet_freq = databuff[2];
		for(i = 0; i<4; i++)//这个是读取float数据的，先用U32的temp装数据
		{
			temp = temp<<8;
			temp |= databuff[i+3];
		}
		Judge_Hero.shoot_data.bullet_speed = *((float *)(&temp));//然后通过指针访问数据，转成float
		Judge_Hero.shoot_flag = 1;
		shoot_big_num++;
		
		
		if(Friction_speed<4000)
		{
			if(Judge_Hero.shoot_data.bullet_speed>9.6f)
			{
				Fric_3508_speed[2] -= 15;
			}
			else if(Judge_Hero.shoot_data.bullet_speed<9.3f)
			{
				Fric_3508_speed[2] += 15;
			}
		}
		else if(Friction_speed >4000)
		{
			if(Judge_Hero.shoot_data.bullet_speed>15.6f)
			{
				Fric_3508_speed[5] -= 20;
			}
			else if(Judge_Hero.shoot_data.bullet_speed<15.2f)
			{
				Fric_3508_speed[5] += 20;
			}
		}
	}
	else if(id == JUDGE_CAN_ID_HURT)
	{
		
	}
	else if(id == JUDGE_CAN_ID_BUFF)
	{
		
	}
	else if(id == JUDGE_CAN_ID_RFID)
	{

	}
	
}


