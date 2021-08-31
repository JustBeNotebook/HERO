#include "supercap.h"

Super_Cap_Receive_DATA SC_R_DATA;
Super_Cap_Transmit_DATA SC_T_DATA;

void super_cap_receive(u8* buff)
{
	memcpy(&SC_R_DATA, buff, 8);
}

void super_cap_Transmit_update(void)
{
	
	SC_T_DATA.order = Slaver_flag.shift_rush;
	SC_T_DATA.power_buff = Judge_Hero.out_and_heat.chassis_power_buffer;
	SC_T_DATA.power_limit = Judge_Hero.robot_status.max_chassis_power;
	SC_T_DATA.power_real_time = Judge_Hero.out_and_heat.chassis_power;
	
}

