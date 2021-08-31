#ifndef RM__SUPER_CAP_H
#define RM__SUPER_CAP_H

#include "link.h"
#include "my_judge.h"

typedef __packed struct 
{
	uint8_t order;
	uint8_t power_limit;
	float power_real_time;
	uint16_t power_buff;
}Super_Cap_Transmit_DATA;

typedef __packed struct
{
	float volt;
	float current;
}Super_Cap_Receive_DATA;

extern Super_Cap_Receive_DATA SC_R_DATA;
extern Super_Cap_Transmit_DATA SC_T_DATA;

void super_cap_receive(u8* buff);
void super_cap_Transmit_update(void);

#endif
