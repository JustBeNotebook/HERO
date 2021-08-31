#ifndef RP__DR16__H
#define RP__DR16__H

#include "link.h"

typedef struct Mouse_Module
{
	short int x;
	short int y;
	short int z;
	
	u8 l;
	u8 r;
}Mouse_Module;


typedef struct DR16_Module
{
	short int ch0;
	short int ch1;
	short int ch2;
	short int ch3;

	u8 sw1;
	u8 sw2;
	
	Mouse_Module mouse;
	
	union{
		u16 key_code;		
		struct
		{
			u16 W:1;
			u16 S:1;
			u16 A:1;
			u16 D:1;
			u16 SHIFT:1;
			u16 CTRL:1;
			u16 Q:1;
			u16 E:1;
			u16 R:1;
			u16 F:1;
			u16 G:1;
			u16 Z:1;
			u16 X:1;
			u16 C:1;
			u16 V:1;
			u16 B:1;
		}bit;
	
	}kb;
	
	short int wheel;
	
}DR16_Module;

typedef struct RC_Module
{
	DR16_Module rece_Data;
	
	OffLine_Judge_Module Judge;
}RC_Module;


extern RC_Module DJI_RC1;

//dr16 接收函数
void RC_Init(u32 bound);
void RC_Init2(void);


void get_dr16_data(DR16_Module *dr16, volatile unsigned char *pData);

void RC_data_Init(DR16_Module *dr16);
u8 DR_data_check(DR16_Module *dr16);
//DR16初始化函数

u8 RC_Reconnecting_func(void);


#endif
