#include "chassis.h"



unknow_Module UNKONW;

unknow_Module UNKONW2;

u8 Motor_analyze2(u16 id, u8 *buf)
{
	
	switch(id)		//ID筛选，如果不是想要的id，跳出函数
	{
		case 0x201: 
		case 0x202: 
		case 0x203: 
		case 0x204:	
		case 0x205:
		case 0x206:
		case 0x207:
		case 0x208:
		case 0x209:
		case 0x20a:
		case 0x20b:
		case 0x20c:
		case 0x20d:
		case 0x20e:
		case 0x20f:id = id&0xf; break;
		default: return 1;
	}

	if(id == 2)
	{
		Motor_analyze(&(Hero_Launcher.Safety), buf);
	}
	else if(id == 3)
	{
		Motor_analyze(&(Hero_Launcher.Dial), buf);
	}
	else if(id == 6)
	{
		Motor_analyze(&(Hero_Launcher.Magazine), buf);
	}
	else if(id<12)
	{
		Motor_analyze(&(UNKONW2.ID5_F[id]), buf);
	}
	
	return 0;
}

