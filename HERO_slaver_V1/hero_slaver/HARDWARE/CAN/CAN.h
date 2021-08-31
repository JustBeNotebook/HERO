#ifndef RP__CAN_H
#define RP__CAN_H

#include "link.h"

//CAN接收数据在CAN.c 的中断里接收


u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);// CAN模式初始化
u8 CAN1_Send_Msg2(u8 *msg, u8 len, u16 stdid);								//CAN1发送数据

u8 CAN2_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);// CAN模式初始化
u8 CAN2_Send_Msg2(u8 *msg, u8 len, u16 stdid);								//CAN1发送数据

#endif

