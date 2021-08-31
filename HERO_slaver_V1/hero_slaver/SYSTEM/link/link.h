#ifndef RP__LINK__H
#define RP__LINK__H

#include "sys.h"
#include "systick.h"
#include <stdbool.h>
#include "string.h"
#include "crc.h"
#include "top.h" 
#include "myiic.h"

#include "task.h"
#include "pid.h"
#include "can.h"
#include "offline_judge.h"
#include "Motor.h"

#include "timer.h"

#include "led.h"
#include "dr16.h"

#include "my_judge.h"
#include "chassis.h"
#include "gimbal.h"
#include "launcher.h"
#include "IMUsensor.h"

#define abs(x) 					((x)>0? (x):(-(x)))

//////////////////////////////////////////
//系统初始化函数
void System_Init(void);

//////////////////////////////////////////


//////////////////////////////////////////
//全部Judge的更新函数
void ALL_Judge_Flash(void);
void ALL_Judge_Config(void);
///////////////////////////////////////////

///////////////////////////////////////////
extern u8 CAN1_0X1ff_BUF[8];
extern u8 CAN1_0X200_BUF[8];
extern u8 CAN1_0X2ff_BUF[8];

extern u8 CAN2_0X1ff_BUF[8];
extern u8 CAN2_0X200_BUF[8];
extern u8 CAN2_0X2ff_BUF[8];

extern u8 CAN1_0X1a9_BUF[8];
extern u8 CAN1_0X1aa_BUF[8];
extern u8 CAN1_0X1ab_BUF[8];
extern u8 CAN1_0X1ac_BUF[8];
extern u8 CAN1_0X1ad_BUF[8];
extern u8 CAN1_0X1ae_BUF[8];
extern u8 CAN1_0X1af_BUF[8];

extern u8 CAN2_0X501_BUF[8];
//CAN 全部ID发送函数
void Can_Total_sent(void);
//CAN发送全部清0
void Can_Sent_Clear(void);
///////////////////////////////////////////


//////////////////////////////////////////
//遥控控制底盘函数
void DR16_C_Chassis(void);
void DR16_C_Gimbal(void);
//机械模式
void Machanistic(void);
//云台模式
void Gimbal_Mode(void);
void Top_Mode(void);
////////////////////////////////////////////


//////////////////////////////////////////
//////////////////////////////////////////


//////////////////////////////////////////
//JUDGE 判定失联相关函数
extern u32 gyro_reset_cnt;
void RC_JUDGE(void);
void RC_Reconnecting(void);
//////////////////////////////////////////


void RP_SendToPc(float yaw, float pitch, float roll, float rateYaw, float ratePitch, float rateRoll);
//////////////////////////////////////////

#endif

