#ifndef __CONTROL_H
#define __CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "can_potocol.h"
#include "chassis.h"
#include "chassis_motor.h"
#include "gimbal.h"
#include "gimbal_motor.h"
#include "launcher.h"
#include "launcher_motor.h"

#include "rc_sensor.h"
#include "string.h"
#include "vision_sensor.h"


/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/


typedef struct control{
	void 			(*reset)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}control_t;

typedef __packed struct Client_Slave_Flag
{
	uint8_t global_fiction;
	uint8_t global_clip;
	uint8_t global_spin;
	uint8_t global_auto_aim;
	uint8_t global_twist;
	uint8_t global_anti_top;
	uint8_t shift_rush;
	uint8_t user1;
}Client_Slave_Flag;

extern short CAN1_0X1ff_BUF[4];
extern short CAN1_0X200_BUF[4];
extern short CAN1_0X2ff_BUF[4];

extern short CAN2_0X1ff_BUF[4];
extern short CAN2_0X200_BUF[4];
extern short CAN2_0X2ff_BUF[4];

extern short CAN1_0X151_BUF[4];

extern control_t control;
extern Client_Slave_Flag Slaver_flag;
/* Exported functions --------------------------------------------------------*/
/* –≈œ¢≤„ --------------------------------------------------------------------*/
//bool CHASSIS_IfBackToMiddleAngle(void);

#endif
