/**
 * @file        chassis_motor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       Chassis Motor(RM3508).
 */
 
/* Includes ------------------------------------------------------------------*/
#include "chassis_motor.h"

#include "can_potocol.h"
#include "rp_math.h"

extern void chassis_motor_update(chassis_motor_t *motor, uint8_t *rxBuf);
extern void chassis_motor_init(chassis_motor_t *motor);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void chassis_motor_check(chassis_motor_t *motor);
static void chassis_motor_heart_beat(chassis_motor_t *motor);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 底盘电机驱动
drv_can_t		chassis_motor_driver[] = {
	[CHAS_LF] = {
		.type = DRV_CAN1,
		.can_id = CHASSIS_CAN_ID_LF,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[CHAS_RF] = {
		.type = DRV_CAN1,
		.can_id = CHASSIS_CAN_ID_RF,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[CHAS_LB] = {
		.type = DRV_CAN1,
		.can_id = CHASSIS_CAN_ID_LB,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[CHAS_RB] = {
		.type = DRV_CAN1,
		.can_id = CHASSIS_CAN_ID_RB,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
};

// 底盘电机信息
chassis_motor_info_t 	chassis_motor_info[] = {
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
};

// 底盘电机传感器
chassis_motor_t		chassis_motor[] = {
	[CHAS_LF] = {
		.info = &chassis_motor_info[CHAS_LF],
		.driver = &chassis_motor_driver[CHAS_LF],
		.init = chassis_motor_init,
		.update = chassis_motor_update,
		.check = chassis_motor_check,
		.heart_beat = chassis_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS_LF,
	},
	[CHAS_RF] = {
		.info = &chassis_motor_info[CHAS_RF],
		.driver = &chassis_motor_driver[CHAS_RF],
		.init = chassis_motor_init,
		.update = chassis_motor_update,
		.check = chassis_motor_check,
		.heart_beat = chassis_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS_RF,
	},
	[CHAS_LB] = {
		.info = &chassis_motor_info[CHAS_LB],
		.driver = &chassis_motor_driver[CHAS_LB],
		.init = chassis_motor_init,
		.update = chassis_motor_update,
		.check = chassis_motor_check,
		.heart_beat = chassis_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS_LB,
	},
	[CHAS_RB] = {
		.info = &chassis_motor_info[CHAS_RB],
		.driver = &chassis_motor_driver[CHAS_RB],
		.init = chassis_motor_init,
		.update = chassis_motor_update,
		.check = chassis_motor_check,
		.heart_beat = chassis_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS_RB,
	},
};

/* Private functions ---------------------------------------------------------*/
static void chassis_motor_check(chassis_motor_t *motor)
{
	int16_t err;
	chassis_motor_info_t *motor_info = motor->info;
	
	/* 未初始化 */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = true;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/* 过零点 */
	if(abs(err) > 4095)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			motor_info->angle_sum += -8192 + err;
		/* 8191↑ -> 0 */
		else
			motor_info->angle_sum += 8192 + err;
	}
	/* 未过零点 */
	else
	{
		motor_info->angle_sum += err;
	}
	
	motor_info->angle_prev = motor_info->angle;		
}

static void chassis_motor_heart_beat(chassis_motor_t *motor)
{
	chassis_motor_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) {
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
	}
	else {
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
}

/* Exported functions --------------------------------------------------------*/

