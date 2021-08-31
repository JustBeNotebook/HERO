/**
 * @file        launcher_motor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       launcher Motor(RM3508).
 */
 
/* Includes ------------------------------------------------------------------*/
#include "launcher_motor.h"

#include "can_potocol.h"
#include "rp_math.h"

extern void launcher_motor_update(launcher_motor_t *motor, uint8_t *rxBuf);
extern void launcher_motor_init(launcher_motor_t *motor);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void launcher_motor_check(launcher_motor_t *motor);
static void launcher_motor_heart_beat(launcher_motor_t *motor);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 发射电机驱动
drv_can_t		launcher_motor_driver[] = {
	[LAUNCHER_MGZ] = {
		.type = DRV_CAN2,
		.can_id = LAUNCHER_CAN_ID_MGZ,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[LAUNCHER_SAFE] = {
		.type = DRV_CAN2,
		.can_id = LAUNCHER_CAN_ID_SAFE,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[LAUNCHER_DIAL] = {
		.type = DRV_CAN2,
		.can_id = LAUNCHER_CAN_ID_DIAL,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[LAUNCHER_FRICT_L] = {
		.type = DRV_CAN2,
		.can_id = LAUNCHER_CAN_ID_FRICT_L,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[LAUNCHER_FRICT_R] = {
		.type = DRV_CAN2,
		.can_id = LAUNCHER_CAN_ID_FRICT_R,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
};

// 发射电机信息
launcher_motor_info_t 	launcher_motor_info[] = {
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
		.offline_max_cnt = 10,
	},
	{
		.offline_max_cnt = 10,
	},

};

// 发射电机传感器
launcher_motor_t		launcher_motor[] = {
	[LAUNCHER_MGZ] = {
		.info = &launcher_motor_info[LAUNCHER_MGZ],
		.driver = &launcher_motor_driver[LAUNCHER_MGZ],
		.init = launcher_motor_init,
		.update = launcher_motor_update,
		.check = launcher_motor_check,
		.heart_beat = launcher_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_LAUNCHER_MGZ,
	},
	[LAUNCHER_SAFE] = {
		.info = &launcher_motor_info[LAUNCHER_SAFE],
		.driver = &launcher_motor_driver[LAUNCHER_SAFE],
		.init = launcher_motor_init,
		.update = launcher_motor_update,
		.check = launcher_motor_check,
		.heart_beat = launcher_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_LAUNCHER_SAFE,
	},
	[LAUNCHER_DIAL] = {
		.info = &launcher_motor_info[LAUNCHER_DIAL],
		.driver = &launcher_motor_driver[LAUNCHER_DIAL],
		.init = launcher_motor_init,
		.update = launcher_motor_update,
		.check = launcher_motor_check,
		.heart_beat = launcher_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_LAUNCHER_DIAL,
	},
	[LAUNCHER_FRICT_L] = {
		.info = &launcher_motor_info[LAUNCHER_FRICT_L],
		.driver = &launcher_motor_driver[LAUNCHER_FRICT_L],
		.init = launcher_motor_init,
		.update = launcher_motor_update,
		.check = launcher_motor_check,
		.heart_beat = launcher_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_LAUNCHER_FRICT_L,
	},
	[LAUNCHER_FRICT_R] = {
		.info = &launcher_motor_info[LAUNCHER_FRICT_R],
		.driver = &launcher_motor_driver[LAUNCHER_FRICT_R],
		.init = launcher_motor_init,
		.update = launcher_motor_update,
		.check = launcher_motor_check,
		.heart_beat = launcher_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_LAUNCHER_FRICT_R,
	},
};

/* Private functions ---------------------------------------------------------*/
static void launcher_motor_check(launcher_motor_t *motor)
{
	int16_t err;
	launcher_motor_info_t *motor_info = motor->info;
	
	/* 未初始化 */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = true;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/* 过零点 计算累计角度*/
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

static void launcher_motor_heart_beat(launcher_motor_t *motor)
{
	launcher_motor_info_t *motor_info = motor->info;
	
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

