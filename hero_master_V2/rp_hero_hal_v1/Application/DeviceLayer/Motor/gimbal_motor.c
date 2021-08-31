/**
 * @file        gimbal_motor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       gimbal Motor(RM3508).
 */
 
/* Includes ------------------------------------------------------------------*/
#include "gimbal_motor.h"

#include "can_potocol.h"
#include "rp_math.h"

extern void gimbal_motor_update(gimbal_motor_t *motor, uint8_t *rxBuf);
extern void gimbal_motor_init(gimbal_motor_t *motor);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void gimbal_motor_check(gimbal_motor_t *motor);
static void gimbal_motor_heart_beat(gimbal_motor_t *motor);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
gimbal_motor_info_t gimbal_yaw, gimbal_m_pitch, gimbal_s_pitch;

// 发射电机驱动
drv_can_t		gimbal_motor_driver[] = {
	[GIMBAL_YAW] = {
		.type = DRV_CAN2,
		.can_id = GIMBAL_CAN_ID_YAW,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[GIMBAL_M_PITCH] = {
		.type = DRV_CAN1,
		.can_id = GIMBAL_CAN_ID_M_PITCH,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[GIMBAL_S_PITCH] = {
		.type = DRV_CAN1,
		.can_id = GIMBAL_CAN_ID_S_PITCH,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
};

// 发射电机信息
gimbal_motor_info_t 	gimbal_motor_info[] = {
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

// 发射电机传感器
gimbal_motor_t		gimbal_motor[] = {
	[GIMBAL_YAW] = {
		.info = &gimbal_motor_info[GIMBAL_YAW],
		.driver = &gimbal_motor_driver[GIMBAL_YAW],
		.init = gimbal_motor_init,
		.update = gimbal_motor_update,
		.check = gimbal_motor_check,
		.heart_beat = gimbal_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_YAW,
	},
	[GIMBAL_M_PITCH] = {
		.info = &gimbal_motor_info[GIMBAL_M_PITCH],
		.driver = &gimbal_motor_driver[GIMBAL_M_PITCH],
		.init = gimbal_motor_init,
		.update = gimbal_motor_update,
		.check = gimbal_motor_check,
		.heart_beat = gimbal_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_M_PITCH,
	},
	[GIMBAL_S_PITCH] = {
		.info = &gimbal_motor_info[GIMBAL_S_PITCH],
		.driver = &gimbal_motor_driver[GIMBAL_S_PITCH],
		.init = gimbal_motor_init,
		.update = gimbal_motor_update,
		.check = gimbal_motor_check,
		.heart_beat = gimbal_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_S_PITCH,
	},
};

/* Private functions ---------------------------------------------------------*/
static void gimbal_motor_check(gimbal_motor_t *motor)
{
	int16_t err;
	gimbal_motor_info_t *motor_info = motor->info;
	
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

static void gimbal_motor_heart_beat(gimbal_motor_t *motor)
{
	gimbal_motor_info_t *motor_info = motor->info;
	
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

