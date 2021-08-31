/**
 * @file        device.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       Devices' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"

#include "drv_haltick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
	.rc_sen = &rc_sensor,
	.imu_sen = &imu_sensor,
	.chas_mtr[CHAS_LF] = &chassis_motor[CHAS_LF],
	.chas_mtr[CHAS_RF] = &chassis_motor[CHAS_RF],
	.chas_mtr[CHAS_LB] = &chassis_motor[CHAS_LB],
	.chas_mtr[CHAS_RB] = &chassis_motor[CHAS_RB],
	.gimbal_mtr[GIMBAL_YAW] = &gimbal_motor[GIMBAL_YAW],
	.gimbal_mtr[GIMBAL_M_PITCH] = &gimbal_motor[GIMBAL_M_PITCH],
	.gimbal_mtr[GIMBAL_S_PITCH] = &gimbal_motor[GIMBAL_S_PITCH],
	.launcher_mtr[LAUNCHER_MGZ] = &launcher_motor[LAUNCHER_MGZ],
	.launcher_mtr[LAUNCHER_SAFE] = &launcher_motor[LAUNCHER_SAFE],
	.launcher_mtr[LAUNCHER_DIAL] = &launcher_motor[LAUNCHER_DIAL],
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
//也许可以用宏定义写成 DEV_INIT((A)) (A)->init((A))
void DEV_Init(void)
{
	dev_list.rc_sen->init(dev_list.rc_sen);
	
	dev_list.chas_mtr[CHAS_LF]->init(dev_list.chas_mtr[CHAS_LF]);
	dev_list.chas_mtr[CHAS_RF]->init(dev_list.chas_mtr[CHAS_RF]);
	dev_list.chas_mtr[CHAS_LB]->init(dev_list.chas_mtr[CHAS_LB]);
	dev_list.chas_mtr[CHAS_RB]->init(dev_list.chas_mtr[CHAS_RB]);
	dev_list.gimbal_mtr[GIMBAL_YAW]->init(dev_list.gimbal_mtr[GIMBAL_YAW]);
	dev_list.gimbal_mtr[GIMBAL_M_PITCH]->init(dev_list.gimbal_mtr[GIMBAL_M_PITCH]);
	dev_list.gimbal_mtr[GIMBAL_S_PITCH]->init(dev_list.gimbal_mtr[GIMBAL_S_PITCH]);
	dev_list.launcher_mtr[LAUNCHER_MGZ]->init(dev_list.launcher_mtr[LAUNCHER_MGZ]);
	dev_list.launcher_mtr[LAUNCHER_SAFE]->init(dev_list.launcher_mtr[LAUNCHER_SAFE]);
	dev_list.launcher_mtr[LAUNCHER_DIAL]->init(dev_list.launcher_mtr[LAUNCHER_DIAL]);
}
