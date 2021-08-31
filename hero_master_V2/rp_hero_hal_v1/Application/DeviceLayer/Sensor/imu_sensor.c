/**
 * @file        imu_sensor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       IMU' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"

#include "imu_potocol.h"

extern void imu_sensor_update(imu_sensor_t *imu_sen);
extern void imu_sensor_init(imu_sensor_t *imu_sen);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void imu_sensor_check(imu_sensor_t *imu_sen);
static void imu_sensor_heart_beat(imu_sensor_t *imu_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
drv_iic_t 	imu_sensor_driver = {
	.type = DRV_IIC,
};

imu_sensor_info_t 	imu_sensor_info = {
	.offline_max_cnt = 50,
};

imu_sensor_t 	imu_sensor = {
	.info = &imu_sensor_info,
	.driver = &imu_sensor_driver,
	.init = imu_sensor_init,
	.update = imu_sensor_update,
	.check = imu_sensor_check,
	.heart_beat = imu_sensor_heart_beat,
	.work_state = DEV_ONLINE,
	.id = DEV_ID_IMU,	
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
static void imu_sensor_check(imu_sensor_t *imu_sen)
{
	imu_sensor_info_t *imu_info = imu_sen->info;

	
	imu_info->rate_yaw -= imu_info->rate_yaw_offset;
	imu_info->rate_pitch -= imu_info->rate_pitch_offset;
	imu_info->rate_roll -= imu_info->rate_roll_offset;
	
	imu_info->offline_cnt = 0;
}

static void imu_sensor_heart_beat(imu_sensor_t *imu_sen)
{
	imu_sen->work_state = DEV_ONLINE;
}
