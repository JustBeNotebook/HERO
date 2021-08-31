#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "rc_sensor.h"
#include "imu_sensor.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "launcher_motor.h"
#include "judge.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct {
	rc_sensor_t 		*rc_sen;
	imu_sensor_t		*imu_sen;
	chassis_motor_t		*chas_mtr[CHAS_MOTOR_CNT];
	gimbal_motor_t		*gimbal_mtr[GIMBAL_MOTOR_CNT];
	launcher_motor_t		*launcher_mtr[LAUNCHER_MOTOR_CNT];
	
} dev_list_t;

extern dev_list_t dev_list;

/* Exported functions --------------------------------------------------------*/
void DEV_Init(void);

#endif
