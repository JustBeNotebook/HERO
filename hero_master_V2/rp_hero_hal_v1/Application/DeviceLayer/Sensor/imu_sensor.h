#ifndef __IMU_SENSOR_H
#define __IMU_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
#define IMU_ID		0

/* Exported types ------------------------------------------------------------*/
typedef struct imu_sensor_info_struct {
	float yaw;
	float pitch;
	float roll;
	short rate_yaw;
	short rate_pitch;
	short rate_roll;
	float rate_yaw_offset;
	float rate_pitch_offset;
	float rate_roll_offset;
	
	uint8_t offline_cnt;
	uint8_t offline_max_cnt;
} imu_sensor_info_t;

typedef struct imu_sensor_struct {
	imu_sensor_info_t	*info;
	drv_iic_t			*driver;
	void				(*init)(struct imu_sensor_struct *self);
	void				(*update)(struct imu_sensor_struct *self);
	void				(*check)(struct imu_sensor_struct *self);	
	void				(*heart_beat)(struct imu_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;	
} imu_sensor_t;

extern imu_sensor_t imu_sensor;

/* Exported functions --------------------------------------------------------*/

#endif
