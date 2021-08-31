#ifndef RP__IMUSENSOR__H
#define RP__IMUSENSOR__H

#include "link.h"

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

extern imu_sensor_info_t imu_chassis;

void IMUsensor_Init(void);
void IMUsensor_GetData(void);

#endif
