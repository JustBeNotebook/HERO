#include "IMUsensor.h"
#include "bmi.h"

imu_sensor_info_t imu_chassis;

void IMUsensor_Init(void)
{
	
	int8_t rslt;
	float xlimit, ylimit, zlimit;
	short gyrox, gyroy, gyroz;
	short accx, accy, accz;
	//uint32_t tickstart = HAL_GetTick();
	imu_sensor_info_t *imu_info = &imu_chassis;

	
  rslt = BMI_Init();
	while(rslt) {
        // ????????????????3????
        rslt = BMI_Init();
    }
    //imu_init_errno = rslt;
  
	rslt = 0;
		
	while(rslt)
	{
		for(uint16_t i=0; i<250; i++) {
			BMI_GET_DATA(&imu_info->rate_pitch, &imu_info->rate_roll, &imu_info->rate_yaw, &accx, &accy, &accz);
			imu_info->rate_pitch_offset += imu_info->rate_pitch;
			imu_info->rate_yaw_offset += imu_info->rate_yaw;
			imu_info->rate_roll_offset += imu_info->rate_roll;
			xlimit += abs(imu_info->rate_pitch);
			ylimit += abs(imu_info->rate_yaw);
			zlimit += abs(imu_info->rate_roll);
		}
		
		if(xlimit>6000 || ylimit>6000 || zlimit>6000)
		{
			xlimit = 0;
			ylimit = 0;
			zlimit = 0;
			imu_info->rate_pitch_offset = 0;
			imu_info->rate_yaw_offset = 0;
			imu_info->rate_roll_offset = 0;
		}
		else
		{
			rslt = 0;
		}
    /**
        @note
        
    */
	}
	imu_info->rate_pitch_offset /= 250.f;
	imu_info->rate_yaw_offset /= 250.f;
	imu_info->rate_roll_offset /= 250.f;
}

void IMUsensor_GetData(void)
{
	short gyrox, gyroy, gyroz;
	short accx, accy, accz;
	
	BMI_GET_DATA(&imu_chassis.rate_pitch, &imu_chassis.rate_roll, &imu_chassis.rate_yaw, &accx, &accy, &accz);
	BMI_Get_data(&imu_chassis.pitch, &imu_chassis.roll, &imu_chassis.yaw, &imu_chassis.rate_pitch, &imu_chassis.rate_roll, &imu_chassis.rate_yaw, &accx, &accy, &accz);
	
}
