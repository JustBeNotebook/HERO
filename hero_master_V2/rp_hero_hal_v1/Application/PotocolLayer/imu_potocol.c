/**
 * @file        imu_potocol.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       Imu Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"
#include "imu_potocol.h"
#include "kalman.h"
#include "bmi.h"

#include "imu_sensor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extKalman_t rate_yaw_kf, rate_pitch_kf, rate_roll_kf;
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
short gyrox, gyroy, gyroz;
short accx, accy, accz;
void imu_sensor_update(imu_sensor_t *imu_sen)
{
	imu_sensor_info_t *imu_info = imu_sen->info;

	BMI_Get_RawData(&gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
	
	//gyrox -= imu_info->rate_pitch_offset;
	//gyroy -= imu_info->rate_yaw_offset;
	//gyroz -= imu_info->rate_roll_offset;
	
	imu_info->rate_pitch = KalmanFilter(&rate_roll_kf,gyrox);
	imu_info->rate_roll = KalmanFilter(&rate_pitch_kf, gyroy);
	imu_info->rate_yaw = KalmanFilter(&rate_yaw_kf, gyroz);
	
	BMI_Get_EulerAngle(&imu_info->pitch, &imu_info->roll, &imu_info->yaw, &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
	
	imu_sen->check(imu_sen);

}

//int8_t imu_init_errno;
//void imu_sensor_init(imu_sensor_t *imu_sen)
//{
//  int8_t rslt;
//	uint8_t fail_times = 0;
//	float xlimit, ylimit, zlimit;
//	uint32_t tickstart = HAL_GetTick();
//	imu_sensor_info_t *imu_info = imu_sen->info;
//	
//	imu_sen->errno = NONE_ERR;

//    rslt = BMI_Init();
//	while(rslt) 
//	{
//        // ????????????????3????
//    imu_sen->errno = DEV_INIT_ERR;
//    rslt = BMI_Init();
//		if(fail_times > 15)
//		{
//			NVIC_SystemReset();
//		}
//		fail_times++;
//  }
//    //imu_init_errno = rslt;

//	fail_times = 0;
//  
//	rslt = 1;
//		
//	while(rslt)
//	{
//		for(uint16_t i=0; i<250; i++) {
//			BMI_Get_RawData(&imu_info->rate_pitch, &imu_info->rate_roll, &imu_info->rate_yaw, &accx, &accy, &accz);
//			imu_info->rate_pitch_offset += imu_info->rate_pitch;
//			imu_info->rate_yaw_offset += imu_info->rate_yaw;
//			imu_info->rate_roll_offset += imu_info->rate_roll;
//			xlimit += abs(imu_info->rate_pitch);
//			ylimit += abs(imu_info->rate_yaw);
//			zlimit += abs(imu_info->rate_roll);
//		}
//		
//		if(xlimit>6000 || ylimit>6000 || zlimit>6000)
//		{
//			xlimit = 0;
//			ylimit = 0;
//			zlimit = 0;
//			imu_info->rate_pitch_offset = 0;
//			imu_info->rate_yaw_offset = 0;
//			imu_info->rate_roll_offset = 0;
//		}
//		else
//		{
//			rslt = 0;
//		}
//    /**
//        @note
//        
//    */
//	}
//	imu_info->rate_pitch_offset /= 250.f;
//	imu_info->rate_yaw_offset /= 250.f;
//	imu_info->rate_roll_offset /= 250.f;
//	

//	KalmanCreate(&rate_yaw_kf, 1, 1);
//	KalmanCreate(&rate_pitch_kf, 1, 1);
//	
//	
//	if(imu_sen->id != DEV_ID_IMU)
//		imu_sen->errno = DEV_ID_ERR;
//}

//int8_t imu_init_errno;
 uint32_t rslt_times;
void imu_sensor_init(imu_sensor_t *imu_sen)
{
  int8_t rslt;
	float xlimit, ylimit, zlimit;
	uint32_t tickstart = HAL_GetTick();
	imu_sensor_info_t *imu_info = imu_sen->info;
	
	imu_sen->errno = NONE_ERR;

  rslt = BMI_Init();
	while(rslt) {
        // ????????????????3????
        imu_sen->errno = DEV_INIT_ERR;
        rslt = BMI_Init();
    }
    //imu_init_errno = rslt;
  
	rslt_times = 1;
		
	while(rslt_times)
	{
		for(uint16_t i=0; i<250; i++) {
			BMI_Get_RawData(&imu_info->rate_pitch, &imu_info->rate_roll, &imu_info->rate_yaw, &accx, &accy, &accz);
			imu_info->rate_pitch_offset += imu_info->rate_pitch;
			imu_info->rate_yaw_offset += imu_info->rate_yaw;
			imu_info->rate_roll_offset += imu_info->rate_roll;
			xlimit += abs(imu_info->rate_pitch);
			ylimit += abs(imu_info->rate_yaw);
			zlimit += abs(imu_info->rate_roll);
		}
		
		if(xlimit>8000 || ylimit>8000 || zlimit>8000)
		{
			xlimit = 0;
			ylimit = 0;
			zlimit = 0;
			imu_info->rate_pitch_offset = 0;
			imu_info->rate_yaw_offset = 0;
			imu_info->rate_roll_offset = 0;
			rslt_times++;
		}
		else
		{
			rslt_times = 0;
		}
		
		if(rslt_times>10)
		{
			//NVIC_SystemReset();
			break;
		}
    /**
        @note
        
    */
	}
	imu_info->rate_pitch_offset /= 250.f;
	imu_info->rate_yaw_offset /= 250.f;
	imu_info->rate_roll_offset /= 250.f;

	KalmanCreate(&rate_roll_kf, 1, 10);
	KalmanCreate(&rate_yaw_kf, 1, 10);
	KalmanCreate(&rate_pitch_kf, 1, 10);
	
	if(imu_sen->id != DEV_ID_IMU)
		imu_sen->errno = DEV_ID_ERR;
}


