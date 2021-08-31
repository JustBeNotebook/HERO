#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "myiic.h" 


MPU6050_Module MPU6050_Gimbal;
MPU6050_Module MPU6050_MK1;

float err_y, err_p, err_r;
float now_y, now_p, now_r;
float last_y = 0, last_p = 0, last_r = 0;
u8 flags;
u8 MPU_Wait(void)
{
	u32 timeout = SysTimeStamp.msec_cnt;
	
	while(flags<50)
	{
		if(mpu_dmp_get_data(&now_p, &now_r, &now_y) != 0)
		{
			if(now_p<0.1f && now_p>-0.1f &&
					now_r<0.1f && now_r>-0.1f &&
					now_y<0.1f && now_y>-0.1f)
			{
			}
			flags++;
		}
		sys_delay_ms(5);
	}
	
	while(1)
	{
		//mpu_dmp_get_data(&now_p, &now_r, &now_y);		//∂¡»°≈∑¿≠
		if(mpu_dmp_get_data(&now_p, &now_r, &now_y) != 0)
			continue;
		
		err_y = now_y-last_y;
		err_r = now_r-last_r;
		err_p = now_p-last_p;
		
		if(((err_y<0.1f && err_y>-0.1f) && 
			 (err_p<0.1f && err_p>-0.1f) && 
			 (err_r<0.1f && err_r>-0.1f)) || 
			(SysTimeStamp.msec_cnt > (timeout+5000)))
		{
				break;
		}
		flags++;
		last_p = now_p;
		last_y = now_y;
		last_r = now_r;
		
		sys_delay_ms(500);
	}
	
	return 0;
}

void MPU_Module_Init(void)
{
	MPU6050_Gimbal.gyrox_zs = 0;
	MPU6050_Gimbal.gyroy_zs = 0;
	MPU6050_Gimbal.gyroz_zs = 0;
	
	MPU6050_Gimbal.gyrox = 0;
	MPU6050_Gimbal.gyroy = 0;
	MPU6050_Gimbal.gyroz = 0;
	
	MPU6050_Gimbal.state = 0;
	
}




