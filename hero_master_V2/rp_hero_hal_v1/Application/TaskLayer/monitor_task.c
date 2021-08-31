/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor_task.h"
#include "vision_potocol.h"
#include "vision_sensor.h"
#include "drv_io.h"
#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"
#include "predict.h"
#include "gimbal.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void device_heart_beat(void)
{
	rc_sensor.heart_beat(&rc_sensor);
	imu_sensor.heart_beat(&imu_sensor);
	chassis_motor[CHAS_LF].heart_beat(&chassis_motor[CHAS_LF]);
	chassis_motor[CHAS_RF].heart_beat(&chassis_motor[CHAS_RF]);
	chassis_motor[CHAS_LB].heart_beat(&chassis_motor[CHAS_LB]);
	chassis_motor[CHAS_RB].heart_beat(&chassis_motor[CHAS_RB]);
	
	gimbal_motor[GIMBAL_YAW].heart_beat(&gimbal_motor[GIMBAL_YAW]);
	gimbal_motor[GIMBAL_M_PITCH].heart_beat(&gimbal_motor[GIMBAL_M_PITCH]);
	gimbal_motor[GIMBAL_S_PITCH].heart_beat(&gimbal_motor[GIMBAL_S_PITCH]);
	
	launcher_motor[LAUNCHER_MGZ].heart_beat(&launcher_motor[LAUNCHER_MGZ]);
	launcher_motor[LAUNCHER_SAFE].heart_beat(&launcher_motor[LAUNCHER_SAFE]);
	launcher_motor[LAUNCHER_DIAL].heart_beat(&launcher_motor[LAUNCHER_DIAL]);
	launcher_motor[LAUNCHER_FRICT_L].heart_beat(&launcher_motor[LAUNCHER_FRICT_L]);
	launcher_motor[LAUNCHER_FRICT_R].heart_beat(&launcher_motor[LAUNCHER_FRICT_R]);
	
	Predict_Anti_Top_Data_Heart_beat();
	
	judge_offline_cnt++;
	if(judge_offline_cnt>5000)
	{
		judge_offline_cnt = 5000;
		Judge_Offline_flag = 1;
	}
	else
	{
		Judge_Offline_flag = 0;
	}

}

static void system_led_flash(void)
{
	static uint16_t led_blue_flash = 0;
	
	led_blue_flash++;
	if(led_blue_flash > 500) 
	{
		led_blue_flash = 0;
		LED_RED_TOGGLE();//11
	}
}

void soft_reset(void)
{
	if(IF_KEY_PRESSED_Z && IF_KEY_PRESSED_X && IF_KEY_PRESSED_V)
	{
		//__set_FAULTMASK(1);//关闭总中断
		NVIC_SystemReset();
	}
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	系统监控任务
 */
uint32_t time = 0;
void StartMonitorTask(void const * argument)
{
	//LED_RED_ON();
	for(;;)
	{
		system_led_flash();
		device_heart_beat();		//掌管心跳大权
		soft_reset();
		imu_sensor.update(&imu_sensor);
		gimbal_now_angle_ms_update();
		
		if(sys.co_mode == CO_GYRO)
		{
			LED_BLUE_ON();//13
			LED_ORANGE_OFF();//14
		}
		else if(sys.co_mode == CO_MECH)
		{
			LED_BLUE_OFF();
			LED_ORANGE_ON();//14
		}
		
		if(sys.remote_mode == KEY)
		{
			LED_GREEN_ON();//10
			LED_RED_OFF();//11
		}
		else if(sys.remote_mode == RC)
		{
			LED_GREEN_OFF();
			LED_RED_ON();
		}
		time = osKernelSysTick();
		if(time%50 == 0)
		{
			Anti_sentry_mode();
			Vision_Sent();
		}
		osDelay(1);
	}
}
