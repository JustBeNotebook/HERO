/**
 * @file        control_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Control Center
 */

/* Includes ------------------------------------------------------------------*/
#include "control_task.h"

#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	控制任务
 */
void StartControlTask(void const * argument)
{
	//module.init();
	control.self_protect();
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL)//正常不干扰
		{
			control.output();
		}
		else if(sys.state == SYS_STATE_RCLOST)
		{
			if(flag.gimbal.reset_start == true && flag.gimbal.reset_ok == false)
			{
				control.reset();
			}
			else if(flag.gimbal.reset_start == false && flag.gimbal.reset_ok == false)
			{
				control.self_protect();
			}
			else if(flag.gimbal.reset_start == false && flag.gimbal.reset_ok == true)
			{
				control.reset();
			}
		}
		else	//不正常全部清零 
		{
			//module.self_protect();
			control.self_protect();
		}
		
		
		osDelay(1);
	}
}
