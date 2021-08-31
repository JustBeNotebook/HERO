/**
 * @file        gimbal_task.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        4-November-2020
 * @brief       Gimbal Task.
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"

#include "gimbal.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	云台任务
 */
extern osThreadId ControlTaskHandle;
int gimbal_StackHighWater = 0;
void StartGimbalTask(void const * argument)
{
	gimbal.init();
	for(;;)
	{
		//
		
		if(gimbal.test_open)//测试模式
		{
			gimbal.test();
		}
		else//常规模式
		{
			if(sys.state  == SYS_STATE_NORMAL)
			{
				gimbal.ctrl();
			}
			else if(sys.state == SYS_STATE_RCLOST)
			{
				VISION_PREDICT();
				if(flag.gimbal.reset_start == true && flag.gimbal.reset_ok == false)
				{
					gimbal.reset();
				}
				else if(flag.gimbal.reset_start == false && flag.gimbal.reset_ok == false)
				{
					gimbal.self_protect();
				}
				else if(flag.gimbal.reset_start == false && flag.gimbal.reset_ok == true)
				{
					gimbal.ctrl();
				}
			}
			else
			{
				gimbal.self_protect();
			}
		}
		gimbal_StackHighWater = uxTaskGetStackHighWaterMark(ControlTaskHandle);
		osDelay(1);
		
	}
}
