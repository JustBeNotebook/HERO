/**
 * @file        chassis_task.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        4-November-2020
 * @brief       Chassis Task.
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis_task.h"

#include "chassis.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	底盘任务
 */
void StartChassisTask(void const * argument)
{
	chassis.init();
	for(;;)
	{
		if(chassis.test_open)//测试模式
		{
			chassis.test();
		}
		else//常规模式
		{
			if(sys.state == SYS_STATE_NORMAL)
			{
				chassis.ctrl();
			}
			else
			{
				chassis.self_protect();
			}
		}
		
		osDelay(1);
	}
}
