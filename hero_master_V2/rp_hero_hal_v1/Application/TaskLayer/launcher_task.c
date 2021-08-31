/**
 * @file        launcher_task.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        4-November-2020
 * @brief       Launcher Task.
 */

/* Includes ------------------------------------------------------------------*/
#include "Launcher_task.h"

//#include "turnplate.h"
//#include "fricwheel.h"
//#include "cover.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	Éä»÷ÈÎÎñ
 */
void StartLauncherTask(void const * argument)
{
	launcher.init();
	
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL)
		{
			launcher.ctrl();
		}
		else
		{
			launcher.self_protect();
		}
		
		osDelay(1); 	 
	}
}
