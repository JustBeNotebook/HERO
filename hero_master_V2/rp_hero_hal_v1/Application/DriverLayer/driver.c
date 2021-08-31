/**
 * @file        driver.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       Drivers' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"
#include "predict.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRIVER_Init(void)
{
	PWM_Init();
    ADC_Init();
    DAC_Init();
	USART1_Init();
	USART2_Init();
	USART4_Init();
	USART5_Init();
	CAN1_Init();
	CAN2_Init();	
	PREDICT_Init();
}
