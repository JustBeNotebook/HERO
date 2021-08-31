/**
 * @file        drv_tim.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        23-August-2020
 * @brief       TIMER Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2);
void COVER_PwmOut(int16_t pwm);

/* Exported functions --------------------------------------------------------*/
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	FRICTION_PwmOut(0, 0);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	COVER_PwmOut(0);
}

void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2)
{
	FRIC_PWM_L = pwm1;
	FRIC_PWM_R = pwm2;
}

void COVER_PwmOut(int16_t pwm)
{
	SERVO_PWM = pwm;
}

