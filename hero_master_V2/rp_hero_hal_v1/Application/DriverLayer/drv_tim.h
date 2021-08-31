#ifndef __DRV_TIM_H
#define __DRV_TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
#define FRIC_PWM_L	TIM3->CCR1
#define FRIC_PWM_R	TIM3->CCR2
#define SERVO_PWM	TIM1->CCR2

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void PWM_Init(void);
void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2);

#endif
