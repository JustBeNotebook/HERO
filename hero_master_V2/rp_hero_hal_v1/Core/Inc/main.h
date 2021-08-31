/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLUE_Pin GPIO_PIN_13
#define LED_BLUE_GPIO_Port GPIOC
#define LED_ORANGE_Pin GPIO_PIN_14
#define LED_ORANGE_GPIO_Port GPIOC
#define CAP_VOL_IN_Pin GPIO_PIN_0
#define CAP_VOL_IN_GPIO_Port GPIOC
#define S_CHARGE_Pin GPIO_PIN_3
#define S_CHARGE_GPIO_Port GPIOC
#define CAP_OUT_Pin GPIO_PIN_2
#define CAP_OUT_GPIO_Port GPIOA
#define CAP_CUR_OUT_Pin GPIO_PIN_4
#define CAP_CUR_OUT_GPIO_Port GPIOA
#define CAP_IN_Pin GPIO_PIN_5
#define CAP_IN_GPIO_Port GPIOA
#define REBOOT_RELAY_SET_Pin GPIO_PIN_0
#define REBOOT_RELAY_SET_GPIO_Port GPIOB
#define REBOOT_READ_Pin GPIO_PIN_1
#define REBOOT_READ_GPIO_Port GPIOB
#define PHOTOGATE_Pin GPIO_PIN_11
#define PHOTOGATE_GPIO_Port GPIOE
#define IIC_SCL_Pin GPIO_PIN_13
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_15
#define IIC_SDA_GPIO_Port GPIOB
#define LASER_Pin GPIO_PIN_9
#define LASER_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_10
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
