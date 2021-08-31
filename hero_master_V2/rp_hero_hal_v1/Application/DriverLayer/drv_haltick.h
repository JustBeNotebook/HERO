#ifndef __DRV_HALTICK_H
#define __DRV_HALTICK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint32_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif
