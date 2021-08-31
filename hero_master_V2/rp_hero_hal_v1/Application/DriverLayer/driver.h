#ifndef __DRIVER_H
#define __DRIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"

#include "drv_haltick.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "drv_io.h"
#include "drv_adda.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRIVER_Init(void);

#endif
