#ifndef __RC_POTOCOL_H
#define __RC_POTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void USART1_rxDataHandler(uint8_t *rxBuf);
void Anti_sentry_mode(void);
	
#endif
