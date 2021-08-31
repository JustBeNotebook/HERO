/**
 * @file        drv_adda.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       ADC/DAC Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_adda.h"

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void ADC_Init(void)
{
}

void DAC_Init(void)
{
    HAL_DAC_Start(&hdac, CAP_CUR_OUT_CHNL);
}

uint16_t ADC_GetValue(ADC_HandleTypeDef *hadc)
{
    uint16_t raw;
    
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    raw = HAL_ADC_GetValue(hadc);
    
    return raw;
}
