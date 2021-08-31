/**
 * @file        rp_math.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       RP Algorithm.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int16_t buffer = 0;
	
	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}

uint32_t RampUInt32(uint32_t final, uint32_t now, uint32_t ramp)
{
	if(final<ramp)
	{
		final = ramp;
	}
	
	if(now<final-ramp)
	{
		now+=ramp;
	}
	else if(now>final+ramp)
	{
		now-=ramp;
	}
	else 
	{
		now = final;
	}
	return now;
}

float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;
	
	buffer = final - now;
	if(abs(now-final)>abs(ramp))
	{
		if(now< final)
		{
			buffer = now+ramp;
		}
		else
		{
			buffer = now-ramp;
		}
	}
	else
	{
		buffer = final;
	}

	return buffer;	
}

float DeathZoom(float input, float center, float death)
{
	if(abs(input - center) < death)
		return center;
	return input;
}


void reset_for_nan(float x)
{
	if(__ARM_isnanf(x))
	{
		__set_FAULTMASK(1);//¹Ø±Õ×ÜÖÐ¶Ï
		NVIC_SystemReset();
	}
}

