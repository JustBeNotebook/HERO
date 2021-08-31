#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "stm32f4xx_hal.h"
#include "math.h"
#include "pid.h"

/* 数值函数 */
#ifndef PI
	#define PI 3.1415926f
#endif

#define constrain(x, min, max)	(((x)>(max))?(max):((x)<(min)?(min):(x)))
#define abs(x) 					((x)>0? (x):(-(x)))
#define RP_MAX(x, y) 					((x)>(y)?(x):(y))
#define RP_MIN(x, y) 					((x)>(y)?(y):(x))

int16_t RampInt(int16_t final, int16_t now, int16_t ramp);
uint32_t RampUInt32(uint32_t final, uint32_t now, uint32_t ramp);
float RampFloat(float final, float now, float ramp);
float DeathZoom(float input, float center, float death);
void reset_for_nan(float x);

#endif

