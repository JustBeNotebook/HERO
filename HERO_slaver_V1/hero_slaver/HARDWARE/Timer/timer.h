#ifndef RM__PWM__H
#define RM__PWM__H


#include "sys.h"



void TIM3_PWM_Init(u16 pres, u32 reload);

void TIM4_PWM_Init(u16 pres, u32 reload);
void TIM3_Int_Init(u16 arr,u16 psc);

#endif


