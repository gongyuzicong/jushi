#ifndef __TIMER_OPTS_H__
#define __TIMER_OPTS_H__


#include "data_type.h"

#define TIMx_ON(TIMx)			(TIMx->CR1 |= 0x01)
#define TIMx_OFF(TIMx)			(TIMx->CR1 = ~((~TIMx->CR1) | 0x01))

#define TIMx_2_7_ENABLE(x)		(RCC->APB1ENR |= (1 << (x - 2)))


void Timer2_Init(u16 arr, u16 psc);
void Delay_Init(u8 sysclk);
u8 Delay_us(u32 nus);
u8 Delay_ms(u32 nms);
void Timer4_Init(u16 arr, u16 psc);
void TIM3_Init(u16 Arr, u16 Psc);


//extern u8 fac_us;
//extern u16 fac_ms;



#endif




