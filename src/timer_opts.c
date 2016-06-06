#include "timer_opts.h"
#include "nvic_opts.h"


u8 fac_us;
u16 fac_ms;


void Delay_Init(u8 sysclk)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	// 时钟源设置为HCLK的8分频
	//SysTick->CTRL &= 0xFFFFFFFB;
	SysTick->CTRL &= ~(0x01 << 2);
	
	fac_us = sysclk / 8;
	fac_ms = (u16)fac_us * 1000;
}


u8 Delay_us(u32 nus)
{
	//u32 temp;
	
	SysTick->LOAD = nus * fac_us;
	SysTick->VAL = 0x00;		//计数器清0,因为currrent字段被手动清零时,load将自动重装到VAL中
	SysTick->CTRL |= 0x01;
	
#if 0
	do
	{
		temp = SysTick->CTRL;
	}while(temp & 0x01 && !(temp & (1 << 16)));
#else
	while(!(SysTick->CTRL & (1 << 16)));
#endif	

	SysTick->CTRL &= ~(0x01);
	SysTick->VAL = 0x00;

	return 0;
}

u8 Delay_ms(u32 nms)
{
	//u32 temp;

	SysTick->LOAD = (u32)nms * fac_ms;
	SysTick->VAL = 0x00;
	SysTick->CTRL |= 0x01;

#if 0
	do
	{
		temp = SysTick->CTRL;
	}while(temp & 0x01 && !(temp & (1 << 16)));
#else
	while(!(SysTick->CTRL & (1 << 16)));
#endif

	SysTick->CTRL &= ~(0x01);
	SysTick->VAL = 0x00;
	
	return 0;
}


void Timer2_Init(u16 arr, u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;		//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);								//初始化

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIMx_2_7_ENABLE(2);					// TIM2时钟使能
	TIM2->ARR = arr;					// 自动装载值
	TIM2->PSC = psc;					// 分频系数
	TIM2->DIER |= (1 << 0);				// 允许更新中断
	TIM2->DIER |= (1 << 6);				// 允许触发中断	
	TIMx_ON(TIM2);

}

void Timer4_Init(u16 arr, u16 psc)
{
#if 0
	Nvic_Paramater node;
	node.Nvic_PreemptionPriority = 1;
	node.Nvic_SubPriority = 3;
	node.Nvic_Channel = TIM4_IRQChannel;
	node.Nvic_Group = 2;
	My_Nvic_Init(node);
#else
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;		//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);//初始化
#endif
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIMx_2_7_ENABLE(4);					// TIM4时钟使能
	TIM4->ARR = arr;					// 自动装载值
	TIM4->PSC = psc;					// 分频系数
	TIM4->DIER |= (1 << 0);				// 允许更新中断
	TIM4->DIER |= (1 << 6);				// 允许触发中断
	TIMx_ON(TIM4);
	
}

/*
	函数参数: 
	DutyCycle: 占空比,参数数值范围0~100
	Frequency: PWM频率,参数数值范围1Hz~36MHz
*/
void PWM_CFG(TIM_TypeDef *TIMx, u16 DutyCycle, u32 Frequency)
{
	
}

void TIM3_Init(u16 Arr, u16 Psc)
{
	/* 定义 TIM_TimeBase 初始化结构体 TIM_TimeBaseStructure */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* 定义 TIM_OCInit 初始化结构体 TIM_OCInitStructure */
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIMx_2_7_ENABLE(3); 
	/* 
	*	计数重载值为9999
	*	预分频值为(0 + 1 = 1)
	*	时钟分割0
	*	向上计数模式
	*/
	TIM_TimeBaseStructure.TIM_Period = 36000;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/*	设置 OC1,OC2,OC3,OC4 通道
	*	工作模式为 PWM 输出模式
	*	使能比较匹配输出极性
	*	时钟分割0
	*	向上计数模式
	*
	*	设置各匹配值分别为 CCR1_Val, CCR1_Val, CCR1_Val, CCR1_Val
	*	得到的占空比分别为 50%, 37.5%, 25%, 12.5%  
	*/

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCInitStructure.TIM_Pulse = 18000;	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = 30000;	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = 15000;	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = 7500;	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	/* 使能预装载寄存器 */
	//TIM_OC1PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	//TIM_OC2PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	/* 启动 TIM 计数 */
	TIM_Cmd(TIM3 , ENABLE); 
	

}




