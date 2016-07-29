#include "timer_opts.h"
#include "nvic_opts.h"


u8 fac_us;
u16 fac_ms;


void Delay_Init(u8 sysclk)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	// ʱ��Դ����ΪHCLK��8��Ƶ
	//SysTick->CTRL &= 0xFFFFFFFB;
	SysTick->CTRL &= ~(0x01 << 2);
	
	fac_us = sysclk / 8;
	fac_ms = (u16)fac_us * 1000;
}


u8 Delay_us(u32 nus)
{
	//u32 temp;
	
	SysTick->LOAD = nus * fac_us;
	SysTick->VAL = 0x00;		//��������0,��Ϊcurrrent�ֶα��ֶ�����ʱ,load���Զ���װ��VAL��
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

void Delay_ns(u32 ns)
{
	u32 cir = 0;
	
	for(cir = 0; cir < ns; cir++)
	{
		Delay_ms(1000);
	}
}


void Timer2_Init(u16 arr, u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);								//��ʼ��

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIMx_2_7_ENABLE(2);					// TIM2ʱ��ʹ��
	TIM2->ARR = arr;					// �Զ�װ��ֵ
	TIM2->PSC = psc;					// ��Ƶϵ��
	TIM2->DIER |= (1 << 0);				// ��������ж�
	TIM2->DIER |= (1 << 6);				// �������ж�	
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ��
#endif
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIMx_2_7_ENABLE(4);					// TIM4ʱ��ʹ��
	TIM4->ARR = arr;					// �Զ�װ��ֵ
	TIM4->PSC = psc;					// ��Ƶϵ��
	TIM4->DIER |= (1 << 0);				// ��������ж�
	TIM4->DIER |= (1 << 6);				// �������ж�
	TIMx_ON(TIM4);
	
}


void Timer5_Init(u16 arr, u16 psc)
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ��
#endif
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIMx_2_7_ENABLE(5);					// TIM4ʱ��ʹ��
	TIM5->ARR = arr;					// �Զ�װ��ֵ
	TIM5->PSC = psc;					// ��Ƶϵ��
	TIM5->DIER |= (1 << 0);				// ��������ж�
	TIM5->DIER |= (1 << 6);				// �������ж�
	TIMx_ON(TIM5);
	
}

void Timer6_Init(u16 arr, u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 			//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ��
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIMx_2_7_ENABLE(6);					// TIM4ʱ��ʹ��
	TIM6->ARR = arr;					// �Զ�װ��ֵ
	TIM6->PSC = psc;					// ��Ƶϵ��
	TIM6->DIER |= (1 << 0);				// ��������ж�
	TIM6->DIER |= (1 << 6);				// �������ж�
	TIMx_ON(TIM6);
	
}

/*
void Timer6_Init2(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʹ������ʱ��
	
	TIM_TimeBaseStructure.TIM_Period = arr;							//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;						//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//TIM���¼���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);					//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);								//ʹ��ARRԤװ�أ���ֹ���ϼ���ʱ�����¼��쳣�ӳ�
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );	//��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;					//TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;				//�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);									//��ʼ��NVIC�Ĵ���

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);					//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
}
*/

/*
	��������: 
	DutyCycle: ռ�ձ�,������ֵ��Χ0~100
	Frequency: PWMƵ��,������ֵ��Χ1Hz~36MHz
*/
void PWM_CFG(TIM_TypeDef *TIMx, u16 DutyCycle, u32 Frequency)
{
	
}

void TIM3_Init(u16 Arr, u16 Psc)
{
	/* ���� TIM_TimeBase ��ʼ���ṹ�� TIM_TimeBaseStructure */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* ���� TIM_OCInit ��ʼ���ṹ�� TIM_OCInitStructure */
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIMx_2_7_ENABLE(3); 
	/* 
	*	��������ֵΪ9999
	*	Ԥ��ƵֵΪ(0 + 1 = 1)
	*	ʱ�ӷָ�0
	*	���ϼ���ģʽ
	*/
	TIM_TimeBaseStructure.TIM_Period = 36000;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/*	���� OC1,OC2,OC3,OC4 ͨ��
	*	����ģʽΪ PWM ���ģʽ
	*	ʹ�ܱȽ�ƥ���������
	*	ʱ�ӷָ�0
	*	���ϼ���ģʽ
	*
	*	���ø�ƥ��ֵ�ֱ�Ϊ CCR1_Val, CCR1_Val, CCR1_Val, CCR1_Val
	*	�õ���ռ�ձȷֱ�Ϊ 50%, 37.5%, 25%, 12.5%  
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

	/* ʹ��Ԥװ�ؼĴ��� */
	//TIM_OC1PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	//TIM_OC2PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	/* ���� TIM ���� */
	TIM_Cmd(TIM3 , ENABLE); 
	

}




