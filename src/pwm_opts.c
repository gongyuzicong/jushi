#include "pwm_opts.h"
#include "timer_opts.h"


PwmOperaterStruct pwm3_opts;
PwmOperaterStruct_P pwmOptsPtr_1 = &pwm3_opts;
PwmParaStruct pwm3Para;
PwmParaStruct_P pwmParaPtr_1 = &pwm3Para;

u8 Pwm_Frequency_Set(PwmParaStruct_P pwmArguPtr)	// PWM 频率设置 范围 1k~10k
{
	u8 flag = 1;
	
	if((pwmArguPtr->Frequency >= 1) && (pwmArguPtr->Frequency <= 10))
	{
		/* 定义 TIM_TimeBase 初始化结构体 TIM_TimeBaseStructure */
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

		if(TIM2 == pwmArguPtr->TimX)
		{
			TIMx_2_7_ENABLE(2);
		}
		else if(TIM3 == pwmArguPtr->TimX)
		{
			TIMx_2_7_ENABLE(3);
		}
		else if(TIM4 == pwmArguPtr->TimX)
		{
			TIMx_2_7_ENABLE(4);
		}
		else if(TIM5 == pwmArguPtr->TimX)
		{
			TIMx_2_7_ENABLE(5);
		}
		else if(TIM6 == pwmArguPtr->TimX)
		{
			TIMx_2_7_ENABLE(6);
		}
		else if(TIM7 == pwmArguPtr->TimX)
		{
			TIMx_2_7_ENABLE(7);
		}
		else
		{
			printf("pwm timer error!\r\n");
		}
				

		if(pwmArguPtr->Frequency <= 1)
		{
			pwmArguPtr->PwmFrequencyCounterNum = 36000;
			TIM_TimeBaseStructure.TIM_Prescaler = 1;
		}
		else
		{
			pwmArguPtr->PwmFrequencyCounterNum = 72000 / pwmArguPtr->Frequency;
			TIM_TimeBaseStructure.TIM_Prescaler = 0;
		}
		
		/* 
		*  	计数重载值为9999
		*  	预分频值为(0 + 1 = 1)
		*  	时钟分割0
		*  	向上计数模式
		*/
		
		
		pwmArguPtr->ResolutionPreStep = pwmArguPtr->PwmFrequencyCounterNum / pwmArguPtr->PwmResolution;
		
		TIM_TimeBaseStructure.TIM_Period = pwmArguPtr->PwmFrequencyCounterNum;
		//TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
		TIM_TimeBaseInit(pwmArguPtr->TimX, &TIM_TimeBaseStructure);
		
		flag = 0;
	}
	
	return flag;
}

// outC: PWM输出端口, dutyCycle: 占空比设置: 目前分辨率pwmResolution为100, 所以范围为0~100
u8 Pwm_Duty_Cycle_Init(u8 outC, PwmParaStruct_P pwmArguPtr)
{
	u8 flag = 0;
	
	/* 定义 TIM_OCInit 初始化结构体 TIM_OCInitStructure */
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	/* 	设置 OC1,OC2,OC3,OC4 通道
	*  	工作模式为 PWM 输出模式
	*  	使能比较匹配输出极性
	*  	时钟分割0
	*  	向上计数模式
	*
	*	设置各匹配值分别为 CCR1_Val, CCR1_Val, CCR1_Val, CCR1_Val
	*	得到的占空比分别为 50%, 37.5%, 25%, 12.5%  
	*/
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	switch(outC)
	{
		case 1:
			TIM_OCInitStructure.TIM_Pulse = pwmArguPtr->ResolutionPreStep * pwmArguPtr->Oc1DutyCycle;
			TIM_OC1Init(pwmArguPtr->TimX, &TIM_OCInitStructure);
			/* 使能预装载寄存器 */
			TIM_OC1PreloadConfig(pwmArguPtr->TimX, TIM_OCPreload_Enable);
			break;

		case 2:
			TIM_OCInitStructure.TIM_Pulse = pwmArguPtr->ResolutionPreStep * pwmArguPtr->Oc2DutyCycle;
			TIM_OC2Init(pwmArguPtr->TimX, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(pwmArguPtr->TimX, TIM_OCPreload_Enable);
			break;

		case 3:
			TIM_OCInitStructure.TIM_Pulse = pwmArguPtr->ResolutionPreStep * pwmArguPtr->Oc3DutyCycle;
			TIM_OC3Init(pwmArguPtr->TimX, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(pwmArguPtr->TimX, TIM_OCPreload_Enable);
			break;

		case 4:
			TIM_OCInitStructure.TIM_Pulse = pwmArguPtr->ResolutionPreStep * pwmArguPtr->Oc4DutyCycle;
			TIM_OC4Init(pwmArguPtr->TimX, &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(pwmArguPtr->TimX, TIM_OCPreload_Enable);
			break;

		default:
			flag = 1;
			break;
	}

	return flag;
}

void Pwm_Duty_Cycle_OC1_Set(PwmParaStruct_P pwmArguPtr, u8 dutyCycle)
{
	pwmArguPtr->TimX->CCR1 = pwmArguPtr->ResolutionPreStep * dutyCycle;
}

void Pwm_Duty_Cycle_OC2_Set(PwmParaStruct_P pwmArguPtr, u8 dutyCycle)
{
	pwmArguPtr->TimX->CCR2 = pwmArguPtr->ResolutionPreStep * dutyCycle;
}

void Pwm_Duty_Cycle_OC3_Set(PwmParaStruct_P pwmArguPtr, u8 dutyCycle)
{
	pwmArguPtr->TimX->CCR3 = pwmArguPtr->ResolutionPreStep * dutyCycle;
}

void Pwm_Duty_Cycle_OC4_Set(PwmParaStruct_P pwmArguPtr, u8 dutyCycle)
{
	pwmArguPtr->TimX->CCR4 = pwmArguPtr->ResolutionPreStep * dutyCycle;
}


void Pwm_Duty_Cycle_OC1_Add(PwmParaStruct_P pwmArguPtr, u8 addStep)
{	
	pwmArguPtr->Oc1DutyCycle += addStep;
	
	if(pwmArguPtr->Oc1DutyCycle > 100)
	{
		pwmArguPtr->Oc1DutyCycle = 100;
	}
	printf("DutyCycle = %d\r\n", pwmArguPtr->Oc1DutyCycle);
	Pwm_Duty_Cycle_OC1_Set(pwmArguPtr, pwmArguPtr->Oc1DutyCycle);
	
}

void Pwm_Duty_Cycle_OC2_Add(PwmParaStruct_P pwmArguPtr, u8 addStep)
{	
	pwmArguPtr->Oc2DutyCycle += addStep;
	
	if(pwmArguPtr->Oc2DutyCycle > 100)
	{
		pwmArguPtr->Oc2DutyCycle = 100;
	}
	
	Pwm_Duty_Cycle_OC2_Set(pwmArguPtr, pwmArguPtr->Oc2DutyCycle);
	
}

void Pwm_Duty_Cycle_OC3_Add(PwmParaStruct_P pwmArguPtr, u8 addStep)
{	
	pwmArguPtr->Oc3DutyCycle += addStep;
	
	if(pwmArguPtr->Oc3DutyCycle > 100)
	{
		pwmArguPtr->Oc3DutyCycle = 100;
	}
	
	Pwm_Duty_Cycle_OC3_Set(pwmArguPtr, pwmArguPtr->Oc3DutyCycle);
	
}

void Pwm_Duty_Cycle_OC4_Add(PwmParaStruct_P pwmArguPtr, u8 addStep)
{	
	pwmArguPtr->Oc4DutyCycle += addStep;
	
	if(pwmArguPtr->Oc4DutyCycle > 100)
	{
		pwmArguPtr->Oc4DutyCycle = 100;
	}
	
	Pwm_Duty_Cycle_OC4_Set(pwmArguPtr, pwmArguPtr->Oc4DutyCycle);
	
}

void Pwm_Duty_Cycle_OC1_Sub(PwmParaStruct_P pwmArguPtr, u8 subStep)
{	
	
	if(pwmArguPtr->Oc1DutyCycle < subStep)
	{
		pwmArguPtr->Oc1DutyCycle = 0;
	}
	else
	{
		pwmArguPtr->Oc1DutyCycle -= subStep;
	}
	printf("DutyCycle = %d\r\n", pwmArguPtr->Oc1DutyCycle);
	Pwm_Duty_Cycle_OC1_Set(pwmArguPtr, pwmArguPtr->Oc1DutyCycle);
	
}

void Pwm_Duty_Cycle_OC2_Sub(PwmParaStruct_P pwmArguPtr, u8 subStep)
{	
	
	if(pwmArguPtr->Oc2DutyCycle < subStep)
	{
		pwmArguPtr->Oc2DutyCycle = 0;
	}
	else
	{
		pwmArguPtr->Oc2DutyCycle -= subStep;
	}
	
	Pwm_Duty_Cycle_OC2_Set(pwmArguPtr, pwmArguPtr->Oc2DutyCycle);
	
}

void Pwm_Duty_Cycle_OC3_Sub(PwmParaStruct_P pwmArguPtr, u8 subStep)
{	
	
	if(pwmArguPtr->Oc3DutyCycle < subStep)
	{
		pwmArguPtr->Oc3DutyCycle = 0;
	}
	else
	{
		pwmArguPtr->Oc3DutyCycle -= subStep;
	}
	
	Pwm_Duty_Cycle_OC3_Set(pwmArguPtr, pwmArguPtr->Oc3DutyCycle);
	
}

void Pwm_Duty_Cycle_OC4_Sub(PwmParaStruct_P pwmArguPtr, u8 subStep)
{	
	
	if(pwmArguPtr->Oc4DutyCycle < subStep)
	{
		pwmArguPtr->Oc4DutyCycle = 0;
	}
	else
	{
		pwmArguPtr->Oc4DutyCycle -= subStep;
	}
	
	Pwm_Duty_Cycle_OC4_Set(pwmArguPtr, pwmArguPtr->Oc4DutyCycle);
	
}

void PwmOptsBinding(PwmOperaterStruct_P PwmOptsPtr)
{
	PwmOptsPtr->Pwm_Frequency_Set = Pwm_Frequency_Set;
	
	PwmOptsPtr->Duty_Cycle_OC1_Set = Pwm_Duty_Cycle_OC1_Set;
	PwmOptsPtr->Duty_Cycle_OC2_Set = Pwm_Duty_Cycle_OC2_Set;
	PwmOptsPtr->Duty_Cycle_OC3_Set = Pwm_Duty_Cycle_OC3_Set;
	PwmOptsPtr->Duty_Cycle_OC4_Set = Pwm_Duty_Cycle_OC4_Set;
	
	PwmOptsPtr->Duty_Cycle_OC1_Add = Pwm_Duty_Cycle_OC1_Add;
	PwmOptsPtr->Duty_Cycle_OC2_Add = Pwm_Duty_Cycle_OC2_Add;
	PwmOptsPtr->Duty_Cycle_OC3_Add = Pwm_Duty_Cycle_OC3_Add;
	PwmOptsPtr->Duty_Cycle_OC4_Add = Pwm_Duty_Cycle_OC4_Add;
	
	PwmOptsPtr->Duty_Cycle_OC1_Sub = Pwm_Duty_Cycle_OC1_Sub;
	PwmOptsPtr->Duty_Cycle_OC2_Sub = Pwm_Duty_Cycle_OC2_Sub;
	PwmOptsPtr->Duty_Cycle_OC3_Sub = Pwm_Duty_Cycle_OC3_Sub;
	PwmOptsPtr->Duty_Cycle_OC4_Sub = Pwm_Duty_Cycle_OC4_Sub;
}


u8 Pwm_Init(void)
{
	pwmParaPtr_1->TimX = TIM3;
	pwmParaPtr_1->Frequency = 2;
	pwmParaPtr_1->Oc1DutyCycle = 0;
	pwmParaPtr_1->Oc2DutyCycle = 0;
	pwmParaPtr_1->Oc3DutyCycle = 0;
	pwmParaPtr_1->Oc4DutyCycle = 0;
	pwmParaPtr_1->PwmResolution = 100;
	pwmParaPtr_1->ResolutionPreStep = 0;
	pwmParaPtr_1->PwmFrequencyCounterNum = 0;
	
	PwmOptsBinding(pwmOptsPtr_1);
	
	if(pwmOptsPtr_1->Pwm_Frequency_Set(pwmParaPtr_1))
	{
		return 1;
	}

	//Pwm_Duty_Cycle_Init(1, pwmParaPtr_1);
	//Pwm_Duty_Cycle_Init(2, pwmParaPtr_1);
	Pwm_Duty_Cycle_Init(3, pwmParaPtr_1);
	Pwm_Duty_Cycle_Init(4, pwmParaPtr_1);

	TIM_ARRPreloadConfig(pwmParaPtr_1->TimX, ENABLE);
	
	/* 启动 TIM 计数 */
	TIM_Cmd(pwmParaPtr_1->TimX, ENABLE);	
	
	return 0;
}



