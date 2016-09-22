#include "ecv_control.h"

PwmParaStruct Timer4PwmPara;
PwmParaStruct_P timer4PwmParaPtr = &Timer4PwmPara;

Ecv_Ctrl_Struct 	FECV_Str;
Ecv_Ctrl_Struct_P 	FECV_Str_Ptr = &FECV_Str;
Ecv_Ctrl_Struct 	BECV_Str;
Ecv_Ctrl_Struct_P 	BECV_Str_Ptr = &BECV_Str;


void CheckFECV_Limt(void)
{
	

	
	
}


void FECV_Ctrl_Func(void)
{
	static u16 speedRec = 0;

	
	if(0 == FECV_Str_Ptr->EcvDir)
	{
		FECV_SPEED_SET(0);
		FECV_Str_Ptr->EcvEnableHallFlag = 0;
	}
	else
	{
		if(FECV_Str_Ptr->EcvDir > 0)
		{
			FECV_UP();
		}
		else if(FECV_Str_Ptr->EcvDir < 0)
		{
			FECV_DOWN();
		}

		FECV_Str_Ptr->EcvEnableHallFlag = 1;

		if(FECV_Str_Ptr->EcvHallCount >= FECV_Str_Ptr->EcvHallCountCmpSet)
		{
			FECV_Str_Ptr->EcvDir = 0;
		}
		
		if(speedRec != FECV_Str_Ptr->EcvSpeed)
		{
			speedRec = FECV_Str_Ptr->EcvSpeed;
			
			FECV_SPEED_SET(FECV_Str_Ptr->EcvSpeed);
		}
	}
	
}

void BECV_Ctrl_Func(void)
{
	static u16 speedRec = 0;

	
	if(0 == BECV_Str_Ptr->EcvDir)
	{
		FECV_SPEED_SET(0);
		BECV_Str_Ptr->EcvEnableHallFlag = 0;
	}
	else
	{
		if(BECV_Str_Ptr->EcvDir > 0)
		{
			FECV_UP();
		}
		else if(BECV_Str_Ptr->EcvDir < 0)
		{
			FECV_DOWN();
		}

		BECV_Str_Ptr->EcvEnableHallFlag = 1;

		if(BECV_Str_Ptr->EcvHallCount >= BECV_Str_Ptr->EcvHallCountCmpSet)
		{
			BECV_Str_Ptr->EcvDir = 0;
		}
		
		if(speedRec != BECV_Str_Ptr->EcvSpeed)
		{
			speedRec = BECV_Str_Ptr->EcvSpeed;
			
			FECV_SPEED_SET(BECV_Str_Ptr->EcvSpeed);
		}
	}
	
}



void ECV_GPIO_Config(void)
{
	#if 1
	
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	/*打开APB2总线上的GPIOA时钟*/
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	#else

	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	/*打开APB2总线上的GPIOA时钟*/
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*打开APB2总线上的GPIOA时钟*/
	

	#endif
}



u8 ECV_Pwm_Init(void)
{
	/*初始化结构体*/
	timer4PwmParaPtr->TimX = TIM4;
	timer4PwmParaPtr->Frequency = 1;
	timer4PwmParaPtr->Oc1DutyCycle = 0;
	timer4PwmParaPtr->Oc2DutyCycle = 0;
	timer4PwmParaPtr->Oc3DutyCycle = 0;
	timer4PwmParaPtr->Oc4DutyCycle = 0;
	timer4PwmParaPtr->PwmResolution = 100;
	timer4PwmParaPtr->ResolutionPreStep = 0;
	timer4PwmParaPtr->PwmFrequencyCounterNum = 0;
	
	if(pwmOptsPtr_1->Pwm_Frequency_Set(timer4PwmParaPtr))
	{
		return 1;
	}

	//Pwm_Duty_Cycle_Init(1, timer4PwmParaPtr);
	//Pwm_Duty_Cycle_Init(2, timer4PwmParaPtr);
	Pwm_Duty_Cycle_Init(3, timer4PwmParaPtr);
	Pwm_Duty_Cycle_Init(4, timer4PwmParaPtr);

	TIM_ARRPreloadConfig(timer4PwmParaPtr->TimX, ENABLE);
	
	/* 启动 TIM 计数 */
	TIM_Cmd(timer4PwmParaPtr->TimX, ENABLE);	
	
	return 0;
}



void ECV_Init(void)
{
	ECV_GPIO_Config();

	//ECV_Pwm_Init();

	FECV_Str_Ptr->EcvDir = 0;
	FECV_Str_Ptr->EcvEnableHallFlag = 0;
	FECV_Str_Ptr->EcvHallCount = 0;
	FECV_Str_Ptr->EcvSpeed = 100;

	*BECV_Str_Ptr = *FECV_Str_Ptr;
	
}





