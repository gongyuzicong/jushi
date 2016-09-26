#include "ecv_control.h"
#include "timer_opts.h"

PwmParaStruct Timer4PwmPara;
PwmParaStruct_P timer4PwmParaPtr = &Timer4PwmPara;

Ecv_Ctrl_Struct 	FECV_Str;
Ecv_Ctrl_Struct_P 	FECV_Str_Ptr = &FECV_Str;
Ecv_Ctrl_Struct 	BECV_Str;
Ecv_Ctrl_Struct_P 	BECV_Str_Ptr = &BECV_Str;
Ecv_Ctrl_Struct 	WECV_Str;
Ecv_Ctrl_Struct_P 	WECV_Str_Ptr = &WECV_Str;
HallCountCmpManagerStruct 	HallCountCmpManager_Str;
HallCountCmpManagerStruct_P HallCountCmpManager_Str_Ptr = &HallCountCmpManager_Str;


void Save_EcvHallCountCmp_To_E2PROM(u16 set, u16 addr)
{
	
	
	
}

void Set_FEcvHallCountCmp(u16 set)
{
	FECV_Str_Ptr->EcvHallCountCmp = set;
	Save_EcvHallCountCmp_To_E2PROM(set, FEcvHallCountCmp_Addr);
}

void Set_BEcvHallCountCmp(u16 set)
{
	BECV_Str_Ptr->EcvHallCountCmp = set;
	Save_EcvHallCountCmp_To_E2PROM(set, BEcvHallCountCmp_Addr);
}

void Set_WEcvHallCountCmp(u16 set)
{
	WECV_Str_Ptr->EcvHallCountCmp = set;
	Save_EcvHallCountCmp_To_E2PROM(set, WEcvHallCountCmp_Addr);
}



u8 CheckFECV_Limt(Ecv_Ctrl_Struct_P ptr)
{
	u8 flag = 0;
	
	if(ptr->EcvHallCountRec < ptr->EcvHallCount)
	{
		ptr->EcvHallCountRec 		= ptr->EcvHallCount;
		ptr->EcvHallCountTimeOut 	= SystemRunningTime;
	}
	else if(ptr->EcvHallCountRec == ptr->EcvHallCount)
	{
		if(SystemRunningTime - ptr->EcvHallCountTimeOut >= 5000)
		{
			flag 					= 1;
			ptr->EcvHallCountRec 	= 0;
			printf("time out\r\n");
		}
	}
	else
	{
		printf("CheckFECV_Limt error!\r\n");
	}

	return flag;
}


void FECV_UpDownFunc(EcvDir ecvDir)
{
	if(ECV_UP == ecvDir)
	{
		FECV_UP();
		//printf("FECV_UP\r\n");
	}
	else if(ECV_DOWN == ecvDir)
	{
		FECV_DOWN();
		//printf("FECV_DOWN\r\n");
	}
	
}

void FECV_PowerOnOffFunc(ECV_PowerOnOff PowerOnOff)
{
	static ECV_PowerOnOff temp = ECV_POWER_OFF;

	if(temp != PowerOnOff)
	{
		temp = PowerOnOff;
		
		if(ECV_POWER_ON == PowerOnOff)
		{
			FECV_POWER_ON();
		}
		else if(ECV_POWER_OFF == PowerOnOff)
		{
			FECV_POWER_OFF();
		}
	}
	
}

void FECV_SetSpeedFunc(u8 speed)
{
	static u8 EcvSpeedRec = 0;
	
	if(EcvSpeedRec != speed)
	{
		if(speed <= 100)
		{
			EcvSpeedRec = speed;
			
			FECV_SPEED_SET(speed);
		}
		
	}
}

void FECV_SetPara(Ecv_Para_P ptr)
{
	if(ECV_UNUSED == FECV_Str_Ptr->UseStatus)
	{
		FECV_Str_Ptr->Dir 				= ptr->Dir;
		FECV_Str_Ptr->EcvSpeed 			= ptr->EcvSpeed;
		FECV_Str_Ptr->HallCountMode 	= ptr->HallCountMode;
		FECV_Str_Ptr->EcvHallCountCmp 	= ptr->EcvHallCountCmp;
		FECV_Str_Ptr->UseStatus 		= ECV_USEING;
	}
	
}

void FECV_Clean_Use_Status(void)
{
	FECV_Str_Ptr->UseStatus = ECV_UNUSED;
}

void BECV_UpDownFunc(EcvDir ecvDir)
{
	if(ECV_UP == ecvDir)
	{
		BECV_UP();
		//printf("BECV_UP\r\n");
	}
	else if(ECV_DOWN == ecvDir)
	{
		BECV_DOWN();
		//printf("BECV_DOWN\r\n");
	}
	
}

void BECV_PowerOnOffFunc(ECV_PowerOnOff PowerOnOff)
{
	static ECV_PowerOnOff temp = ECV_POWER_OFF;

	if(temp != PowerOnOff)
	{
		temp = PowerOnOff;

		if(ECV_POWER_ON == PowerOnOff)
		{
			BECV_POWER_ON();
		}
		else if(ECV_POWER_OFF == PowerOnOff)
		{
			BECV_POWER_OFF();
		}
		
	}
	
}

void BECV_SetSpeedFunc(u8 speed)
{

	static u8 EcvSpeedRec = 0;
	
	if(EcvSpeedRec != speed)
	{
		if(speed <= 100)
		{
			EcvSpeedRec = speed;
			
			BECV_SPEED_SET(speed);
		}
		
	}
}

void BECV_SetPara(Ecv_Para_P ptr)
{
	if(ECV_UNUSED == BECV_Str_Ptr->UseStatus)
	{
		BECV_Str_Ptr->Dir 				= ptr->Dir;
		BECV_Str_Ptr->EcvSpeed 			= ptr->EcvSpeed;
		BECV_Str_Ptr->HallCountMode 	= ptr->HallCountMode;
		BECV_Str_Ptr->EcvHallCountCmp 	= ptr->EcvHallCountCmp;
		BECV_Str_Ptr->UseStatus 		= ECV_USEING;

		//printf("Dir = %d\r\n", BECV_Str_Ptr->Dir);
		//printf("EcvSpeed = %d\r\n", BECV_Str_Ptr->EcvSpeed);
		//printf("HallCountMode = %d\r\n", BECV_Str_Ptr->HallCountMode);
		//printf("EcvHallCountCmp = %d\r\n", BECV_Str_Ptr->EcvHallCountCmp);
		
	}
	
}

void BECV_Clean_Use_Status(void)
{
	BECV_Str_Ptr->UseStatus = ECV_UNUSED;
}

void WECV_UpDownFunc(EcvDir ecvDir)
{
	if(ECV_UP == ecvDir)
	{
		WECV_UP();
		//printf("WECV_UP\r\n");
	}
	else if(ECV_DOWN == ecvDir)
	{
		WECV_DOWN();
		//printf("WECV_DOWN\r\n");
	}
	
}

void WECV_PowerOnOffFunc(ECV_PowerOnOff PowerOnOff)
{
	
	static ECV_PowerOnOff temp = ECV_POWER_OFF;
	
	if(temp != PowerOnOff)
	{
		temp = PowerOnOff;

		if(ECV_POWER_ON == PowerOnOff)
		{
			WECV_POWER_ON();
		}
		else if(ECV_POWER_OFF == PowerOnOff)
		{
			WECV_POWER_OFF();
		}
		
	}
	
}

void WECV_SetSpeedFunc(u8 speed)
{

	static u8 EcvSpeedRec = 0;
	
	if(EcvSpeedRec != speed)
	{
		EcvSpeedRec = speed;
		
		if(speed <= 100)
		{
			if(0 == speed)
			{
				ECV3_PWM = 1;
			}
			else
			{
				ECV3_PWM = 0;
			}
		}
		
	}
}

void WECV_SetPara(Ecv_Para_P ptr)
{
	if(ECV_UNUSED == WECV_Str_Ptr->UseStatus)
	{
		WECV_Str_Ptr->Dir 				= ptr->Dir;
		WECV_Str_Ptr->EcvSpeed 			= ptr->EcvSpeed;
		WECV_Str_Ptr->HallCountMode 	= ptr->HallCountMode;
		WECV_Str_Ptr->EcvHallCountCmp 	= ptr->EcvHallCountCmp;
		WECV_Str_Ptr->UseStatus 		= ECV_USEING;
	}
	
}

void WECV_Clean_Use_Status(void)
{
	WECV_Str_Ptr->UseStatus = ECV_UNUSED;
}


void ECV_Ctrl_Func(Ecv_Ctrl_Struct_P ptr)
{
	if(ECV_STOP== ptr->Dir)
	{
		ptr->ECV_SetSpeedFunc(0);
		ptr->EcvEnableHallFlag 		= 0;
		ptr->EcvHallCountRec 		= 0;
		ptr->EcvHallCountTimeOut 	= SystemRunningTime;
	}
	else
	{
		ptr->EcvEnableHallFlag 		= 1;

		ptr->ECV_UpDownFunc(ptr->Dir);
		
		if(ECV_USE_HALL_COUNT_MODE_ENABLE == ptr->HallCountMode)
		{
			if(CheckFECV_Limt(ptr) || (ptr->EcvHallCount >= ptr->EcvHallCountCmp))
			{
				ptr->Dir 			= ECV_STOP;
				ptr->UseStatus 		= ECV_COMPLETE;
			}
		}
		else if(ECV_USE_HALL_COUNT_MODE_DISABLE == ptr->HallCountMode)
		{
			if(CheckFECV_Limt(ptr))
			{
				ptr->Dir 			= ECV_STOP;
				ptr->UseStatus 		= ECV_COMPLETE;
			}
		}

		ptr->ECV_SetSpeedFunc(ptr->EcvSpeed);
	}
}

void ECV_Ctrl_Func_W(Ecv_Ctrl_Struct_P ptr)
{
	
	if(ECV_STOP == ptr->Dir)
	{
		ptr->ECV_SetSpeedFunc(0);
		ptr->EcvEnableHallFlag = 0;
	}
	else
	{
		ptr->ECV_UpDownFunc(ptr->Dir);

		ptr->EcvEnableHallFlag = 1;

		if(((ECV_UP == ptr->Dir) && (WECV_DOWN_LIMT_SW_RESP)) ||\
			((ECV_DOWN == ptr->Dir) && (WECV_UP_LIMT_SW_RESP)))
		{
			ptr->Dir = ECV_STOP;
			printf("ECV_STOP\r\n");
		}
		else
		{
			ptr->ECV_SetSpeedFunc(ptr->EcvSpeed);
		}
		//ptr->ECV_PowerOnOffFunc(ptr->Power);
	}
	
}

void ECV_Ctrl_Func_F(Ecv_Ctrl_Struct_P ptr)
{
	
	if(ECV_STOP == ptr->Dir)
	{
		ptr->ECV_SetSpeedFunc(0);
		ptr->EcvEnableHallFlag 		= 0;
		ptr->EcvHallCountTimeOut 	= SystemRunningTime;
	}
	else
	{
		ptr->ECV_UpDownFunc(ptr->Dir);

		ptr->EcvEnableHallFlag = 1;
		
		ptr->ECV_SetSpeedFunc(ptr->EcvSpeed);
		//ptr->ECV_PowerOnOffFunc(ptr->Power);
	}
	
}



void M_A_Init(void)
{
	Ecv_Para temp;
	temp.Dir 			= ECV_DOWN;
	temp.EcvSpeed 		= 100;
	temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_DISABLE;
	BECV_Str_Ptr->ECV_SetPara(&temp);
	BECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
	while(ECV_USEING == BECV_Str_Ptr->UseStatus)
	{
		ECV_Ctrl_Func(BECV_Str_Ptr);
	}
	BECV_Str_Ptr->ECV_Clean_Use_Status();
	temp.Dir 			 = ECV_UP;
	temp.HallCountMode 	 = ECV_USE_HALL_COUNT_MODE_ENABLE;
	temp.EcvHallCountCmp = HallCountCmpManager_Str_Ptr->BecvInit;
	BECV_Str_Ptr->ECV_SetPara(&temp);
	while(ECV_USEING == BECV_Str_Ptr->UseStatus)
	{
		ECV_Ctrl_Func(BECV_Str_Ptr);
	}
	BECV_Str_Ptr->ECV_Clean_Use_Status();
	/*
	FECV_Str_Ptr->ECV_SetSpeedFunc(100);
	FECV_Str_Ptr->ECV_UpDownFunc(ECV_UP);
	FECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
	Delay_ns(3);
	FECV_Str_Ptr->ECV_SetSpeedFunc(0);
	FECV_Str_Ptr->Dir = ECV_STOP;
	*/
}


void ECV_GPIO_Config(void)
{
	#if 0
	
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	/*��APB2�����ϵ�GPIOAʱ��*/
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	/*��APB2�����ϵ�GPIOAʱ��*/

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*��APB2�����ϵ�GPIOAʱ��*/

	#else

	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	/*��APB2�����ϵ�GPIOAʱ��*/
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	/*��APB2�����ϵ�GPIOAʱ��*/

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*��APB2�����ϵ�GPIOAʱ��*/
	
	GPIO_SetBits(GPIOE, GPIO_Pin_8);

	
	
	#endif
}

void ECV_SW_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*��APB2�����ϵ�GPIOAʱ��*/
}

u8 ECV_Pwm_Init(void)
{
	/*��ʼ���ṹ��*/
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
	
	/* ���� TIM ���� */
	TIM_Cmd(timer4PwmParaPtr->TimX, ENABLE);	
	
	return 0;
}



void ECV_Struct_Init(void)
{
	FECV_Str_Ptr->Dir 					= ECV_STOP;
	FECV_Str_Ptr->EcvEnableHallFlag 	= 0;
	FECV_Str_Ptr->EcvHallCount 			= 0;
	FECV_Str_Ptr->EcvSpeed 				= 100;
	FECV_Str_Ptr->EcvHallCountCmp 		= 100;
	FECV_Str_Ptr->HallCountMode 		= ECV_USE_HALL_COUNT_MODE_ENABLE;
	FECV_Str_Ptr->Power					= ECV_POWER_OFF;
	FECV_Str_Ptr->UseStatus				= ECV_UNUSED;

	*BECV_Str_Ptr = *WECV_Str_Ptr = *FECV_Str_Ptr;

	FECV_Str_Ptr->ECV_PowerOnOffFunc 	= FECV_PowerOnOffFunc;
	FECV_Str_Ptr->ECV_UpDownFunc 		= FECV_UpDownFunc;
	FECV_Str_Ptr->ECV_SetSpeedFunc 		= FECV_SetSpeedFunc;
	FECV_Str_Ptr->ECV_SetPara			= FECV_SetPara;
	FECV_Str_Ptr->ECV_Clean_Use_Status 	= FECV_Clean_Use_Status;

	BECV_Str_Ptr->ECV_PowerOnOffFunc 	= BECV_PowerOnOffFunc;
	BECV_Str_Ptr->ECV_UpDownFunc 		= BECV_UpDownFunc;
	BECV_Str_Ptr->ECV_SetSpeedFunc 		= BECV_SetSpeedFunc;
	BECV_Str_Ptr->ECV_SetPara			= BECV_SetPara;
	BECV_Str_Ptr->ECV_Clean_Use_Status 	= BECV_Clean_Use_Status;

	WECV_Str_Ptr->ECV_PowerOnOffFunc 	= WECV_PowerOnOffFunc;
	WECV_Str_Ptr->ECV_UpDownFunc 		= WECV_UpDownFunc;
	WECV_Str_Ptr->ECV_SetSpeedFunc 		= WECV_SetSpeedFunc;
	WECV_Str_Ptr->ECV_SetPara			= WECV_SetPara;
	WECV_Str_Ptr->ECV_Clean_Use_Status 	= WECV_Clean_Use_Status;

	HallCountCmpManager_Str_Ptr->FecvBigFiberHall 	= 0;
	HallCountCmpManager_Str_Ptr->FecvSmallFiberHall = 0;

	HallCountCmpManager_Str_Ptr->BecvInit			= 100;
	HallCountCmpManager_Str_Ptr->BecvBigFiberHall	= 0;
	HallCountCmpManager_Str_Ptr->BecvSmallFiberHall	= 0;
}


void ECV_Init(void)
{
	ECV_GPIO_Config();

	ECV_Pwm_Init();

	ECV_Struct_Init();

	ECV_SW_Init();

	ECV_ALL_POWER_OFF();
}





