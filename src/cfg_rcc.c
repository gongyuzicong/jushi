#include "cfg_rcc.h"


/**********************************************************************************
函数名: CB_RCC_Config
函数描述: 配置系统各部分时钟
输入参数: 无
输出结果: PLL_CLK = 72MHz, HCLK = 72MHz, PCLK1 = 36MHz, PCLK2 = 72MHz
返回值: 若设置成功，返回SUCCESS, 若设置失败，返回ERROR
***********************************************************************************/
#if 1

FlagStatus Check_HSEStatus(void)
{
	FlagStatus bitStatus = RESET;

	if(( RCC->CR & 1 << 17) != (u32)RESET)
	{
		bitStatus = SET;
	}

	return bitStatus;
}

	
ErrorStatus RCC_WaitingForHSEStartUp(void)
{
	vu32 StartUpCounter = HSEStartUp_TimeOut;
	ErrorStatus status = ERROR;
	FlagStatus HSEStatus = RESET;
	
	do
	{
		HSEStatus = Check_HSEStatus();
		StartUpCounter--;
	}while((HSEStatus == RESET) && (StartUpCounter));

	if(RESET != HSEStatus)
	{
		status = SUCCESS;
	}

	return status;
}

ErrorStatus CB_RCC_Config(void)
{
	//ErrorStatus HSEStartUpStatus;	/*定义HSE就绪位*/
	ErrorStatus cbRccConfigStatus;
	RCC_DeInit();
	
	/*1. 通过RCC里的时钟控制寄存器(RCC Clock Control Register)开启HSE*/
	//RCC_HSEConfig(RCC_HSE_ON);		/*通过RCC开启HSE*/
	
	//Reset HSEON bit
	RCC->CR &= 0xFFFAFFFF;		//HSEBYP:0 HSEON:0
	//set HSEON Enable
	RCC->CR |= 1 << 16;
	
	//HSEStartUpStatus = RCC_WaitForHSEStartUp();		/*等待HSE稳定(通过就绪位判断)*/
	
	if(SUCCESS == RCC_WaitingForHSEStartUp())	/*如果HSE开启成功*/
	//if(SUCCESS == HSEStartUpStatus)
	{
		/*2. 设置FLASH预取缓冲*/
		//FLASH_SetLatency(FLASH_Latency_2);	/*设置FLASH缓冲延时周期为2*/
		//FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);	/*使能FLASH预取缓存*/
		FLASH->ACR |= 0x12;
		//while(1 != ((FLASH->ACR >> 5) & 1));

		/*3. 配置PLL锁相环*/
		//RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		
		/*4. 配置HCLK*/
		//RCC_HCLKConfig(RCC_SYSCLK_Div1);	/*设置HCLK时钟源为SYSCLK 1分频*/
		
		/*5. 配置PCLK1*/
		//RCC_PCLK1Config(RCC_HCLK_Div2);		/*设置PCLK1时钟源为HCLK(AHB) 2分频*/
		
		/*6. 配置PCLK2*/
		//RCC_PCLK2Config(RCC_HCLK_Div1);		/*设置PCLK2时钟源为HCLK(AHB) 1分频*/
		
		RCC->CFGR |= 0x071D040C;

		/*7. 使能PLL并且等待PLL稳定*/
		//RCC_PLLCmd(ENABLE);		/*使能PLL*/
		RCC->CR |= 1 << 24;
		//while(RESET == RCC_GetFlagStatus(RCC_FLAG_PLLRDY));		/*等待PLL稳定输出*/
		while(1 != ((RCC->CR >> 25) & 1));

		/*8. 将SYSCLK时钟源切换为PLL，并等待稳定*/
		//RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);		/*选择PLL作为SYSCLK时钟源*/
		RCC->CFGR |= 0x0000000E;
		//while(0x08 != RCC_GetSYSCLKSource());		/*等待PLL成为SYSCLK时钟源*/
		while(0x08 != (RCC->CFGR & (u32)0x0000000C));
		
		cbRccConfigStatus = SUCCESS;
	}
	else	/*如果HSE开启失败*/
	{
		cbRccConfigStatus = ERROR;
	}

	return (cbRccConfigStatus);
}

#else

FlagStatus Check_HSEStatus(void)
{
	FlagStatus bitStatus = RESET;

	if(( RCC->CR & 1 << 17) != (u32)RESET)
	{
		bitStatus = SET;
	}

	return bitStatus;
}

	
ErrorStatus RCC_WaitingForHSEStartUp(void)
{
	vu32 StartUpCounter = HSEStartUp_TimeOut;
	ErrorStatus status = ERROR;
	FlagStatus HSEStatus = RESET;

	
	do
	{
		HSEStatus = Check_HSEStatus();
		StartUpCounter--;
	}while((HSEStatus == RESET) && (StartUpCounter));

	if(RESET != HSEStatus)
	{
		status = SUCCESS;
	}

	return status;
}

ErrorStatus CB_RCC_Config(void)
{
	ErrorStatus HSEStartUpStatus, RCCConfigStatus = ERROR;
	RCC_DeInit();

	//Reset HSEON bit
	RCC->CR &= 0xFFFAFFFF;		//HSEBYP:0 HSEON:0
	//set HSEON Enable
	//RCC->CR |= 1 << 16;
	RCC->CR |= 0x00010000;

	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	
	//wait for HSE startup
	//if(SUCCESS == RCC_WaitingForHSEStartUp())
	if(SUCCESS == HSEStartUpStatus)
	{
		FLASH_SetLatency(FLASH_Latency_2);	/*设置FLASH缓冲延时周期为2*/
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);	/*使能FLASH预取缓存*/
		
		//Config
		RCC->CFGR |= 0x075DC40C;

		//PLL Enable
		RCC->CR |= 1 << 24;
		//wait for PLL Ready
		while(1 != (RCC->CR & (1 << 25)));

		//System Clock Switch To PLL
		RCC->CFGR |= 0xFFFFFFFE;
		//wait for switch ready
		while(0x08 != (RCC->CFGR & 0x0C));
		
		RCCConfigStatus = SUCCESS;
	}
	
	return RCCConfigStatus;
}

#endif



