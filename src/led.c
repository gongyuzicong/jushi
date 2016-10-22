#include "led.h"



LED_Control_Str 	WarningLedCtrl;
LED_Control_Str_P 	WarningLedCtrlPtr = &WarningLedCtrl;





void LED_Status_Handle(void)
{
	if(0x00 != ctrlParasPtr->LED_Warning)
	{
		if(WARNING_STATUS_FLMT_SW_CHECK)
		{
			Warning_LED_FLMT_SW_ERR_STATUS();
		}
		else if(WARNING_STATUS_NORMAL_CHECK)
		{
			Warning_LED_NORMAL_STATUS();
		}
	}
	
	
}






/********************************************************************/





void SIMU_PWM_BreathBoardLED_Ctrl(void)
{
	static u32 timRec = 0;
	static u8 step = 0, flag = 0, counter = 0;
	static u16 dutyTime = 0;
	
	if(0 == timRec)
	{
		timRec = SystemRunningTime;
		Board_LED_ON();
		step = 0;
	}
	else
	{
		
		if(0 == step)
		{
			if(SystemRunningTime - timRec >= dutyTime)
			{
				Board_LED_OFF();
				step = 1;
				timRec = SystemRunningTime;
			}
		}
		else if(1 == step)
		{
			if(SystemRunningTime - timRec >= (100 - dutyTime))
			{
				timRec = 0;
				step = 0;

				if(counter < 1)
				{
					counter++;
				}
				else
				{
					counter = 0;
					if(0 == dutyTime)
					{
						flag = 0;
					}
					else if(dutyTime >= 100)
					{
						flag = 1;
					}

					if(0 == flag)
					{
						dutyTime++;
					}
					else if(1 == flag)
					{
						dutyTime--;
					}
				}
				
				
			}
		}
		
	}
	
}


void Warning_LED_ON(void)
{
	Warning_LED_RED_ON();
	Warning_LED_GREEN_ON();
	Warning_LED_ORANGE_ON();
}

void Warning_LED_OFF(void)
{
	Warning_LED_RED_OFF();
	Warning_LED_GREEN_OFF();
	Warning_LED_ORANGE_OFF();
}

void Warning_LED_ON_CHECK(void)
{
	Warning_LED_RED_ON_CHECK();
	Warning_LED_GREEN_ON_CHECK();
	Warning_LED_ORANGE_ON_CHECK();
}


void WarningLedTwinkleCtrl(LED_Control_Str_P ptr)
{	
	if(1 == ptr->twinkleFlag)
	{
		if(0 == ptr->timRec)
		{
			ptr->timRec = SystemRunningTime;
			//Warning_LED_GREEN_OFF();
			ptr->LED_OFF();
			
			ptr->step = 0;
		}
		else
		{
			
			if(0 == ptr->step)
			{
				if(SystemRunningTime - ptr->timRec >= ptr->intervalTime_ms * 10)
				{
					//Warning_LED_GREEN_ON();
					ptr->LED_ON_CHECK();
					ptr->step = 1;
					ptr->timRec = SystemRunningTime;
				}
			}
			else if(1 == ptr->step)
			{
				if(SystemRunningTime - ptr->timRec >= ptr->intervalTime_ms * 10)
				{
					ptr->timRec = 0;
					
					ptr->twinkleCounter++;
				}
			}
			
		}

		if(LED_Twinkle_Count_Enable == ptr->mode)
		{
			if(ptr->twinkleCounter >= ptr->twinkleNum)
			{
				ptr->twinkleFlag = 0;
				ptr->twinkleCounter = 0;
				ptr->step = 0;
			}
		}
		else if(LED_Twinkle_Count_Disable == ptr->mode)
		{
			ptr->twinkleCounter = 0;
		}
		
	}
	
}


void Warning_LED_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

}


void Board_LED_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*设置GPIOA.2和GPIOA.3为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	/*打开APB2总线上的GPIOA时钟*/
}



void LED_Init(void)
{
	Warning_LED_GPIO_Init();
	
	Board_LED_GPIO_Init();
	
	WarningLedCtrlPtr->twinkleFlag = 0;
	WarningLedCtrlPtr->intervalTime_ms = 100;
	WarningLedCtrlPtr->twinkleNum = 2;

	WarningLedCtrlPtr->step = 0;
	WarningLedCtrlPtr->twinkleCounter = 0;
	WarningLedCtrlPtr->timRec = 0;

	WarningLedCtrlPtr->LED_ON = Warning_LED_ON;
	WarningLedCtrlPtr->LED_OFF = Warning_LED_OFF;
	WarningLedCtrlPtr->LED_ON_CHECK = Warning_LED_ON_CHECK;
	WarningLedCtrlPtr->twinkleCtrlFunc = WarningLedTwinkleCtrl;

	Warning_LED_RED_OFF();
	Warning_LED_GREEN_ON();
	Warning_LED_ORANGE_OFF();

	Board_LED_ON();

	WARNING_STATUS_NORMAL_SET();
}


