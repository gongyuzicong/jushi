#include "main_include.h"
#include "cfg_rcc.h"
#include "cfg_gpio.h"
#include "cfg_usart.h"
#include "timer_opts.h"
#include "key_opts.h"
#include "buffer.h"
//#include "dht11_opts.h"
#include "pwm_opts.h"
#include "motion_control.h"
#include "exti_opts.h"
#include "nrf24l01_opts.h"
#include "spi_opts.h"

void SystemInit(void)
{	
	keyConfig();
	BufferOpts_Init();
	Delay_Init(72);
	//Timer2_Init(10000, 7199);
	Timer2_Init(65535, 35999);
	TIM3_Init(10000, 7199);		// 1s
	Timer4_Init(10, 7199);		// 1ms
	Motion_Ctrl_Init();
	Pwm_Init();
	SPI_Initial();
	NFR24L01_Init();
	ExtiInit();
	
	errorInfo.errorType = errorTypeBegin;
	errorInfo.errorData = 0;
}



int main(void)
{
	//TIMx_PwmOpts_Struct TIM3_PWM;
	//int cir = 1, cir2 = 0;
	
	CB_RCC_Config();	/*ÅäÖÃÏµÍ³Ê±ÖÓ*/
	CB_GPIO_Config();	/*ÅäÖÃGPIO¿Ú*/
	CB_USART_Config();	/*ÅäÖÃUSART*/
	SystemInit();
	
	while(1)
	{
		if(keyScanFlag)
		{
			keyEvent(keyScan());
			//printf("%d\r\n", keyScan());
			keyScanFlag = 0;
		}

		if(need2SendInfo)
		{
			need2SendInfo = 0;
			NRF24L01OptsPtr->Send_Info_To_Contorler();
		}
		//NRF24L01OptsPtr->TEST_Recv();
	}

	
}





