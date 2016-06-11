#include "key_opts.h"
#include "cfg_gpio.h"
#include "timer_opts.h"
#include "cfg_usart.h"
//#include "cfg_can.h"
#include "buffer.h"
#include "pwm_opts.h"
#include "motion_control.h"
#include "nrf24l01_opts.h"

void (*keyStatusEventPtr[KEY_SATAUS_STEP])(int *);
void (*keyEventOpts[KEY_NUM])(void);
KeyScanState_Typedef keyScanStatus = KeyScanState_Check;

int keyScan(void)
{
	int scaningKey = -1;
	
	keyStatusEventPtr[keyScanStatus](&scaningKey);

	return scaningKey;
}


void KeyScanState_Check_Opt(int *scaningKey)
{
	//if((0x0007 != (GPIOA->IDR & 0x0007)) || (0x2000 != (GPIOC->IDR & 0x2000)))	// 如果PA0 PA1 PA2有按键按下
	if(0x0001 != (GPIOA->IDR & 0x0001))
	{
		keyScanStatus = KeyScanState_MakeSure;
	}
	else if(0x0000 == (GPIOD->IDR & 0x0004))
	{
		keyScanStatus = KeyScanState_MakeSure;
	}
}

void KeyScanState_MakeSure_Opt(int *scaningKey)
{
	if(0x0000 != (GPIOA->IDR & 0x0001))	// 如果PA0 PA1 PA2有按键按下
	{
		//printf("key1-3\r\n");
		//printf("key_b = %d\r\n", *scaningKey);
		//*scaningKey = (~GPIOA->IDR & 0x0007) / 2;
		*scaningKey = KEY4;
		//printf("key = %d\r\n", *scaningKey);
		keyScanStatus = KeyScanState_Up;
	}
	else if(0x0000 == (GPIOD->IDR & 0x0004))
	{
		*scaningKey = KEY3;
		keyScanStatus = KeyScanState_Up;
	}
}

void KeyScanState_Up_Opt(int *scaningKey)
{
	//if((0x0007 == (GPIOA->IDR & 0x0007)) && (0x2000 == (GPIOC->IDR & 0x2000)))
	if(0x0000 != (GPIOA->IDR & 0x0001))
	{
		//printf("up\r\n");
		keyScanStatus = KeyScanState_Check;
	}
	else if(0x0004 == (GPIOD->IDR & 0x0004))
	{
		keyScanStatus = KeyScanState_Check;
	}
}

void key1Opt(void)
{
	//pwmOptsPtr_1->Duty_Cycle_OC1_Add(pwmParaPtr_1, 10);
	//pwmOptsPtr_1->Duty_Cycle_OC2_Add(pwmParaPtr_1, 10);
	printf("key1\r\n");
}

void key2Opt(void)
{
	//pwmOptsPtr_1->Duty_Cycle_OC1_Sub(pwmParaPtr_1, 10);
	//pwmOptsPtr_1->Duty_Cycle_OC2_Sub(pwmParaPtr_1, 10);
	printf("key2\r\n");
}

void key3Opt(void)
{
	/*
	static u8 enFlag = 0;
	
	printf("enFlag = %d\r\n", enFlag);
	if(0 == enFlag)
	{
		
		MOTOR_RIGHT_EN = 0;
		MOTOR_RIGHT_BK = 1;
		
		MOTOR_LEFT_EN = 0;
		MOTOR_LEFT_FR = 0;
		MOTOR_LEFT_BK = 1;
		
		enFlag = 1;
	}
	else
	{
		
		MOTOR_RIGHT_EN = 1;
		MOTOR_RIGHT_BK = 0;
		
		MOTOR_LEFT_EN = 1;
		MOTOR_LEFT_FR = 1;
		MOTOR_LEFT_BK = 0;
		
		enFlag = 0;
	}
	*/
	//pwmOptsPtr_1->Duty_Cycle_OC2_Add(pwmParaPtr_1, 10);
	printf("key3\r\n");
	
}

void key4Opt(void)
{
	//pwmOptsPtr_1->Duty_Cycle_OC2_Sub(pwmParaPtr_1, 10);
	//printf("key4\r\n");
	NRF24L01OptsPtr->TEST_Send();
}

void key1Opt_cepark(void)
{
	//pwmOptsPtr_1->Duty_Cycle_OC1_Add(pwmParaPtr_1, 10);
	//pwmOptsPtr_1->Duty_Cycle_OC2_Add(pwmParaPtr_1, 10);
	printf("key1\r\n");
	NRF24L01OptsPtr->nrf_send_up();
}

void key2Opt_cepark(void)
{
	//pwmOptsPtr_1->Duty_Cycle_OC1_Sub(pwmParaPtr_1, 10);
	//pwmOptsPtr_1->Duty_Cycle_OC2_Sub(pwmParaPtr_1, 10);
	printf("key2\r\n");
	NRF24L01OptsPtr->nrf_send_down();
}

void key3Opt_cepark(void)
{
	/*
	static u8 enFlag = 0;
	
	printf("enFlag = %d\r\n", enFlag);
	if(0 == enFlag)
	{
		
		MOTOR_RIGHT_EN = 0;
		MOTOR_RIGHT_BK = 1;
		
		MOTOR_LEFT_EN = 0;
		MOTOR_LEFT_FR = 0;
		MOTOR_LEFT_BK = 1;
		
		enFlag = 1;
	}
	else
	{
		
		MOTOR_RIGHT_EN = 1;
		MOTOR_RIGHT_BK = 0;
		
		MOTOR_LEFT_EN = 1;
		MOTOR_LEFT_FR = 1;
		MOTOR_LEFT_BK = 0;
		
		enFlag = 0;
	}
	*/
	//pwmOptsPtr_1->Duty_Cycle_OC2_Add(pwmParaPtr_1, 10);
	printf("key3\r\n");
	NRF24L01OptsPtr->nrf_send_left();
}

void key4Opt_cepark(void)
{
	//pwmOptsPtr_1->Duty_Cycle_OC2_Sub(pwmParaPtr_1, 10);
	printf("key4\r\n");
	NRF24L01OptsPtr->nrf_send_right();
	//NRF24L01OptsPtr->TEST_Send();
}

void KeyScanState_Check_Opt_cepark(int *scaningKey)
{
	if((0x0007 != (GPIOA->IDR & 0x0007)) || (0x2000 != (GPIOC->IDR & 0x2000)))	// 如果PA0 PA1 PA2有按键按下
	//if(0x0001 != (GPIOA->IDR & 0x0001))
	{
		keyScanStatus = KeyScanState_MakeSure;
	}
}

void KeyScanState_MakeSure_Opt_cepark(int *scaningKey)
{
	if(0x0007 != (GPIOA->IDR & 0x0007))
	{
		switch(GPIOA->IDR & 0x0007)
		{
			case 0x06:
				*scaningKey = KEY1;
				break;

			case 0x05:
				*scaningKey = KEY2;
				break;

			case 0x03:
				*scaningKey = KEY3;
				break;
		}
		
		keyScanStatus = KeyScanState_Up;
	}
	else if(0x2000 != (GPIOC->IDR & 0x2000))
	{
		*scaningKey = KEY4;
		keyScanStatus = KeyScanState_Up;
	}
}

void KeyScanState_Up_Opt_cepark(int *scaningKey)
{
	if((0x0007 == (GPIOA->IDR & 0x0007)) && (0x2000 == (GPIOC->IDR & 0x2000)))
	//if(0x0000 != (GPIOA->IDR & 0x0001))
	{
		//printf("up\r\n");
		keyScanStatus = KeyScanState_Check;
	}
}



void keyEvent(int key)
{
	if((key >= 0) && (key < KEY_NUM))
	{
		keyEventOpts[key]();
	}
	//printf("no\r\n");
}

void keyGPIO_CFG(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*设置GPIOA.2和GPIOA.3为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void zidian_keyInit(void)
{
	keyEventOpts[0] = key1Opt;
	keyEventOpts[1] = key2Opt;
	keyEventOpts[2] = key3Opt;
	keyEventOpts[3] = key4Opt;

	keyStatusEventPtr[KeyScanState_Check] = KeyScanState_Check_Opt;
	keyStatusEventPtr[KeyScanState_MakeSure] = KeyScanState_MakeSure_Opt;
	keyStatusEventPtr[KeyScanState_Up] = KeyScanState_Up_Opt;
	
	keyGPIO_CFG();
}

void cepark_keyInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*设置GPIOA.2和GPIOA.3为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	keyEventOpts[0] = key1Opt_cepark;
	keyEventOpts[1] = key2Opt_cepark;
	keyEventOpts[2] = key3Opt_cepark;
	keyEventOpts[3] = key4Opt_cepark;

	keyStatusEventPtr[KeyScanState_Check] = KeyScanState_Check_Opt_cepark;
	keyStatusEventPtr[KeyScanState_MakeSure] = KeyScanState_MakeSure_Opt_cepark;
	keyStatusEventPtr[KeyScanState_Up] = KeyScanState_Up_Opt_cepark;
}

void keyConfig(void)
{
	zidian_keyInit();
	//cepark_keyInit();
}





/***********************old key opts**************************/

void send(void)
{
	SendChar_USART3(0x2E);
	SendChar_USART3(0x83);
	SendChar_USART3(0x02);
	SendChar_USART3(0x11);
	SendChar_USART3(0x06);
}

void send2(void)
{
	SendChar_USART3(0x2E);
	SendChar_USART3(0x83);
	SendChar_USART3(0x02);
	SendChar_USART3(0x11);
	SendChar_USART3(0x07);
}



