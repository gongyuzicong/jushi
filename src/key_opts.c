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
u8 keyScanFlag = 0;
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

void keyConfig(void)
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

void Key_1(void)
{
	//u8 time = 100;
	static u8 flag = 1;
	CmdRecvFromFYT b;
	if((GPIOA->IDR & 0x00000002) == 0)
	{
		Delay_ms(10);
		if((GPIOA->IDR & 0x00000002) == 0)
		{
			if(1 == flag)
			{
				//GPIOA_X_ON(4);
				//GPIOA->BSRR |= (1 << 4);
				//showSimuBufData(simuDataBufHeadNode, simuDataBufTailNode);
				//send();
				//CanOptsPtr->YL_KeySendCan();
				//SendLoopTestData2();
				b.dataType = 0x01;
				b.dataLength = 0x02;
				b.data[0] = 0x01;
				b.data[1] = 0x02;
				b.checkSum = 0x01;
				FytBuf_Append(b);
				flag = 0;
				//printf("flag1 = %d\r\n", flag);
			}
			else
			{
				//GPIOA_X_OFF(4);
				//GPIOA->BSRR |= (1 << 20);
				//showSimuBufData(simuDataBufHeadNode, simuDataBufTailNode);
				//send2();
				//SendLoopTestData3();
				b.dataType = 0x02;
				b.dataLength = 0x02;
				b.data[0] = 0x01;
				b.data[1] = 0x02;
				b.checkSum = 0x01;
				FytBuf_Append(b);
				flag = 1;
				//printf("flag2 = %d\r\n", flag);
			}
			while((GPIOA->IDR & 0x00000002) == 0);
			//time = 100;
			Delay_ms(10);
			while((GPIOA->IDR & 0x00000002) == 0);
		}
	}
}


void Key_3(void)
{
	//u8 time = 100;
	static u8 flag = 1;
	
	if((GPIOA->IDR & 0x00000004) == 0)
	{
		Delay_ms(10);
		if((GPIOA->IDR & 0x00000004) == 0)
		{
			if(1 == flag)
			{
				//GPIOA_X_ON(4);
				//GPIOA->BSRR |= (1 << 4);
				//showSimuBufData(simuDataBufHeadNode, simuDataBufTailNode);
				//send();
				//CanOptsPtr->YL_KeySendCan();
				//SendLoopTestData2();
				FytBuf_Delete();
				flag = 0;
				//printf("flag1 = %d\r\n", flag);
			}
			else
			{
				//GPIOA_X_OFF(4);
				//GPIOA->BSRR |= (1 << 20);
				//showSimuBufData(simuDataBufHeadNode, simuDataBufTailNode);
				//send2();
				//SendLoopTestData3();
				FytBuf_Delete();
				flag = 1;
				//printf("flag2 = %d\r\n", flag);
			}
			while((GPIOA->IDR & 0x00000004) == 0);
			//time = 100;
			Delay_ms(10);
			while((GPIOA->IDR & 0x00000004) == 0);
		}
	}
}


