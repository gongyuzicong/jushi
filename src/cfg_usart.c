#include "cfg_usart.h"
#include "nvic_opts.h"
#include <string.h>
#include "buffer.h"

#define USE_USART1
//#define USE_USART3

#if 0
int SendChar(u8 ch)  //���͵������� 
{
	USART_SendData(USART1, ch); 
	while (!(USART1->SR & USART_FLAG_TXE)); 
	return (ch); 
} 
#else
int SendChar(u8 ch)  //���͵������� 
{
	vu32 counter = 0x00;
	//USART_SendData(USART1, ch); 
	USART1->DR = (ch & (u16)0x01FF);
#if 0
	counter = SystemRunningTime;
	while (!(USART1->SR & USART_FLAG_TXE))
	{
		if(10 == (SystemRunningTime - counter))		// ��ʱ10ms
		{
			break;
		}
		else
		{
			//RecvCanAndUsartFunc();
		}
	}
#else
	counter = SystemRunningTime;
	while(!USART1_TC)
	{
		if((SystemRunningTime - counter) >= 10)		// ��ʱ10ms
		{
			break;
		}
		else
		{
			//RecvCanAndUsartFunc();
		}
	}
#endif
	return (ch); 
}

#endif


int SendChar_USART3(u8 ch)  //���͵������� 
{
	USART_SendData(USART3, ch); 
	while (!(USART3->SR & USART_FLAG_TXE)); 
	return (ch); 
} 


void Print_String(u8 *p)  //����һ������ 
{ 
	while(*p) 
	{ 
		USART_SendData(USART1, *p++); 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	} 
} 

int cb_usart_send_char(u8 ch)//���͵������� 
{
	USART_SendData(USART1, ch);
	while (!(USART1->SR & USART_FLAG_TXE));
	return 0;
}

int cb_usart_send_string(char *string)//����һ������ 
{
	int cir;

	for(cir = 0; cir < strlen(string); cir++)
	{
		cb_usart_send_char((u8)*(string + cir));
	}
	return 0;
}

u8 cb_usart_receive_char(void)
{
	u8 ret;
#if 0
	while(1)
	{
		if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
		{
			ret = USART_ReceiveData(USART1);
			break;
		}
	}
#else
	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != SET);
	ret = USART_ReceiveData(USART1);
#endif
	return ret;
}



char* cb_usart_receive_string(char getString[])
{
	
	int cir;
	u8 recv;
	
	for(cir = 0; cir < 30; cir++)
	{
		recv = cb_usart_receive_char();
		
		if(IS_USART_CMD_OVER_CHAR(recv))
		{
			(*(getString + cir)) = '\0';
			break;
		}
		else
		{
			(*(getString + cir)) = (char)recv;
		}
	
	}
	//USART_ClearFlag(USART1, USART_FLAG_RXNE);
	return (getString);

}


//////////////////////////////usart���Գ���////////////////////////////////
void cb_usart_test(void)
{
	while(1)
	{
		if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
		{
			//printf("%d\r\n", USART_ReceiveData(USART1));
			USART_SendData(USART1 , USART_ReceiveData(USART1));
			//for(cir = 0; cir < 500; cir++);
		}
	}
}


void Debug_Uart_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ����GPIOʱ�Ӻ͸��ù���ʱ��RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* ���� USART Tx ����������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ���� USART Rx �������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
}


void Debug_UART_INIT(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	Debug_Uart_GPIOInit();
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);								//��ʼ��

	/*
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(UART4, &USART_ClockInitStructure);
	*/
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_Init(UART4, &USART_InitStructure);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART4, ENABLE);
	USART_ClearFlag(UART4, USART_FLAG_TC);
}



void CB_USART_Config(void)
{
	Debug_UART_INIT();
	
}



