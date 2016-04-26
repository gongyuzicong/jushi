#include "cfg_usart.h"
#include "nvic_opts.h"
#include <string.h>
#include "buffer.h"

#define USE_USART1
#define USE_USART3

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











void CB_USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//Nvic_Paramater node;

	//node.Nvic_PreemptionPriority = 1;
	//node.Nvic_SubPriority = 0;
	//node.Nvic_Group = 2;
	//node.Nvic_Channel = USART1_IRQChannel;
	
	//USART_ClockInitTypeDef USART_ClockInitStructure;

#ifdef USE_USART1
	/***USART3 ��ʼ�� BEGIN***/
	/*���ô���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);								//��ʼ��

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	/***USART1 ��ʼ�� END***/
#endif

#ifdef USE_USART3
	/***USART3 ��ʼ�� BEGIN***/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	USART_ClockInit(USART3, &USART_ClockInitStructure);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;

	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	/***USART3 ��ʼ�� END***/
#endif

	//USART1->CR1 |= (1 << 8);	// PE(У�����)�ж�ʹ��
	//USART1->CR1 |= (1 << 5);	// RXNE(���ջ�����)�ǿ��ж�ʹ��
	//My_Nvic_Init(node);

#ifdef USE_TC
	
	//USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
#endif
	
}



