#include "common_include.h"
#include "buffer.h"

vu32 SystemRunningTime = 0x00000000;

/*******************************************************************************
* ������  		: fputc
* ��������    	: ��printf�����ض�λ��USATR1
* �������    	: ��
* ������    	: ��
* ����ֵ		: ��
*******************************************************************************/
//ע�⽫MDK�е�Target Options->Target->Code Generation->Use MicroLIBѡ���

int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (u16)ch);
	//while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	while (!(USART1->SR & USART_FLAG_TXE));
	return ch;
}



