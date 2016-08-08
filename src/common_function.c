#include "common_include.h"
#include "buffer.h"

vu32 SystemRunningTime = 0x00000000;

/*******************************************************************************
* 函数名  		: fputc
* 函数描述    	: 将printf函数重定位到USATR1
* 输入参数    	: 无
* 输出结果    	: 无
* 返回值		: 无
*******************************************************************************/
//注意将MDK中的Target Options->Target->Code Generation->Use MicroLIB选项勾上

int fputc(int ch, FILE *f)
{
	USART_SendData(UART4, (u16)ch);
	//while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	while (!(UART4->SR & USART_FLAG_TXE));
	return ch;
}



