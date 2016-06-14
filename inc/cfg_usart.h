#ifndef __CFG_USART_H__
#define __CFG_USART_H__

#include "common_include.h"

#define IS_USART_CMD_VALID_CHAR(CMD_CHAR)	(	(((u8)CMD_CHAR >= '0') && ((u8)CMD_CHAR <= '9')) || \
									   			(((u8)CMD_CHAR >= 'A') && ((u8)CMD_CHAR <= 'Z')) || \
									   			(((u8)CMD_CHAR >= 'a') && ((u8)CMD_CHAR <= 'z')) || \
									   			((u8)CMD_CHAR == ' ')	)

#define IS_USART_CMD_OVER_CHAR(CMD_CHAR)	(	((u8)CMD_CHAR == '\0') || \
									   			((u8)CMD_CHAR == '\r') || \
									   			((u8)CMD_CHAR == '\n')	)

#define ACK 			0xFF
#define NACK_CS_NG		0XF0
#define NACK_NS			0XF3
#define NACK_BUSY		0xFC


#define USARTx_EXIST_DATA(USARTx)	(((USARTx)->SR >> 5) & 0x0001)
#define USARTx_RECV_DATA(USARTx)	((u8)((USARTx)->DR & (u8)0xFF))

#define USART1_EXIST_DATA 			(USARTx_EXIST_DATA(USART1))
#define USART1_RECV_DATA			(USARTx_RECV_DATA(USART1))

#define USART2_EXIST_DATA 			(USARTx_EXIST_DATA(USART2))
#define USART2_RECV_DATA			(USARTx_RECV_DATA(USART2))

#define USART3_EXIST_DATA 			(USARTx_EXIST_DATA(USART3))
#define USART3_RECV_DATA			(USARTx_RECV_DATA(USART3))

#define USART1_SEND_DATA(Data)		(USART1->DR = (Data & (u16)0x01FF))
#define USART1_TC					((USART1->SR >> 6) & 0x0001)
#define USART1_TXE					(USART1->SR & USART_FLAG_TXE)

#define USART1_ACK					(USART1_SEND_DATA(ACK))
#define USART1_NCAK_CS_NG			(USART1_SEND_DATA(NACK_CS_NG))
#define USART1_NACK_NS				(USART1_SEND_DATA(NACK_NS))
#define USART1_NACK_BUSY			(USART1_SEND_DATA(NACK_BUSY))


void CB_USART_Config(void);
void cb_usart_test(void);
char* cb_usart_receive_string(char []);
u8 cb_usart_receive_char(void);
int SendChar(u8 ch);
int SendChar_USART3(u8 ch);


#endif



