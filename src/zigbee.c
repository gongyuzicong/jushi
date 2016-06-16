#include "zigbee.h"

Zigbee_Info Zigbee;
Zigbee_Info_P Zigbee_Ptr = &Zigbee;

#if 0
/******************发送命令****************/
u8 ZD_send_1[8]  = {0xfe,0x08,0x00,0x00,0x01,0x02,0x7F,0x01};	//短按，呼叫
u8 ZD_send_2[8]  = {0xfe,0x08,0x00,0x00,0x01,0x02,0x7F,0x02};	//长按，取消
u8 ZD_send_3[15] = {0xfe,0x0e,0x00,0x00,0x01,0x02,0x7F,0x03,
                          0x00,0x00,0x05,0x17,0x00,0x00,0x00};	//发送时间
#endif

u8 NC_send_1[8] = {0xfe, 0x08, 0x58, 0xd3, 0x01, 0x02, 0x7F, 0x01};		//led off
#define USART2_TC					((USART2->SR >> 6) & 0x0001)


void Protocol_analysis(u8 rec_dat)
{
  u8 pres; 
  
  static u8 receive_num = 0x00;
  static u8 receive_flag = 0x00;

  
  pres  =       rec_dat;
  
  
  switch(receive_flag)
  {
    case 0x00:   
		if(pres == 0xFE) //帧头
		{
			receive_flag = 0x01;
			Zigbee_Ptr->frm_1.startID = pres;
		}
		break;
		
    case 0x01:   
		receive_flag = 0x02; //长度
		Zigbee_Ptr->frm_1.length = pres; 
		break;
		
    case 0x02:    
		receive_flag =0x03; //ID0
		Zigbee_Ptr->frm_1.decID0 = pres;
		break;
		
    case 0x03:    
		receive_flag = 0x04;//ID1
		Zigbee_Ptr->frm_1.decID1 = pres;
		break;
		
    case 0x04:    
		receive_flag = 0x05;//cmd1
		Zigbee_Ptr->frm_1.cmd1 = pres;
		break;
		
    case 0x05:    
		receive_flag = 0x06;//cmd2
		Zigbee_Ptr->frm_1.cmd2 = pres;
		break;
		
    case 0x06:    
		Zigbee_Ptr->frm_1.buf[receive_num] = pres;//data
		receive_num++;
		if(receive_num == (Zigbee_Ptr->frm_1.length-6))
		{
			receive_flag = 0x00;
			receive_num = 0x00;
			Zigbee_Ptr->receive_end = 0x01;
		}
		break;
   }
  
}

#if 0

u8 SendChar_Zigbee(u8 ch)  //发送单个数据 
{
	USART_SendData(USART2, ch); 
	while (!(USART2->SR & USART_FLAG_TXE));
	return (ch); 
} 

#else

u8 SendChar_Zigbee(u8 ch)  //发送单个数据 
{
	vu32 counter = 0x00;
	//USART_SendData(USART1, ch); 
	USART2->DR = (ch & (u16)0x01FF);
#if 0
	counter = SystemRunningTime;
	while (!(USART1->SR & USART_FLAG_TXE))
	{
		if(10 == (SystemRunningTime - counter))		// 超时10ms
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
	while(!USART2_TC)
	{
		if((SystemRunningTime - counter) >= 100)		// 超时10ms
		{
			printf("error!\r\n");
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


void send_cmd(u8 cmd[])
{
  u8 i;
  frmFmt *ptr;
  
  ptr = (frmFmt *)cmd;
  
  if(SendChar_Zigbee(ptr->startID) == ptr->startID)
  {
  	printf("%d\r\n", ptr->startID);
  }
  
  SendChar_Zigbee(ptr->length);
  SendChar_Zigbee(ptr->decID0);
  SendChar_Zigbee(ptr->decID1);
  SendChar_Zigbee(ptr->cmd1);
  SendChar_Zigbee(ptr->cmd2);
  
  for(i = 0; i < ptr->length - 6; i++)
 	SendChar_Zigbee(ptr->buf[i]);  
  
}


void Zigbee_Usart_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/***USART3 初始化 BEGIN***/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;		//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);								//初始化
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART2, &USART_ClockInitStructure);
	
	USART_ClockInit(USART2, &USART_ClockInitStructure);
	USART_InitStructure.USART_BaudRate = 19200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;

	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
	
	USART2->CR1 |= (1 << 5);	// RXNE(接收缓冲区)非空中断使能
	/***USART3 初始化 END***/
}


void Zigbee_Data_Scan(void)
{
	if(1 == Zigbee_Ptr->receive_end)
	{
		if(0x7F == Zigbee_Ptr->frm_1.buf[0])
		{
			Zigbee_Ptr->receive_end = 0;
			
			Zigbee_Ptr->recvId = (Zigbee_Ptr->frm_1.decID1 << 8) | Zigbee_Ptr->frm_1.decID0;
			//printf("decID0 = %02x, decID1 = %02x\r\n", Zigbee_Ptr->frm_1.decID0, Zigbee_Ptr->frm_1.decID1);
			//printf("buf[0] = %02x, buf[1] = %02x\r\n", Zigbee_Ptr->frm_1.buf[0], Zigbee_Ptr->frm_1.buf[1]);
			printf("recvId = %04x\r\n", Zigbee_Ptr->recvId);
			//printf("\r\n");
			
			Zigbee_Ptr->recvValidDataFlag = 1;
			
		}
		else
		{
			printf("7ferror\r\n");
		}
		
	}
	
}


void Zigbee_Init(void)
{
	Zigbee_Usart_Init();
	
	Zigbee_Ptr->receive_end = 0;
	Zigbee_Ptr->recvValidDataFlag = 0;
	Zigbee_Ptr->recvId = 0x0002;
}


