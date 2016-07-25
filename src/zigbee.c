#include "zigbee.h"
#include "cfg_gpio.h"
#include "buffer.h"
#include "timer_opts.h"

#define USART2_TC					((USART2->SR >> 6) & 0x0001)

Zigbee_Info Zigbee;
Zigbee_Info_P Zigbee_Ptr = &Zigbee;

RecvCmdFlag CMD_Flag;
RecvCmdFlag_P CMD_Flag_Ptr = &CMD_Flag;

/////////////////uart波特率115200，八位数据，一位停止位
///////变量声明//////////////
u8 receive_state=0;		//接收完成标志
u8 receive_count=0;		//接收数据计数
u8 i;									//循环体变量
u8 flag1=0;						//记录串口接收字节

ZigbeeID_Info Id_Arr[11]; 

u8 nc_send1[8]=
	{0xfe, 0x08, 0x8c, 0xfe, 0x01, 0x02, 0x7F, 0x01};//小车已取物请龙门准备
u8 nc_send2[8]=
	{0xfe, 0x08, 0x8c, 0xfe, 0x01, 0x02, 0x7F, 0x02};//小车到达龙门请龙门取物
u8 nc_send3[8]=
	{0xfe, 0x08, 0x8c, 0xfe, 0x01, 0x02, 0x7F, 0x03};//小车离开龙门
u8 nc_send4[8]=	// 1号
	{0xfe, 0x08, 0x71, 0xdc, 0x01, 0x02, 0x7F, 0x01};//小车已取物，发送给按钮使其停止亮灯
u8 nc_send5[8]=	// 2号
	{0xfe, 0x08, 0x76, 0x3f, 0x01, 0x02, 0x7F, 0x01};//小车已取物，发送给按钮使其停止亮灯
u8 nc_send6[8]= // 3号
	{0xfe, 0x08, 0x54, 0xd7, 0x01, 0x02, 0x7F, 0x01};//小车已取物，发送给按钮使其停止亮灯
u8 nc_send7[8]= // 4号
	{0xfe, 0x08, 0xa4, 0x52, 0x01, 0x02, 0x7F, 0x01};//小车已取物，发送给按钮使其停止亮灯

u8 zigbeeAck[8] = {0xfe, 0x08, 0x00, 0x00, 0x01, 0x02, 0x8f, 0xff};

u8 nc_receive[8];		//接收数据缓存


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

#if 1

u8 SendChar_Zigbee(u8 ch)  //发送单个数据 
{
	//u16 timeout = U16_MAX;
	u32 timeout = 0;
	u32 cir = 0;
	USART_SendData(USART2, ch);
	
	#if 0
	
	//while (!(USART2->SR & USART_FLAG_TXE));
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//等待串口发送完成
	
	#else
	
	timeout = SystemRunningTime;
	while(!(USART2->SR & USART_FLAG_TXE))
	{
		if(SystemRunningTime - timeout > 100)
		{
			printf("timeout\r\n");
			break;
		}
	}

	#endif
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

#if 0
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

#else

void send_cmd(u8 *cmd)
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

void send_N_char(u8 *cmd, u8 num)
{
	u8 cir = 0;
	
	for(cir = 0; cir < num; cir++)
	{
		SendChar_Zigbee(cmd[cir]);
	}
}

#endif

void Send_Zigbee_ACK(void)
{
	u8 cir = 8;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_Zigbee_ACK\r\n");
		}
		
		SendChar_Zigbee(zigbeeAck[cir]);
		
	}
	
}


void UART2_REC_IRQ(u8 UART2_DR)//串口接收中断函数
{
	nc_receive[receive_count] = UART2_DR;
	
	receive_count++;
	
	if(receive_count == 2)
	{
		if(nc_receive[1] == 0x07)
		{
			flag1 = 7;
		}
		
		if(nc_receive[1] == 0x08)
		{
			flag1 = 8;
		}
	}
	
	if(flag1==7)
	{
		if(receive_count == 7)
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			
			receive_count=0;
		}
	}
	
	if(flag1 == 8)
	{
		if(receive_count == 8)
		{
			receive_count = 0;
			receive_state = 1;
		}
	}
}


void Receive_handle(void)
{
	if(receive_state == 1)
	{
		receive_state = 0;
		if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x01))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			///终端请求取物////
			zigbeeRecvDataBuf_Append(0x0001);
			printf("0x0001\r\n");
			//zigbeeAck[2] = Id_Arr[1].zigbee_ID1;
			//zigbeeAck[3] = Id_Arr[1].zigbee_ID2;
			zigbeeAck[2] = 0x71;
			zigbeeAck[3] = 0xdc;
			Send_Zigbee_ACK();
			CMD_Flag_Ptr->cmdFlag = GoodReq;
			//Delay_ns(1);
			//Send_GettedGoods(1);
			/////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x11))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			///终端请求取物////
			zigbeeRecvDataBuf_Append(0x0002);
			zigbeeAck[2] = Id_Arr[2].zigbee_ID1;
			zigbeeAck[3] = Id_Arr[2].zigbee_ID2;
			Send_Zigbee_ACK();
			printf("0x0002\r\n");
			Delay_ns(1);
			Send_GettedGoods(2);
			CMD_Flag_Ptr->cmdFlag = GoodReq;
			/////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x21))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			///终端请求取物////
			zigbeeRecvDataBuf_Append(0x0003);
			zigbeeAck[2] = Id_Arr[3].zigbee_ID1;
			zigbeeAck[3] = Id_Arr[3].zigbee_ID2;
			Send_Zigbee_ACK();
			printf("0x0003\r\n");
			Delay_ns(1);
			Send_GettedGoods(3);
			CMD_Flag_Ptr->cmdFlag = GoodReq;
			/////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x31))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			///终端请求取物////
			zigbeeRecvDataBuf_Append(0x0004);
			zigbeeAck[2] = Id_Arr[4].zigbee_ID1;
			zigbeeAck[3] = Id_Arr[4].zigbee_ID2;
			Send_Zigbee_ACK();
			printf("0x0003\r\n");
			Delay_ns(1);
			Send_GettedGoods(3);
			CMD_Flag_Ptr->cmdFlag = GoodReq;
			/////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x02))
		{ 
			for(i=0;i<8;i++)
			{
				nc_receive[i] = 0x00;
			}
			///////终端请求停止取物/////////
			CMD_Flag_Ptr->cmdFlag = GoodReqCancel;	
			//////////////////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x12))
		{ 
			for(i=0;i<8;i++)
			{
				nc_receive[i] = 0x00;
			}
			///////终端请求停止取物/////////
			CMD_Flag_Ptr->cmdFlag = GoodReqCancel;
			//////////////////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x22))
		{ 
			for(i=0;i<8;i++)
			{
				nc_receive[i] = 0x00;
			}
			///////终端请求停止取物/////////
			CMD_Flag_Ptr->cmdFlag = GoodReqCancel;
			//////////////////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0xc1))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			zigbeeRecvDataBuf_Append(0x0000);
			zigbeeAck[2] = Id_Arr[0].zigbee_ID1;
			zigbeeAck[3] = Id_Arr[0].zigbee_ID2;
			Send_Zigbee_ACK();
			/////龙门已经将货物取走//////
			CMD_Flag_Ptr->cmdFlag = GoodLeav;
			////////////////////////////
		}
		else if((nc_receive[6] == 0x8f) && (nc_receive[7] == 0xff))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			///ACK////
			printf("recv ACK\r\n");
			CMD_Flag_Ptr->cmdFlag = ZigbeeACK;
			/////////////////
		}
		
	}
}

void Send_GettedGoods(u8 node)
{
	u8 cir = 8, flag = 0, flag1 = 0;
	u32 timRec = 0;
	
	switch(node)
	{
		case 1:
			nc_send4[2] = Id_Arr[1].zigbee_ID1;
			nc_send4[3] = Id_Arr[1].zigbee_ID2;
			break;

		case 2:
			nc_send4[2] = Id_Arr[2].zigbee_ID1;
			nc_send4[3] = Id_Arr[2].zigbee_ID2;
			break;

		case 3:
			nc_send4[2] = Id_Arr[3].zigbee_ID1;
			nc_send4[3] = Id_Arr[3].zigbee_ID2;
			break;

		default:
			printf("error\r\n");
			break;
	}

	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_GettedGoods\r\n");
		}
		SendChar_Zigbee(nc_send4[cir]);
		flag = 1;
		timRec = SystemRunningTime;
	}

	if(1 == flag)
	{
		while(SystemRunningTime - timRec < 1000)
		{
			if(ZigbeeACK == CMD_Flag_Ptr->cmdFlag)
			{
				flag1 = 1;
				break;
			}
		}

		if(1 == flag1)
		{
			for(cir = 0; cir < 8; cir++)
			{
				if(0 == cir)
				{
					printf("Send_GettedGoods\r\n");
				}
				SendChar_Zigbee(nc_send4[cir]);
			}
		}
		
	}
	
}

void Send_WaitForGoods(void)
{
	u8 cir = 8;
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_WaitForGoods\r\n");
		}
		SendChar_Zigbee(nc_send1[cir]);
		
	}
	
}




void Send_Arrive(void)
{
	u8 cir = 8;
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_Arrive\r\n");
		}
		
		SendChar_Zigbee(nc_send2[cir]);
		
	}
	
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
			//printf("7ferror\r\n");
		}
		
	}
	
}

void Zigbee_Usart_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 开启GPIO时钟和复用功能时钟RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* 配置 USART Tx 复用推挽输出 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置 USART Rx 浮空输入 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
}



void Zigbee_Usart_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;

	Zigbee_Usart_GPIOInit();

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;		//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);								//初始化

	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART2, &USART_ClockInitStructure);
	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
}






void Zigbee_Init(void)
{
	Zigbee_Usart_Init();
	
	Zigbee_Ptr->receive_end = 0;
	Zigbee_Ptr->recvValidDataFlag = 0;
	Zigbee_Ptr->recvId = 0x0000;

	CMD_Flag_Ptr->cmdFlag = NcNone;

	// 龙门
	Id_Arr[0].zigbee_ID1 = 0x8c;
	Id_Arr[0].zigbee_ID2 = 0xfe;
	
	Id_Arr[1].zigbee_ID1 = 0x71;
	Id_Arr[1].zigbee_ID2 = 0xdc;

	Id_Arr[2].zigbee_ID1 = 0x76;
	Id_Arr[2].zigbee_ID2 = 0x3f;

	Id_Arr[3].zigbee_ID1 = 0x54;
	Id_Arr[3].zigbee_ID2 = 0xd7;

	Id_Arr[4].zigbee_ID1 = 0xa4;
	Id_Arr[4].zigbee_ID2 = 0x52;

	
}


