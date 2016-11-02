#include "zigbee.h"
#include "cfg_gpio.h"
#include "buffer.h"
#include "timer_opts.h"
#include "motion_control.h"
#include "circle_recoder.h"
#include "magn_sensor.h"
#include "led.h"

#define USART2_TC					((USART2->SR >> 6) & 0x0001)

Zigbee_Info Zigbee;
Zigbee_Info_P Zigbee_Ptr = &Zigbee;

RecvCmdFlag CMD_Flag;
RecvCmdFlag_P CMD_Flag_Ptr = &CMD_Flag;

Zigbee_ACK_Info ZigbeeResendInfo;
Zigbee_ACK_Info_P ZigbeeResendInfo_Ptr = &ZigbeeResendInfo;

ZigbeeID_Info Id_Arr[12]; 

#define WaitForGoods()			{Zigbee_Ptr->ZigbeeSendCmdData[6] = 0x7F; Zigbee_Ptr->ZigbeeSendCmdData[7] = 0x01;}
#define AgvArrive()				{Zigbee_Ptr->ZigbeeSendCmdData[6] = 0x7F; Zigbee_Ptr->ZigbeeSendCmdData[7] = 0x02;}
#define AgvAbnormal()			{Zigbee_Ptr->ZigbeeSendCmdData[6] = 0x7F; Zigbee_Ptr->ZigbeeSendCmdData[7] = 0x03;}
#define GetGoods()				{Zigbee_Ptr->ZigbeeSendCmdData[6] = 0x7F; Zigbee_Ptr->ZigbeeSendCmdData[7] = 0x01;}
#define MachineAutoRun()		{Zigbee_Ptr->ZigbeeSendCmdData[6] = 0x7F; Zigbee_Ptr->ZigbeeSendCmdData[7] = 0x05;}
#define Ack()					{Zigbee_Ptr->ZigbeeSendCmdData[6] = 0x8F; Zigbee_Ptr->ZigbeeSendCmdData[7] = 0xFF;}

/*
u8 nc_send1[8]=
	{0xfe, 0x08, 0x73, 0xD7, 0x01, 0x02, 0x7F, 0x01};//小车已取物请龙门准备
u8 nc_send2[8]=
	{0xfe, 0x08, 0x73, 0xD7, 0x01, 0x02, 0x7F, 0x02};//小车到达龙门请龙门取物
u8 nc_send3[8]=
	{0xfe, 0x08, 0x8c, 0xfe, 0x01, 0x02, 0x7F, 0x03};//小车异常
u8 nc_send4[8]=
	{0xfe, 0x08, 0x71, 0xdc, 0x01, 0x02, 0x7F, 0x01};//小车已取物，发送给按钮使其停止亮灯
u8 nc_send5[8]=
	{0xfe, 0x08, 0x00, 0x00, 0x01, 0x02, 0x7F, 0x05};//小车告诉络纱机上线


u8 zigbeeAck[8] = {0xfe, 0x08, 0x00, 0x00, 0x01, 0x02, 0x8f, 0xff};
u8 zigbeeAck_LM[8] = {0xfe, 0x08, 0x00, 0x00, 0x01, 0x02, 0x8f, 0xff};
*/

#if 1

u8 SendChar_Zigbee(u8 ch)  //发送单个数据 
{
	//u16 timeout = U16_MAX;
	u32 timeout = 0;
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


void Send_Zigbee_ACK(u8 node)
{
	u8 cir = 8;

	
	Ack();
	Zigbee_Ptr->ZigbeeSendCmdData[2] = Id_Arr[node].zigbee_ID1;
	Zigbee_Ptr->ZigbeeSendCmdData[3] = Id_Arr[node].zigbee_ID2;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_Zigbee_ACK\r\n");
		}
		
		SendChar_Zigbee(Zigbee_Ptr->ZigbeeSendCmdData[cir]);
		ZigbeeResendInfo_Ptr->resendFlag = 0;
		//ZigbeeResendInfo_Ptr->resendInfo = zigbeeAck;
	}
	
}

void Send_Zigbee_LM_ACK(void)
{
	u8 cir = 8;

	Ack();
	Zigbee_Ptr->ZigbeeSendCmdData[2] = Id_Arr[11].zigbee_ID1;
	Zigbee_Ptr->ZigbeeSendCmdData[3] = Id_Arr[11].zigbee_ID2;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_Zigbee_LM_ACK\r\n");
		}
		
		SendChar_Zigbee(Zigbee_Ptr->ZigbeeSendCmdData[cir]);
		ZigbeeResendInfo_Ptr->resendFlag = 0;
		//ZigbeeResendInfo_Ptr->resendInfo = zigbeeAck_LM;
	}
	
}


void UART2_REC_IRQ(u8 UART2_DR)//串口接收中断函数
{
	zbDataRecvBufCtrl_Ptr->Append(UART2_DR);
}

void ZB_Data_Receive_handle(void)
{
	static u8 index = 0;
	
	if(zbDataRecvBufCtrl_Ptr->Total > 0)
	{
		u8 data = 0;

		data = zbDataRecvBufCtrl_Ptr->GetDataDelete();

		if(0 == index)
		{
			if(0xFE == data)
			{
				Zigbee_Ptr->ZigbeeRecvCmdData[index] = data;
				index = 1;
			}
			else
			{
				index = 0;
			}
			
		}
		else if(1 == index)
		{
			if(0x08 == data)
			{
				Zigbee_Ptr->ZigbeeRecvCmdData[index] = data;
				index = 2;
			}
			else
			{
				index = 0;
			}
		}
		else if(2 == index)
		{
			Zigbee_Ptr->ZigbeeRecvCmdData[index] = data;
			index = 3;
		}
		else if(3 == index)
		{
			Zigbee_Ptr->ZigbeeRecvCmdData[index] = data;
			index = 4;
		}
		else if(4 == index)
		{
			if(0x01 == data)
			{
				Zigbee_Ptr->ZigbeeRecvCmdData[index] = data;
				index = 5;
			}
			else
			{
				index = 0;
			}
		}
		else if(5 == index)
		{
			if(0x02 == data)
			{
				Zigbee_Ptr->ZigbeeRecvCmdData[index] = data;
				index = 6;
			}
			else
			{
				index = 0;
			}
		}
		else if(6 == index)
		{
			Zigbee_Ptr->ZigbeeRecvCmdData[index] = data;
			index = 7;
		}
		else if(7 == index)
		{
			Zigbee_Ptr->ZigbeeRecvCmdData[index] = data;
			index = 0;

			Zigbee_Ptr->ZigbeeRecvCmdUpdate = 1;
		}
		
	}
	
}

void ZB_Data_Analysis(void)
{
	if(1 == Zigbee_Ptr->ZigbeeRecvCmdUpdate)
	{
		u8 cir = 0;
		Zigbee_Ptr->ZigbeeRecvCmdUpdate = 0;
		
		if(0x7f == Zigbee_Ptr->ZigbeeRecvCmdData[6])
		{
			u8 node = 0;
			ReqQueueStr ReqInfo;
			u8 cir = 0;
			
			node = (Zigbee_Ptr->ZigbeeRecvCmdData[7] >> 4) + 1;
			
			//printf("ZB_Data_Analysis ZigbeeRecvCmdData[6] = %02x, ZigbeeRecvCmdData[7] = %02x\r\n", Zigbee_Ptr->ZigbeeRecvCmdData[6], Zigbee_Ptr->ZigbeeRecvCmdData[7]);
			if((0x01 == (Zigbee_Ptr->ZigbeeRecvCmdData[7] & 0x0f)) || (0x03 == (Zigbee_Ptr->ZigbeeRecvCmdData[7] & 0x0f)))	// 请求取物/自动呼叫
			{
				if(0xC1 == Zigbee_Ptr->ZigbeeRecvCmdData[7])
				{
					CMD_Flag_Ptr->cmdFlag = GoodLeav;
					Send_Zigbee_LM_ACK();
					
					WarningLedCtrlPtr->twinkleFlag = 1;
					printf("GoodLeav\r\n");
				}
				else
				{
					u8 index = 0;
					Send_Zigbee_ACK(node);
					if(0 == searchZigbeeData(node, &index))
					{
						u8 data = 0;
						
						#if USE_CIRCLE_INFO_RECODER
						//CircleInfoStrPtr->CircleRecoderCount++;
						//CircleInfoStrPtr->Station = node;
						//CircleInfoStrPtr->REQ_TIME = BackgroudRTC_Rec;
						#endif
						
						Id_Arr[node].zigbee_ID1 = Zigbee_Ptr->ZigbeeRecvCmdData[2];
						Id_Arr[node].zigbee_ID2 = Zigbee_Ptr->ZigbeeRecvCmdData[3];
						
						
						
						ReqInfo.Req_Station = node;
						data = (Zigbee_Ptr->ZigbeeRecvCmdData[7] & 0x0f);
						//printf("******* data = %02x ********\r\n", data);
						if(0x01 == data)
						{
							CMD_Flag_Ptr->cmdFlag = GoodReq;
							
							ReqInfo.Req_Type = TypeManuReq;
							printf("GoodReq recv = %d\r\n", ReqInfo.Req_Station);
						}
						else if(0x03 == data)
						{
							CMD_Flag_Ptr->cmdFlag = AutoReq;
							
							ReqInfo.Req_Type = TypeAutoReq;
							printf("AutoReq recv = %d\r\n", ReqInfo.Req_Station);
						}

						//zigbeeReqQueue(ReqInfo);
						zigbeeRecvDataBuf_Append(ReqInfo);
						Show_Queue_Data();
						for(cir = 0; cir < 8; cir++)
						{
							//printf("ZigbeeRecvCmdData[%d] = %x\r\n", cir, Zigbee_Ptr->ZigbeeRecvCmdData[cir]);
							Zigbee_Ptr->ZigbeeRecvCmdData[cir] = 0x00;
						}
						
						BuzzerCtrlPtr->buzzerFlag = 1;
						WarningLedCtrlPtr->twinkleFlag = 1;
					}				}
					
			}
			else if((0x02 == (Zigbee_Ptr->ZigbeeRecvCmdData[7] & 0x0f)) || (0x04 == (Zigbee_Ptr->ZigbeeRecvCmdData[7] & 0x0f)))		// 请求取消取物
			{
				if(1 == ctrlParasPtr->AutoCancel_Respond)
				{
					u8 index = 0;
					Send_Zigbee_ACK(node);
					if(1 == searchZigbeeData(node, &index))
					{
						if(0 == index)
						{
							//printf("1 Total = %d\r\n", zigbeeQueueCtrl.Total);
							CHANGE_TO_STOP_MODE();
							Show_Queue_Data();
							zigbeeDeleteQueue(index);
							Show_Queue_Data();
							//printf("2 Total = %d\r\n", zigbeeQueueCtrl.Total);
							printf("Good cancel node = %d, index = %d\r\n", node, index);
							
							ctrlParasPtr->walkingstep = step_origin;
							
							ctrlParasPtr->rifdAdaptFlag = 0;
							//printf("GoodReqCancel step_origin = %d\r\n", ctrlParasPtr->walkingstep);
							
							
							CMD_Flag_Ptr->Cancel_Flag = GoodReqCancel;
							
							BuzzerCtrlPtr->buzzerFlag = 1;
							//printf("@!@!@!@!@! cancel %04x\r\n", node);

							#if USE_CIRCLE_INFO_RECODER
							CircleInfoStrPtr->lock = 0;
							#endif
						}
						else
						{
							zigbeeDeleteQueue(index);
							Show_Queue_Data();
							//printf("2 Total = %d\r\n", zigbeeQueueCtrl.Total);
							printf("Good cancel node = %d, index = %d\r\n", node, index);
							BuzzerCtrlPtr->buzzerFlag = 1;
						}
						
					}
				}
				
				
			}
			
		}
		else if(0x8f == Zigbee_Ptr->ZigbeeRecvCmdData[6])
		{
			if(0xff == Zigbee_Ptr->ZigbeeRecvCmdData[7])
			{
				printf("recv ACK\r\n");
				CMD_Flag_Ptr->cmdFlag = ZigbeeACK;
			}
		}

		for(cir = 0; cir < 8; cir++)
		{
			Zigbee_Ptr->ZigbeeRecvCmdData[cir] = 0x00;
		}
		
	}
}

void Send_FiberMachine(void)
{
	u8 cir = 8;

	MachineAutoRun();
	Zigbee_Ptr->ZigbeeSendCmdData[2] = Id_Arr[Zigbee_Ptr->runningInfo.Req_Station].zigbee_ID1;
	Zigbee_Ptr->ZigbeeSendCmdData[3] = Id_Arr[Zigbee_Ptr->runningInfo.Req_Station].zigbee_ID2;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_FiberMachine\r\n");
		}
		//printf("Zigbee_Ptr->ZigbeeSendCmdData[%d] = %02x\r\n", cir , Zigbee_Ptr->ZigbeeSendCmdData[cir]);
		SendChar_Zigbee(Zigbee_Ptr->ZigbeeSendCmdData[cir]);
	}
	
	//printf("************ cmdFlag = %d ***********\r\n", CMD_Flag_Ptr->cmdFlag);
	CMD_Flag_Ptr->cmdFlag = NcNone;
	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = Zigbee_Ptr->ZigbeeSendCmdData;
	//ZigbeeResendInfo_Ptr->resendInfo = nc_send5;
}


void Send_GettedGoods(void)
{
	u8 cir = 8;

	GetGoods();
	Zigbee_Ptr->ZigbeeSendCmdData[2] = Id_Arr[Zigbee_Ptr->runningInfo.Req_Station].zigbee_ID1;
	Zigbee_Ptr->ZigbeeSendCmdData[3] = Id_Arr[Zigbee_Ptr->runningInfo.Req_Station].zigbee_ID2;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_GettedGoods\r\n");
		}
		
		//printf("Zigbee_Ptr->ZigbeeSendCmdData[%d] = %02x\r\n", cir, Zigbee_Ptr->ZigbeeSendCmdData[cir]);
		SendChar_Zigbee(Zigbee_Ptr->ZigbeeSendCmdData[cir]);
	}
	
	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = Zigbee_Ptr->ZigbeeSendCmdData;
	//ZigbeeResendInfo_Ptr->resendInfo = nc_send4;
}

void Send_WaitForGoods(void)
{
	u8 cir = 8;
	WaitForGoods();
	Zigbee_Ptr->ZigbeeSendCmdData[2] = Id_Arr[11].zigbee_ID1;
	Zigbee_Ptr->ZigbeeSendCmdData[3] = Id_Arr[11].zigbee_ID2;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_WaitForGoods\r\n");
		}
		
		SendChar_Zigbee(Zigbee_Ptr->ZigbeeSendCmdData[cir]);

		//printf("Zigbee_Ptr->ZigbeeSendCmdData[%d] = %02x\r\n", cir, Zigbee_Ptr->ZigbeeSendCmdData[cir]);
	}

	//printf("************cmdFlag = %d***********\r\n", CMD_Flag_Ptr->cmdFlag);
	CMD_Flag_Ptr->cmdFlag = NcNone;
	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = Zigbee_Ptr->ZigbeeSendCmdData;
	//ZigbeeResendInfo_Ptr->resendInfo = nc_send1;
}


void Send_Arrive(void)
{
	u8 cir = 8;

	AgvArrive();
	Zigbee_Ptr->ZigbeeSendCmdData[2] = Id_Arr[11].zigbee_ID1;
	Zigbee_Ptr->ZigbeeSendCmdData[3] = Id_Arr[11].zigbee_ID2;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_Arrive\r\n");
		}
		
		SendChar_Zigbee(Zigbee_Ptr->ZigbeeSendCmdData[cir]);
		
		//printf("Zigbee_Ptr->ZigbeeSendCmdData[%d] = %d\r\n", cir, Zigbee_Ptr->ZigbeeSendCmdData[cir])
	}

	//printf("************cmdFlag = %d***********\r\n", CMD_Flag_Ptr->cmdFlag);
	CMD_Flag_Ptr->cmdFlag = NcNone;
	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = Zigbee_Ptr->ZigbeeSendCmdData;
	//ZigbeeResendInfo_Ptr->resendInfo = nc_send2;
}


void ZigbeeWaitForAck(void)
{
	static u32 recTime = 0;
	static u8 resendCount = 0;

	
	
	if(1 == ZigbeeResendInfo_Ptr->resendFlag)
	{
		if(ZigbeeACK == CMD_Flag_Ptr->cmdFlag)
		{
			resendCount = 0;
			ZigbeeResendInfo_Ptr->resendFlag = 0;
			CMD_Flag_Ptr->cmdFlag = NcNone;
			recTime = 0;
		}
		else
		{
			if(0 == recTime)
			{
				recTime = SystemRunningTime;
			}
			else
			{
				if(SystemRunningTime - recTime > ZigbeeResendInfo_Ptr->intervalTime_ms * 10)
				{
					u8 cir = 0;
					
					for(cir = 0; cir < 8; cir++)
					{
						if(0 == cir)
						{
							printf("resend\r\n");
						}
						
						SendChar_Zigbee(ZigbeeResendInfo_Ptr->resendInfo[cir]);
					}					

					if(ZIGBEE_RESEND_COUNT_MODE_ENABLE == ZigbeeResendInfo_Ptr->ResendCountMode)
					{
						resendCount++;
						recTime = 0;
						
						if(resendCount >= ZigbeeResendInfo_Ptr->resendNum)
						{
							resendCount = 0;
							ZigbeeResendInfo_Ptr->resendFlag = 0;
							printf("resend failed\r\n");
						}
					}
					else if(ZIGBEE_RESEND_COUNT_MODE_DISABLE == ZigbeeResendInfo_Ptr->ResendCountMode)
					{
						resendCount = 0;
						recTime = 0;
					}
					
				}
				
			}
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
	
	Zigbee_Ptr->recvValidDataFlag = 0;
	Zigbee_Ptr->runningInfo.Req_Station = 0x0000;
	Zigbee_Ptr->runningInfo.Req_Type = TypeUnknow;

	Zigbee_Ptr->ZigbeeSendCmdData[0] = 0xFE;
	Zigbee_Ptr->ZigbeeSendCmdData[1] = 0x08;
	Zigbee_Ptr->ZigbeeSendCmdData[2] = 0x00;
	Zigbee_Ptr->ZigbeeSendCmdData[3] = 0x00;
	Zigbee_Ptr->ZigbeeSendCmdData[4] = 0x01;
	Zigbee_Ptr->ZigbeeSendCmdData[5] = 0x02;
	Zigbee_Ptr->ZigbeeSendCmdData[6] = 0x7F;
	Zigbee_Ptr->ZigbeeSendCmdData[7] = 0x00;

	CMD_Flag_Ptr->cmdFlag = NcNone;
	CMD_Flag_Ptr->Cancel_Flag = NcNone;
	
	Id_Arr[0].zigbee_ID1 = 0x00;
	Id_Arr[0].zigbee_ID2 = 0x00;
	
	Id_Arr[1].zigbee_ID1 = 0xE4;
	Id_Arr[1].zigbee_ID2 = 0x89;
	
	Id_Arr[2].zigbee_ID1 = 0xBE;
	Id_Arr[2].zigbee_ID2 = 0x51;
	
	Id_Arr[3].zigbee_ID1 = 0x0B;
	Id_Arr[3].zigbee_ID2 = 0x5F;
	
	Id_Arr[4].zigbee_ID1 = 0x6E;
	Id_Arr[4].zigbee_ID2 = 0xA7;
	
	Id_Arr[5].zigbee_ID1 = 0x84;
	Id_Arr[5].zigbee_ID2 = 0x69;
	
	Id_Arr[6].zigbee_ID1 = 0x9B;
	Id_Arr[6].zigbee_ID2 = 0xFA;
	
	Id_Arr[7].zigbee_ID1 = 0x36;
	Id_Arr[7].zigbee_ID2 = 0xB8;
	
	Id_Arr[8].zigbee_ID1 = 0xC8;
	Id_Arr[8].zigbee_ID2 = 0x8A;
	
	Id_Arr[9].zigbee_ID1 = 0xC2;
	Id_Arr[9].zigbee_ID2 = 0x2A;
	
	Id_Arr[10].zigbee_ID1 = 0x2D;
	Id_Arr[10].zigbee_ID2 = 0x90;
	
	// 龙门
	Id_Arr[11].zigbee_ID1 = 0x6C;
	Id_Arr[11].zigbee_ID2 = 0xE4;
	
	ZigbeeResendInfo_Ptr->resendFlag = 0;
	ZigbeeResendInfo_Ptr->intervalTime_ms = 500;
	ZigbeeResendInfo_Ptr->resendNum = 30;
	ZigbeeResendInfo_Ptr->resendCtrlFunc = ZigbeeWaitForAck;
}


