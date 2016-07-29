#include "zigbee.h"
#include "cfg_gpio.h"
#include "buffer.h"
#include "timer_opts.h"
#include "motion_control.h"

#define USART2_TC					((USART2->SR >> 6) & 0x0001)

Zigbee_Info Zigbee;
Zigbee_Info_P Zigbee_Ptr = &Zigbee;

RecvCmdFlag CMD_Flag;
RecvCmdFlag_P CMD_Flag_Ptr = &CMD_Flag;

Zigbee_ACK_Info ZigbeeResendInfo;
Zigbee_ACK_Info_P ZigbeeResendInfo_Ptr = &ZigbeeResendInfo;



/////////////////uart������115200����λ���ݣ�һλֹͣλ
///////��������//////////////
u8 receive_state=0;		//������ɱ�־
u8 receive_count=0;		//�������ݼ���
u8 i;					//ѭ�������
u8 flag1=0;				//��¼���ڽ����ֽ�

ZigbeeID_Info Id_Arr[12]; 

u8 nc_send1[8]=
	{0xfe, 0x08, 0x8c, 0xfe, 0x01, 0x02, 0x7F, 0x01};//С����ȡ��������׼��
u8 nc_send2[8]=
	{0xfe, 0x08, 0x8c, 0xfe, 0x01, 0x02, 0x7F, 0x02};//С����������������ȡ��
u8 nc_send3[8]=
	{0xfe, 0x08, 0x8c, 0xfe, 0x01, 0x02, 0x7F, 0x03};//С���뿪����
u8 nc_send4[8]=
	{0xfe, 0x08, 0x71, 0xdc, 0x01, 0x02, 0x7F, 0x01};//С����ȡ����͸���ťʹ��ֹͣ����

u8 zigbeeAck[8] = {0xfe, 0x08, 0x00, 0x00, 0x01, 0x02, 0x8f, 0xff};

u8 nc_receive[8];		//�������ݻ���


void Protocol_analysis(u8 rec_dat)
{
  u8 pres; 
  
  static u8 receive_num = 0x00;
  static u8 receive_flag = 0x00;

  
  pres  =       rec_dat;
  
  
  switch(receive_flag)
  {
    case 0x00:   
		if(pres == 0xFE) //֡ͷ
		{
			receive_flag = 0x01;
			Zigbee_Ptr->frm_1.startID = pres;
		}
		break;
		
    case 0x01:   
		receive_flag = 0x02; //����
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

u8 SendChar_Zigbee(u8 ch)  //���͵������� 
{
	//u16 timeout = U16_MAX;
	u32 timeout = 0;
	USART_SendData(USART2, ch);
	
	#if 0
	
	//while (!(USART2->SR & USART_FLAG_TXE));
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ����ڷ������
	
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

u8 SendChar_Zigbee(u8 ch)  //���͵������� 
{
	vu32 counter = 0x00;
	//USART_SendData(USART1, ch); 
	USART2->DR = (ch & (u16)0x01FF);
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
	while(!USART2_TC)
	{
		if((SystemRunningTime - counter) >= 100)		// ��ʱ10ms
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


void UART2_REC_IRQ(u8 UART2_DR)//���ڽ����жϺ���
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
			///�ն�����ȡ��////
			
			if(ZBandRFIDmapping[SpinStation_8] != Zigbee_Ptr->recvId)
			//if(1)
			{
				zigbeeReqQueue(0x0008);
				zigbeeAck[2] = Id_Arr[1].zigbee_ID1;
				zigbeeAck[3] = Id_Arr[1].zigbee_ID2;
				Send_Zigbee_ACK();
				CMD_Flag_Ptr->cmdFlag = GoodReq;
				//WarningLedCtrlPtr->twinkleFlag = 1;
				//BuzzerCtrlPtr->buzzerFlag = 1;
			}
			
			printf("0x0001\r\n");
						
			/////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x11))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			///�ն�����ȡ��////
			zigbeeReqQueue(0x0002);
			zigbeeAck[2] = Id_Arr[2].zigbee_ID1;
			zigbeeAck[3] = Id_Arr[2].zigbee_ID2;
			Send_Zigbee_ACK();
			printf("0x0002\r\n");
			CMD_Flag_Ptr->cmdFlag = GoodReq;
			/////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x21))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			///�ն�����ȡ��////
			zigbeeReqQueue(0x0003);
			zigbeeAck[2] = Id_Arr[3].zigbee_ID1;
			zigbeeAck[3] = Id_Arr[3].zigbee_ID2;
			Send_Zigbee_ACK();
			printf("0x0003\r\n");
			CMD_Flag_Ptr->cmdFlag = GoodReq;
			/////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x31))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			///�ն�����ȡ��////
			zigbeeReqQueue(0x0004);
			zigbeeAck[2] = Id_Arr[4].zigbee_ID1;
			zigbeeAck[3] = Id_Arr[4].zigbee_ID2;
			Send_Zigbee_ACK();
			printf("0x0003\r\n");
			CMD_Flag_Ptr->cmdFlag = GoodReq;
			/////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x02))
		{ 
			for(i=0;i<8;i++)
			{
				nc_receive[i] = 0x00;
			}

			zigbeeCancelQueue(0x0008);
			
			Send_Zigbee_ACK();
			printf("cancel 0x0001\r\n");
			///////�ն�����ֹͣȡ��/////////
			CMD_Flag_Ptr->cmdFlag = GoodReqCancel;	
			//////////////////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x12))
		{ 
			for(i=0;i<8;i++)
			{
				nc_receive[i] = 0x00;
			}
			Send_Zigbee_ACK();
			printf("cancel 0x0002\r\n");
			///////�ն�����ֹͣȡ��/////////
			CMD_Flag_Ptr->cmdFlag = GoodReqCancel;
			//////////////////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0x22))
		{ 
			for(i=0;i<8;i++)
			{
				nc_receive[i] = 0x00;
			}
			Send_Zigbee_ACK();
			printf("cancel 0x0003\r\n");
			///////�ն�����ֹͣȡ��/////////
			CMD_Flag_Ptr->cmdFlag = GoodReqCancel;
			//////////////////////////////
		}
		else if((nc_receive[6] == 0x7f) && (nc_receive[7] == 0xc1))
		{
			for(i = 0; i < 8; i++)
			{
				nc_receive[i] = 0x00;
			}
			//zigbeeReqQueue(0x000b);
			zigbeeAck[2] = Id_Arr[0].zigbee_ID1;
			zigbeeAck[3] = Id_Arr[0].zigbee_ID2;
			Send_Zigbee_ACK();
			printf("GoodLeav\r\n");
			//WarningLedCtrlPtr->twinkleFlag = 1;
			//BuzzerCtrlPtr->buzzerFlag = 1;
			/////�����Ѿ�������ȡ��//////
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

void Receive_handle2(void)
{
	if(receive_state == 1)
	{
		u8 cir = 0;
		receive_state = 0;

		if(nc_receive[6] == 0x7f)
		{
			u16 node = 0;

			node = (nc_receive[7] >> 4) + 1;
			
			if(0x01 == (nc_receive[7] & 0x0f))			// ����ȡ��
			{
				if(ZBandRFIDmapping[node] != Zigbee_Ptr->recvId)
				//if(1)
				{
					zigbeeReqQueue(node);
					zigbeeAck[2] = Id_Arr[node].zigbee_ID1;
					zigbeeAck[3] = Id_Arr[node].zigbee_ID2;
					Send_Zigbee_ACK();
					CMD_Flag_Ptr->cmdFlag = GoodReq;
					//WarningLedCtrlPtr->twinkleFlag = 1;
					//BuzzerCtrlPtr->buzzerFlag = 1;
					printf("req %04x\r\n", node);
				}
				
			}
			else if(0x02 == (nc_receive[7] & 0x0f))		// ����ȡ��ȡ��
			{
				u8 index = 0;

				if(1 == searchZigbeeData(node, &index))
				{
					zigbeeDeleteQueue(index);
					Send_Zigbee_ACK();
					CMD_Flag_Ptr->cmdFlag = GoodReqCancel;
					BuzzerCtrlPtr->buzzerFlag = 1;
					printf("cancel %04x\r\n", node);
				}
				
			}
			
		}
		else if(nc_receive[6] == 0x8f)
		{
			if(0xff == nc_receive[7])
			{
				printf("recv ACK\r\n");
				CMD_Flag_Ptr->cmdFlag = ZigbeeACK;
			}
		}

		for(cir = 0; cir < 8; cir++)
		{
			nc_receive[cir] = 0x00;
		}
		
	}
}


void Send_GettedGoods(u8 node)
{
	u8 cir = 8;
	
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
	}
	
	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = nc_send4;
}

void Send_GettedGoods2(u16 node)
{
	u8 cir = 8;
	
	nc_send4[2] = Id_Arr[node].zigbee_ID1;
	nc_send4[3] = Id_Arr[node].zigbee_ID2;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_GettedGoods2\r\n");
		}
		SendChar_Zigbee(nc_send4[cir]);
	}
	
	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = nc_send4;
}

void Send_GettedGoods3(void)
{
	u8 cir = 8;
	
	nc_send4[2] = Id_Arr[Zigbee_Ptr->recvId].zigbee_ID1;
	nc_send4[3] = Id_Arr[Zigbee_Ptr->recvId].zigbee_ID2;
	
	for(cir = 0; cir < 8; cir++)
	{
		if(0 == cir)
		{
			printf("Send_GettedGoods3\r\n");
		}
		SendChar_Zigbee(nc_send4[cir]);
	}
	
	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = nc_send4;
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

	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = nc_send1;
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

	ZigbeeResendInfo_Ptr->resendFlag = 1;
	ZigbeeResendInfo_Ptr->resendInfo = nc_send2;
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
					u8 cir = 8;
					for(cir = 0; cir < 8; cir++)
					{
						if(0 == cir)
						{
							printf("resend\r\n");
						}
						SendChar_Zigbee(ZigbeeResendInfo_Ptr->resendInfo[cir]);
					}
					resendCount++;
					recTime = 0;
				}

				if(resendCount >= ZigbeeResendInfo_Ptr->resendNum)
				{
					resendCount = 0;
					ZigbeeResendInfo_Ptr->resendFlag = 0;
					recTime = 0;
					printf("resend failed\r\n");
				}
			}
		}
		
		
	}

	
}

void Zigbee_Usart_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ����GPIOʱ�Ӻ͸��ù���ʱ��RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* ���� USART Tx ����������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ���� USART Rx �������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
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
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);								//��ʼ��

	
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

	Id_Arr[0].zigbee_ID1 = 0x00;
	Id_Arr[0].zigbee_ID2 = 0x00;
	
	Id_Arr[1].zigbee_ID1 = 0x71;
	Id_Arr[1].zigbee_ID2 = 0xdc;

	Id_Arr[2].zigbee_ID1 = 0x76;
	Id_Arr[2].zigbee_ID2 = 0x3f;

	Id_Arr[3].zigbee_ID1 = 0x54;
	Id_Arr[3].zigbee_ID2 = 0xd7;

	Id_Arr[4].zigbee_ID1 = 0xa4;
	Id_Arr[4].zigbee_ID2 = 0x52;

	Id_Arr[5].zigbee_ID1 = 0x40;
	Id_Arr[5].zigbee_ID2 = 0x47;

	Id_Arr[6].zigbee_ID1 = 0x05;
	Id_Arr[6].zigbee_ID2 = 0x74;

	Id_Arr[7].zigbee_ID1 = 0xd2;
	Id_Arr[7].zigbee_ID2 = 0xc1;

	Id_Arr[8].zigbee_ID1 = 0xd2;
	Id_Arr[8].zigbee_ID2 = 0xa1;

	Id_Arr[9].zigbee_ID1 = 0x60;
	Id_Arr[9].zigbee_ID2 = 0x40;

	Id_Arr[10].zigbee_ID1 = 0xd9;
	Id_Arr[10].zigbee_ID2 = 0x2a;

	// ����
	Id_Arr[11].zigbee_ID1 = 0x8c;
	Id_Arr[11].zigbee_ID2 = 0xfe;
	
	ZigbeeResendInfo_Ptr->resendFlag = 0;
	ZigbeeResendInfo_Ptr->intervalTime_ms = 100;
	ZigbeeResendInfo_Ptr->resendNum = 3;
	ZigbeeResendInfo_Ptr->resendCtrlFunc = ZigbeeWaitForAck;
}


