#include "rfid.h"


u8 SendChar_RFID(u8 ch)  //���͵������� 
{
	//u16 timeout = U16_MAX;
	u32 timeout = 0;
	USART_SendData(USART3, ch);
	
	#if 0
	
	//while (!(USART2->SR & USART_FLAG_TXE));
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ����ڷ������
	
	#else
	
	timeout = SystemRunningTime;
	while(!(USART3->SR & USART_FLAG_TXE))
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


void SendWriteCMD(u8 *cmd)
{
	u8 cir = 0;
	
	cmd[2] = 0x03;
	cmd[3] = 0x41;
	cmd[4] = 0x00;
	cmd[5] = 0x42;
	
	for(cir = 0; cir < 6; cir++)
	{
		SendChar_RFID(cmd[cir]);
	}
}

void SendReadCMD(u8 *cmd)
{
	u8 cir = 0;
	
	cmd[2] = 0x03;
	cmd[3] = 0x41;
	cmd[4] = 0x15;
	cmd[5] = 0x57;
	
	for(cir = 0; cir < 6; cir++)
	{
		SendChar_RFID(cmd[cir]);
	}
}

void SendWriteID(u8 *cmd, u8 id)
{
	u8 cir = 0;
	
	cmd[9] = id;
	cmd[10] += id;
	
	for(cir = 0; cir < 11; cir++)
	{
		SendChar_RFID(cmd[cir]);
	}
}


void RFID_Write(u8 id)
{
	u8 writeCMD[11] = {0xAA, 0xBB, 0x08, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08};
	
	SendWriteCMD(writeCMD);

	SendWriteID(writeCMD, id);
	
}

void RFID_Read(void)
{
	u8 writeCMD[11] = {0xAA, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08};
	
	SendReadCMD(writeCMD);
}


void RFID_Usart_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ����GPIOʱ�Ӻ͸��ù���ʱ��RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* ���� USART Tx ����������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ���� USART Rx �������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
}



void RFID_Usart_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;

	RFID_Usart_GPIOInit();

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);								//��ʼ��

	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART3, &USART_ClockInitStructure);
	
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_Init(USART3, &USART_InitStructure);

	//USART3->CR1 |= (1 << 8);	// PE(У�����)�ж�ʹ��
	USART3->CR1 |= (1 << 5);	// RXNE(���ջ�����)�ǿ��ж�ʹ��
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);

	
}



