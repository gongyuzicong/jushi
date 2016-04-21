#include "cfg_can.h"
#include "cfg_usart.h"
#include "stm32f10x_can.h"
#include "buffer.h"

#define USED_FILTER_NUM 	8
#define FxR1_31_16_STDID_LIST_16BIT(x, ID)		(CAN->sFilterRegister[x].FR1 = (u32)((CAN->sFilterRegister[x].FR1 & 0x0000FFFF) | (((u32)ID << 21) & 0xFFFF0000)))
#define FxR1_15_0_STDID_LIST_16BIT(x, ID)		(CAN->sFilterRegister[x].FR1 = (u32)((CAN->sFilterRegister[x].FR1 & 0xFFFF0000) | (((u32)ID << 5) & 0x0000FFFF)))
#define FxR2_31_16_STDID_LIST_16BIT(x, ID)		(CAN->sFilterRegister[x].FR2 = (u32)((CAN->sFilterRegister[x].FR2 & 0x0000FFFF) | (((u32)ID << 21) & 0xFFFF0000)))
#define FxR2_15_0_STDID_LIST_16BIT(x, ID)		(CAN->sFilterRegister[x].FR2 = (u32)((CAN->sFilterRegister[x].FR2 & 0xFFFF0000) | (((u32)ID << 5) & 0x0000FFFF)))

#define FxR1_STDID_LIST_32BIT(x, ID)			(CAN->sFilterRegister[x].FR1 = (((u32)ID << 21) & 0xFFFF0000))
#define FxR2_STDID_LIST_32BIT(x, ID)			(CAN->sFilterRegister[x].FR1 = (((u32)ID << 21) & 0xFFFF0000))		


StructCanOpts_P CanOptsPtr = NULL;


//u32 setFilterMask_G = 0x00;
void CAN_FilterInit_Id_List(void)
{
	u32 setFilterMask = 0x00000000;
	u8 cir = 0;
	
	for(cir = 0; cir < USED_FILTER_NUM; cir++)
	{
		setFilterMask = (u32)((setFilterMask << 1) | 0x01);
	}
	//setFilterMask_G = setFilterMask;
	/* �ù������鹤���ڳ�ʼ��ģʽ */
  	CAN->FMR |= (u32)0x00000001;

	/* ���������� */
	CAN->FA1R &= ~(setFilterMask);

	/* ���ù�������Ϊidentify listģʽ */
	CAN->FM1R |= setFilterMask;

	/* 16λ */
	/* ���ù�������λ��Ϊ2��16λ */
	CAN->FS1R &= ~(setFilterMask);

	/* ������FIFO0 */
	CAN->FFA1R &= ~(setFilterMask);
	
	/* ����identify list */
	FxR1_31_16_STDID_LIST_16BIT(0, 0x02a1);
	FxR1_15_0_STDID_LIST_16BIT(0, 0x02a0);
	FxR2_31_16_STDID_LIST_16BIT(0, 0x0225);
	FxR2_15_0_STDID_LIST_16BIT(0, 0x0361);
	
	FxR1_31_16_STDID_LIST_16BIT(1, 0x0360);
	FxR1_15_0_STDID_LIST_16BIT(1, 0x0048);
	FxR2_31_16_STDID_LIST_16BIT(1, 0x0431);
	FxR2_15_0_STDID_LIST_16BIT(1, 0x03b2);
	
	FxR1_31_16_STDID_LIST_16BIT(2, 0x03c3);
	FxR1_15_0_STDID_LIST_16BIT(2, 0x0467);
	FxR2_31_16_STDID_LIST_16BIT(2, 0x02d8);
	FxR2_15_0_STDID_LIST_16BIT(2, 0x0331);

	FxR1_31_16_STDID_LIST_16BIT(3, 0x0193);
	FxR1_15_0_STDID_LIST_16BIT(3, 0x02db);
	FxR2_31_16_STDID_LIST_16BIT(3, 0x02e6);
	FxR2_15_0_STDID_LIST_16BIT(3, 0x02b4);

	FxR1_31_16_STDID_LIST_16BIT(4, 0x01f7);
	FxR1_15_0_STDID_LIST_16BIT(4, 0x02d5);
	FxR2_31_16_STDID_LIST_16BIT(4, 0x02dc);
	FxR2_15_0_STDID_LIST_16BIT(4, 0x01ff);

	FxR1_31_16_STDID_LIST_16BIT(5, 0x02cd);
	FxR1_15_0_STDID_LIST_16BIT(5, 0x03f0);
	FxR2_31_16_STDID_LIST_16BIT(5, 0x02e0);
	FxR2_15_0_STDID_LIST_16BIT(5, 0x03eb);

	FxR1_31_16_STDID_LIST_16BIT(6, 0x0109);
	FxR1_15_0_STDID_LIST_16BIT(6, 0x0331);
	FxR2_31_16_STDID_LIST_16BIT(6, 0x01E6);
	FxR2_15_0_STDID_LIST_16BIT(6, 0x022d);

	FxR1_31_16_STDID_LIST_16BIT(7, 0x03EB);
	FxR1_15_0_STDID_LIST_16BIT(7, 0x02CD);
	FxR2_31_16_STDID_LIST_16BIT(7, 0x0000);
	FxR2_15_0_STDID_LIST_16BIT(7, 0x0000);

	xBitOn(CAN->IER, 1);	// FIFO message pending interrupt enable
	xBitOn(CAN->IER, 0);	// Transmit mailbox empty interrupt enable
	
	/* ���������� */
	CAN->FA1R |= setFilterMask;

	/* �ù��������˳���ʼ��ģʽ */
	CAN->FMR &= ~(u32)0x00000001;
	
}

void Get_Can_Data(CanRxMsg *RxMessage)
{
	/* �ȴ�������� */
	while((CAN_MessagePending(CAN_FIFO0) == 0));
	/* ��ʼ���������ݽṹ�� */
	RxMessage->StdId = 0x00;
	RxMessage->IDE = CAN_ID_STD;
	RxMessage->DLC = 0;
	RxMessage->Data[0] = 0x00;
	RxMessage->Data[1] = 0x00;
	RxMessage->Data[2] = 0x00;
	RxMessage->Data[3] = 0x00;
	RxMessage->Data[4] = 0x00;
	RxMessage->Data[5] = 0x00;
	RxMessage->Data[6] = 0x00;
	RxMessage->Data[7] = 0x00;
	
   	/* �������� */
	CAN_Receive(CAN_FIFO0 , RxMessage);
	/*
	printf("The CAN has receive data: %x,%x,%x,%x,%x,%x,%x,%x\r\n",		   	
		    	RxMessage.Data[0], RxMessage.Data[1], RxMessage.Data[2],
				RxMessage.Data[3], RxMessage.Data[4], RxMessage.Data[5],
				RxMessage.Data[6], RxMessage.Data[7]);
				*/
	CAN_FIFORelease(CAN_FIFO0);
	
}

void CAN_Transmit_Personal(u8 TransmitMailbox, CanTxMsg* TxMessage)
{

	/* Set up the Id */
	CAN->sTxMailBox[TransmitMailbox].TIR &= ((u32)0x00000001);
	
	TxMessage->StdId &= (u32)0x000007FF;
	TxMessage->StdId = TxMessage->StdId << 21;

	CAN->sTxMailBox[TransmitMailbox].TIR |= (TxMessage->StdId | TxMessage->IDE | TxMessage->RTR);
    
    /* Set up the DLC */
    TxMessage->DLC &= (u8)0x0000000F;
    CAN->sTxMailBox[TransmitMailbox].TDTR &= (u32)0xFFFFFFF0;
    CAN->sTxMailBox[TransmitMailbox].TDTR |= TxMessage->DLC;

    /* Set up the data field */
    CAN->sTxMailBox[TransmitMailbox].TDLR = (((u32)TxMessage->Data[3] << 24) | 
                                             ((u32)TxMessage->Data[2] << 16) |
                                             ((u32)TxMessage->Data[1] << 8) | 
                                             ((u32)TxMessage->Data[0]));
    CAN->sTxMailBox[TransmitMailbox].TDHR = (((u32)TxMessage->Data[7] << 24) | 
                                             ((u32)TxMessage->Data[6] << 16) |
                                             ((u32)TxMessage->Data[5] << 8) |
                                             ((u32)TxMessage->Data[4]));

    /* Request transmission */
    CAN->sTxMailBox[TransmitMailbox].TIR |= ((u32)0x00000001);
	
}

u8 CAN_Transmit_Amend(CanTxMsg* TxMessage)
{
	u8 TransmitMailbox = 0;

	/* Select one empty transmit mailbox */
	if ((CAN->TSR&((u32)0x04000000)) == ((u32)0x04000000))
	{
		TransmitMailbox = 0;
	}
	else if ((CAN->TSR&((u32)0x08000000)) == ((u32)0x08000000))
	{
		TransmitMailbox = 1;
	}
	else if ((CAN->TSR&((u32)0x10000000)) == ((u32)0x10000000))
	{
		TransmitMailbox = 2;
	}
	else
	{
		TransmitMailbox = CAN_NO_MB;
	}

	if (TransmitMailbox != CAN_NO_MB)
	{
		/* Set up the Id */
		CAN->sTxMailBox[TransmitMailbox].TIR &= ((u32)0x00000001);
		
		TxMessage->StdId &= (u32)0x000007FF;
		TxMessage->StdId = TxMessage->StdId << 21;

		CAN->sTxMailBox[TransmitMailbox].TIR |= (TxMessage->StdId | TxMessage->IDE | TxMessage->RTR);

		/* Set up the DLC */
		TxMessage->DLC &= (u8)0x0000000F;
		CAN->sTxMailBox[TransmitMailbox].TDTR &= (u32)0xFFFFFFF0;
		CAN->sTxMailBox[TransmitMailbox].TDTR |= TxMessage->DLC;

		/* Set up the data field */
		CAN->sTxMailBox[TransmitMailbox].TDLR = (((u32)TxMessage->Data[3] << 24) | 
		                                         ((u32)TxMessage->Data[2] << 16) |
		                                         ((u32)TxMessage->Data[1] << 8) | 
		                                         ((u32)TxMessage->Data[0]));
		CAN->sTxMailBox[TransmitMailbox].TDHR = (((u32)TxMessage->Data[7] << 24) | 
		                                         ((u32)TxMessage->Data[6] << 16) |
		                                         ((u32)TxMessage->Data[5] << 8) |
		                                         ((u32)TxMessage->Data[4]));

		/* Request transmission */
		CAN->sTxMailBox[TransmitMailbox].TIR |= ((u32)0x00000001);

		SendCanBuf_Delete();
	}

  return TransmitMailbox;
}



void Transmit_Can_Data(CanTxMsg tmp)
{
	u8 TransmitMailbox = 0;		/* ������Ϣ����״̬���� */
	vu32 TransmitCounter = TransmitTimeOut;
	CanTxMsg TxMessage;
	/* ���÷������ݽṹ�� ����׼ID��ʽ ��IDΪ0xAA������֡ �����ݳ���Ϊ8���ֽ� */
	TxMessage.StdId = tmp.StdId;
	TxMessage.RTR = tmp.RTR;
	TxMessage.IDE = tmp.IDE;
	TxMessage.DLC = tmp.DLC;
	TxMessage.Data[0] = tmp.Data[0];
	TxMessage.Data[1] = tmp.Data[1];
	TxMessage.Data[2] = tmp.Data[2];
	TxMessage.Data[3] = tmp.Data[3];
	TxMessage.Data[4] = tmp.Data[4];
	TxMessage.Data[5] = tmp.Data[5];
	TxMessage.Data[6] = tmp.Data[6];
	TxMessage.Data[7] = tmp.Data[7];
	/* �������� */
	TransmitMailbox = CAN_Transmit(&TxMessage);
	/* �ȴ�������� */
#if 1
	do
	{
		TransmitCounter--;
	}
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK) && (TransmitCounter));
#else
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK));
#endif
}

void Transmit_Can_Data_2(CanTxMsg tmp)
{
	u8 TransmitMailbox = 0;		/* ������Ϣ����״̬���� */
	vu32 TransmitCounter = TransmitTimeOut;
	CanTxMsg TxMessage;
	/* ���÷������ݽṹ�� ����׼ID��ʽ ��IDΪ0xAA������֡ �����ݳ���Ϊ8���ֽ� */
	TxMessage.StdId = tmp.StdId;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = tmp.DLC;
	TxMessage.Data[0] = tmp.Data[0];
	TxMessage.Data[1] = tmp.Data[1];
	TxMessage.Data[2] = tmp.Data[2];
	TxMessage.Data[3] = tmp.Data[3];
	TxMessage.Data[4] = tmp.Data[4];
	TxMessage.Data[5] = tmp.Data[5];
	TxMessage.Data[6] = tmp.Data[6];
	TxMessage.Data[7] = tmp.Data[7];
	/* �������� */
	TransmitMailbox = CAN_Transmit(&TxMessage);
	/* �ȴ�������� */
#if 1
	do
	{
		TransmitCounter--;
	}
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK) && (TransmitCounter));
#else
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK));
#endif
}

void Transmit_Can_Data_3(CanTxMsg TxMessage)
{
	u8 TransmitMailbox = 0;		/* ������Ϣ����״̬���� */
	vu32 TransmitCounter = 0x00;

	/* �������� */
	TransmitMailbox = CAN_Transmit(&TxMessage);
	/* �ȴ�������� */
#if 0
	do
	{
		TransmitCounter--;
	}
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK) && (TransmitCounter));
#else
	//while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK));
	TransmitCounter = SystemRunningTime;
	while(CAN_TransmitStatus(TransmitMailbox) != CANTXOK)
	{
		if((SystemRunningTime - TransmitCounter) >= 10)		// ��ʱ10ms
		{
			printf("break\r\n");
			break;
		}
		else
		{
			//RecvCanAndUsartFunc();
		}
	}
	printf("SystemRunningTime = %d, TransmitCounter = %d, SystemRunningTime - TransmitCounter = %d\r\n", SystemRunningTime, TransmitCounter, (SystemRunningTime - TransmitCounter));
#endif
}
#if 0
void Transmit_Can_Data_Unwait(CanTxMsg TxMessage)
{
	u8 TransmitMailbox = 0;		/* ������Ϣ����״̬���� */

	/* �������� */
	TransmitMailbox = CAN_Transmit(&TxMessage);
}
#endif

////////////////////////////////CAN���Գ���//////////////////////////////////////////
/**************************
����can�Ľ����Ƿ����
**************************/
void can_test(void)
{
	CanRxMsg RxMessage;			/* ������Ϣ���սṹ�� */

	/* �ȴ�������� */
	while((CAN_MessagePending(CAN_FIFO0) == 0));	
	/* ��ʼ���������ݽṹ�� */
	RxMessage.StdId = 0x00;
	RxMessage.IDE = CAN_ID_STD;
	RxMessage.DLC = 0;
	RxMessage.Data[0] = 0x00;
	RxMessage.Data[1] = 0x00;
	RxMessage.Data[2] = 0x00;
	RxMessage.Data[3] = 0x00;
	RxMessage.Data[4] = 0x00;
	RxMessage.Data[5] = 0x00;
	RxMessage.Data[6] = 0x00;
	RxMessage.Data[7] = 0x00;
	
   	/* �������� */
	CAN_Receive(CAN_FIFO0 , &RxMessage);
	
	printf("The CAN has receive data: RxMessage.StdId %x, %x,%x,%x,%x,%x,%x,%x,%x\r\n",	RxMessage.StdId,   	
		    	RxMessage.Data[0], RxMessage.Data[1], RxMessage.Data[2],
				RxMessage.Data[3], RxMessage.Data[4], RxMessage.Data[5],
				RxMessage.Data[6], RxMessage.Data[7]);
	
	CAN_FIFORelease(CAN_FIFO0);
	
}


void KeySendCan(void)
{
	u8 TransmitMailbox = 0;		/* ������Ϣ����״̬���� */
	CanTxMsg TxMessage;			/* ������Ϣ���ͽṹ�� */
	//CanRxMsg RxMessage;			/* ������Ϣ���սṹ�� */
	//printf("setFilterMask_G = %lx\r\n", setFilterMask_G);
	CAN_FIFORelease(CAN_FIFO0);
	/* ���÷������ݽṹ�� ����׼ID��ʽ ��IDΪ0xAA������֡ �����ݳ���Ϊ8���ֽ� */
	TxMessage.StdId = 0x02a1;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 8;
	TxMessage.Data[0] = 0x00;
	TxMessage.Data[1] = 0x12;
	TxMessage.Data[2] = 0x34;
	TxMessage.Data[3] = 0x56;
	TxMessage.Data[4] = 0x78;
	TxMessage.Data[5] = 0xAB;
	TxMessage.Data[6] = 0xCD;
	TxMessage.Data[6] = 0xEF;
	TxMessage.Data[7] = 0xFF;

	printf("The CAN has send id: %x, data: %x,%x,%x,%x,%x,%x,%x,%x\r\n", TxMessage.StdId,
		   	TxMessage.Data[0], TxMessage.Data[1], TxMessage.Data[2],
			TxMessage.Data[3], TxMessage.Data[4], TxMessage.Data[5],
			TxMessage.Data[6], TxMessage.Data[7]);
	/* �������� */
	TransmitMailbox = CAN_Transmit(&TxMessage);
	/* �ȴ�������� */
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK));
	
	printf("The CAN has send id: %x, data: %x,%x,%x,%x,%x,%x,%x,%x\r\n", TxMessage.StdId,
		   	TxMessage.Data[0], TxMessage.Data[1], TxMessage.Data[2],
			TxMessage.Data[3], TxMessage.Data[4], TxMessage.Data[5],
			TxMessage.Data[6], TxMessage.Data[7]);
}


/****************************************************************
CAN�����շ����Գ���
ʹ��ʱע�⽫����void CB_CAN_Config(void)��CAN_Mode�ĳ�CAN_Mode_LoopBack
*****************************************************************/
void can_loop_test(void)
{
	u8 TransmitMailbox = 0;		/* ������Ϣ����״̬���� */
	CanTxMsg TxMessage;			/* ������Ϣ���ͽṹ�� */
	CanRxMsg RxMessage;			/* ������Ϣ���սṹ�� */
	//printf("setFilterMask_G = %lx\r\n", setFilterMask_G);
	CAN_FIFORelease(CAN_FIFO0);
	/* ���÷������ݽṹ�� ����׼ID��ʽ ��IDΪ0xAA������֡ �����ݳ���Ϊ8���ֽ� */
	TxMessage.StdId = 0x02a1;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 8;
	TxMessage.Data[0] = 0x00;
	TxMessage.Data[1] = 0x12;
	TxMessage.Data[2] = 0x34;
	TxMessage.Data[3] = 0x56;
	TxMessage.Data[4] = 0x78;
	TxMessage.Data[5] = 0xAB;
	TxMessage.Data[6] = 0xCD;
	TxMessage.Data[6] = 0xEF;
	TxMessage.Data[7] = 0xFF;

	printf("The CAN has send id: %x, data: %x,%x,%x,%x,%x,%x,%x,%x\r\n", TxMessage.StdId,
		   	TxMessage.Data[0], TxMessage.Data[1], TxMessage.Data[2],
			TxMessage.Data[3], TxMessage.Data[4], TxMessage.Data[5],
			TxMessage.Data[6], TxMessage.Data[7]);
	/* �������� */
	TransmitMailbox = CAN_Transmit(&TxMessage);
	/* �ȴ�������� */
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK));
	
	printf("The CAN has send id: %x, data: %x,%x,%x,%x,%x,%x,%x,%x\r\n", TxMessage.StdId,
		   	TxMessage.Data[0], TxMessage.Data[1], TxMessage.Data[2],
			TxMessage.Data[3], TxMessage.Data[4], TxMessage.Data[5],
			TxMessage.Data[6], TxMessage.Data[7]);
	
	/* �ȴ�������� */
	while((CAN_MessagePending(CAN_FIFO0) == 0));	
	/* ��ʼ���������ݽṹ�� */
	RxMessage.StdId = 0x00;
	RxMessage.IDE = CAN_ID_STD;
	RxMessage.DLC = 0;
	RxMessage.Data[0] = 0x00;
	RxMessage.Data[1] = 0x00;
	RxMessage.Data[2] = 0x00;
	RxMessage.Data[3] = 0x00;
	RxMessage.Data[4] = 0x00;
	RxMessage.Data[5] = 0x00;
	RxMessage.Data[6] = 0x00;
	RxMessage.Data[7] = 0x00;
	
   	/* �������� */
	CAN_Receive(CAN_FIFO0 , &RxMessage);
	
	printf("The CAN has receive id: %x, data: %x,%x,%x,%x,%x,%x,%x,%x\r\n", RxMessage.StdId,
		    	RxMessage.Data[0], RxMessage.Data[1], RxMessage.Data[2],
				RxMessage.Data[3], RxMessage.Data[4], RxMessage.Data[5],
				RxMessage.Data[6], RxMessage.Data[7]);
	
}

void CB_CAN_Config(void)
{
	/* ���� CAN �������͹�������ʼ���ṹ�� */
	CAN_InitTypeDef        CAN_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);						//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;			//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;					//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);										//��ʼ��

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);						//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN_TX_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;			//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;					//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);										//��ʼ��
	/*
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN_TX_IRQChannel;//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ��

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ��
	*/
	
	/* CAN �Ĵ�����λ */
	CAN_DeInit();
	CAN_StructInit(&CAN_InitStructure);
	
	/* 	
	*	CAN ��������ʼ����
	*  
	*	ʧ��ʱ�䴥��ͨѶģʽ
	*	ʧ���Զ����߹���
	*	ʧ���Զ�����ģʽ
	*	ʧ�ܷ��Զ��ش���ģʽ
	*	ʧ�ܽ��� FIFO ����ģʽ
	*	ʧ�ܷ��� FIFO ���ȼ�
	*	CAN Ӳ�������ڻ���ģʽ 
	*	����ͬ����Ծ��� 1 ��ʱ�䵥λ
	*	ʱ��� 1 Ϊ 8 ��ʱ�䵥λ
	*	ʱ��� 2 Ϊ 7 ��ʱ�䵥λ
	*	ʱ�䵥λ�ĳ���Ϊ5  
	*/
	
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
	CAN_InitStructure.CAN_Prescaler = 4;
	CAN_Init(&CAN_InitStructure);
	
	/*
	CAN->MCR |= ((u32)0x00000001);
	CAN->IER |= ((u32)0x00000001);
	CAN->MCR &= ~((u32)0x00000001);
	*/
	/* 	 
	*	CAN ��������ʼ����
	*
	*	��ʼ��������2
	*	��ʶ������λģʽ
	*	ʹ��1 �� 32 λ������ 
	*	��������ʶ��Ϊ (0x00AA << 5)
	*	���������α�ʶ��0xFFFF
	*	������ FIFO0 ָ�������0 
	*	ʹ�ܹ����� 
	*/
#if 0
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = ((0x12<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow = ((0x12<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow =  0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
#else
	
	CAN_FilterInit_Id_List();
	
#endif
	
	CanOptsPtr = (StructCanOpts_P)malloc(sizeof(StructCanOpts));
	CanOptsPtr->YL_KeySendCan = KeySendCan;
	CanOptsPtr->YL_Transmit_Can_Data = Transmit_Can_Data;
	CanOptsPtr->YL_Transmit_Can_Data_2 = Transmit_Can_Data_2;
	CanOptsPtr->YL_Transmit_Can_Data_3 = Transmit_Can_Data_3;
	//CanOptsPtr->YL_Transmit_Can_Data_Unwait = Transmit_Can_Data_Unwait;
	CanOptsPtr->YL_can_loop_test = can_loop_test;
	CanOptsPtr->YL_can_test = can_test;
}


