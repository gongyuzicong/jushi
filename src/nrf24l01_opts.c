#include "nrf24l01_opts.h"
#include "timer_opts.h"
#include "spi_opts.h"
#include "buffer.h"
#include "motion_control.h"

//NrfOptStruct nrfOpts;
//NrfOptStruct_P NRF24L01OptsPtr = &nrfOpts;
u8 need2SendInfo = 0;

u8 const TX_ADDRESS[TX_ADR_WIDTH] = {0x12, 0x34, 0x56, 0x78, 0x9A};	//���ص�ַ
u8 const RX_ADDRESS[RX_ADR_WIDTH] = {0x12, 0x34, 0x56, 0x78, 0x9A};	//���յ�ַ
u8 tx_buf[TX_PLOAD_WIDTH] = {0x03, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
u8 rx_buf[RX_PLOAD_WIDTH] = {0x00, 0x00, 0x00, 0x00, 0x00};


#ifdef NRF_USE_SPI

//��NRF��ȡһ���ֽ�����
//reg �Ĵ�����ַ
u8 SPI_Read(u8 regAddr)
{
	u8 reg_val;
	
	NRF24L01_CSN = 0;                		//Ƭѡʹ��  
	
	SPI1_ReadWriteByte(regAddr);
	reg_val = SPI1_ReadWriteByte(0xff);    	// ��ȡ���ݵ�reg_val
	
	NRF24L01_CSN = 1;                		// ȡ��Ƭѡ
	
	return(reg_val);        				// ���ض�ȡ������
}
   


//��NRFд��һ���ֽ�����
//reg �Ĵ�����ַ  value Ҫд�������
u8 SPI_RW_Reg(u8 reg, u8 value)
{
	u8 status;
	
	NRF24L01_CSN = 0;                   	// CSN low, init SPI transaction
	status = SPI1_ReadWriteByte(reg);
    SPI1_ReadWriteByte(value);
	NRF24L01_CSN = 1;                   	// CSN high again
		
	return(status);            				// return nRF24L01 status uchar
}



//��NRF��ȡ����ֽ�����
//reg �Ĵ�����ַ  *pBuf ��ȡ���ݴ洢ָ��  uchars ��ȡ���ֽڸ���
u8 SPI_Read_Buf(u8 reg, u8 *pBuf, u8 uchars)
{
	u8 status,uchar_ctr;
	
	NRF24L01_CSN = 0;                    		// Set CSN low, init SPI tranaction

	status = SPI1_ReadWriteByte(reg);
	for(uchar_ctr = 0; uchar_ctr < uchars; uchar_ctr++)		  	//ѭ�� uchars��
	{
    	pBuf[uchar_ctr] = SPI1_ReadWriteByte(0xff); 			//�ֱ�SPI_RW(0)���������ݵ�ַ ����������
	}
	
	NRF24L01_CSN = 1;                           
	
	return status;                    // return nRF24L01 status uchar
}


//��NRFд�����ֽ�����
//reg �Ĵ�����ַ  *pBuf Ҫд�������  uchars д����ֽڸ���
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 uchars)
{
	u8 status,uchar_ctr;
	
	NRF24L01_CSN = 0;            // SPIʹ��       

	status = SPI1_ReadWriteByte(reg);
	
	for(uchar_ctr=0; uchar_ctr < uchars; uchar_ctr++) 		// �������ݸ���ѭ��	
	{
		SPI1_ReadWriteByte(*pBuf++);						// ����������� ����д��
	}

	NRF24L01_CSN = 1;           //�ر�SPI
	return(status);    // 
}


//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 NRF24L01_Check(void)
{
	u8 buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	u8 i;
	
    //SPI1_SetSpeed(SPI_BaudRatePrescaler_8);       //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, buf, 5);   //д��5���ֽڵĵ�ַ.	
	SPI_Read_Buf(TX_ADDR, buf, 5); //����д��ĵ�ַ  
	
	for(i=0;i<5;i++)
	{
		if(buf[i] != 0xA5)
		{
			break;
		}
	}
	
	if(i != 5)
	{
		return 1;//���24L01����	
	}
	
	return 0;		 //��⵽24L01
}


//NRF24L01��ʼ��
//m 1 ����ģʽ   0 ����ģʽ
void TX_Mode(void)	        //���� or ���� ģʽ ��ʼ��
{
	u8 temp = 0;
	
 	NRF24L01_CE = 0;    // chip enable
 	
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);    // д���ص�ַ	
	SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // д���ն˵�ַ
	
	SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x0f);   		//���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB
	SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 0);        //   �����ŵ�����Ϊ2.4GHZ���շ�����һ��
	SPI_RW_Reg(WRITE_REG_CMD + RX_PW_P0, RX_PLOAD_WIDTH); //���ý������ݳ��ȣ���������Ϊ32�ֽ�
	
	#if 1

	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01);      //  Ƶ��0�Զ�	ACKӦ������	
	temp = SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_EN_AA = %x\r\n", temp);
	temp = 0xff;
	
	SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01);  //  ������յ�ַֻ��Ƶ��0�������Ҫ��Ƶ�����Բο�Page21  
	temp = SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("NRF24L01_EN_RXADDR = %x\r\n", temp);
	temp = 0xff;
	
	SPI_RW_Reg(WRITE_REG_CMD + SETUP_RETR, 0x1a);	// �Զ��ط�10��, ���500us
	temp = SPI_Read(READ_REG_CMD + SETUP_RETR);
	printf("NRF24L01_SETUP_RETR = %x\r\n\r\n", temp);
	temp = 0xff;
	
	#else
	
	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x00);
	temp = SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_EN_AA = %x\r\n", temp);
	temp = 0xff;
	
	SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x00);
	temp = SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("NRF24L01_EN_RXADDR = %x\r\n", temp);
	temp = 0xff;
	
	SPI_RW_Reg(WRITE_REG_CMD + SETUP_RETR, 0x00);
	temp = SPI_Read(READ_REG_CMD + SETUP_RETR);
	printf("NRF24L01_SETUP_RETR = %x\r\n\r\n", temp);
	temp = 0xff;
	#endif
	
	SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0e);   		 // IRQ�շ�����ж���Ӧ��16λCRC��������

	NRF24L01_CE = 1;
}

void Change_To_TX_Mode_Fast(void)
{
	NRF24L01_CE = 0;    // chip enable
 	
	SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0e);   		 // IRQ�շ�����ж���Ӧ��16λCRC��������

	NRF24L01_CE = 1;
}

void RX_Mode(void)
{
	u8 temp = 0;
	
	NRF24L01_CE = 0;	// chip enable
	
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);	  // д���ص�ַ 
	SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // д���ն˵�ַ
	SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01);			//	������յ�ַֻ��Ƶ��0�������Ҫ��Ƶ�����Բο�Page21  
	SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x0f); 			//���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB
	SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 0);		 			//   �����ŵ�����Ϊ2.4GHZ���շ�����һ��
	SPI_RW_Reg(WRITE_REG_CMD + RX_PW_P0, RX_PLOAD_WIDTH); //���ý������ݳ��ȣ���������Ϊ32�ֽ�
	
	#if 1

	
	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01);				//	Ƶ��0�Զ�	ACKӦ������ 
	temp = SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_EN_AA = %x\r\n\r\n", temp);
	
	#else
	
	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x00);
	temp = SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_EN_AA = %x\r\n", temp);
	//temp = 0xff;
	
	#endif
	
	SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0f);			// IRQ�շ�����ж���Ӧ��16λCRC��������
	
	NRF24L01_CE = 1;
}

void Change_To_RX_Mode_Fast(void)
{
	NRF24L01_CE = 0;    // chip enable
 	//NRF24L01_CSN = 1;   // Spi disable 
 	
	SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0f);   		 // IRQ�շ�����ж���Ӧ��16λCRC��������

	NRF24L01_CE = 1;
}

//NRF�������ݺ���
//rx_buf  ���ݻ�����
//�ú������NRF״̬�Ĵ���״̬ �����ж������������ݵ�rx_buf������
u8 nRF24L01_RxPacket(u8 *rx_buf)
{	 
    u8 sta;
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz�� 

	sta = SPI_Read(READ_REG_CMD + STATUS);	    				// ��ȡ״̬�Ĵ������ж����ݽ���״��
	SPI_RW_Reg(WRITE_REG_CMD + STATUS, sta);   //���ж� �����յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�־��
	if(sta & RX_OK)								// �ж��Ƿ���յ�����
	{

		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		SPI_RW_Reg(FLUSH_RX,0xff);

		return 0; 
	}
	
	return 1;
}


//��������tx_buf�е����ݷ��ͳ�ȥ
//tx_buf  Ҫ���͵����ݻ�����
u8 nRF24L01_TxPacket(u8 *tx_buf)
{	 
    u8 st;
	
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz�� 
	
	NRF24L01_CSN = 0;			//StandBy Iģʽ	

	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // װ������	

	NRF24L01_CSN = 1;		 				//�ø�CE���������ݷ���
	
	while(NRF24L01_IRQ != 0);		   //�ȴ��������
	
	st=	SPI_Read(STATUS);			  //��NRF�Ĵ���״̬
	SPI_RW_Reg(WRITE_REG_CMD + STATUS, st);  //���ж�

	if(st & MAX_TX)//�ﵽ����ط�����
	{
		SPI_RW_Reg(FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}

	if(st & TX_OK)				 //���ͳɹ�
	{
		return TX_OK;
	}

	return 0xff;
}

u8 NRF24L01_Get_Status_Reg(void)
{
	u8 temp;
	
	temp = SPI_Read(READ_REG_CMD + STATUS);

	return temp;
}

u8 NRF24L01_Clean_Status_Reg(u8 clean)
{
	u8 temp = 0xff;
	
	temp = SPI_RW_Reg(WRITE_REG_CMD + STATUS, clean);

	return temp;
}

u8 NRF24L01_Clean_All_Status_Reg(void)
{
	u8 temp = 0;

	temp = SPI_Read(READ_REG_CMD + STATUS);
	temp = SPI_RW_Reg(WRITE_REG_CMD + STATUS, temp);
	//printf("STATUS_A1 = %x\r\n", temp);
	temp = SPI_Read(READ_REG_CMD + STATUS);
	//printf("STATUS_A2 = %x\r\n", temp);
	
	return 0;
}

void NRF24L01_Read_Data_2Buf(void)
{
	
	
	SPI_Read_Buf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);	//��ȡ����

}


void Send_Info_To_Contorler(void)
{
	Change_To_TX_Mode_Fast();
	
	if(PWM_MODE == ctrlParasPtr->speedMode)
	{
		tx_buf[0] = 0x01;
		tx_buf[1] = 0x03;
		tx_buf[2] = 0x07;
		
		tx_buf[3] = ctrlParasPtr->agvStatus;				// AGV STATUS
		//printf("agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
		tx_buf[4] = ctrlParasPtr->leftMotorSettedSpeed;		// LeftMotorSpeed
		//printf("leftMotorSettedSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
		//tx_buf[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);		// LeftMotorEN
		tx_buf[5] = MOTOR_LEFT_EN_IN;		// LeftMotorEN
		//printf("MOTOR_LEFT_EN_IN = %d\r\n", MOTOR_LEFT_EN_IN);
		tx_buf[6] = MOTOR_LEFT_FR_IN;		// LeftMotorFR
		//printf("MOTOR_LEFT_FR_IN = %d\r\n", MOTOR_LEFT_FR_IN);
		
		tx_buf[7] = ctrlParasPtr->rightMotorSettedSpeed;		// RightMotorSpeed
		//printf("rightMotorSettedSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
		tx_buf[8] = MOTOR_RIGHT_EN_IN;		// RightMotorEN
		//printf("MOTOR_RIGHT_EN_IN = %d\r\n", MOTOR_RIGHT_EN_IN);
		tx_buf[9] = MOTOR_RIGHT_FR_IN;		// RightMotorFR
		//printf("MOTOR_RIGHT_FR_IN = %d\r\n", MOTOR_RIGHT_FR_IN);
		
		SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	}
	else if(X1X2X3_I_MODE == ctrlParasPtr->speedMode)
	{
		u8 tempValue = 0xff;
		
		tx_buf[0] = 0x01;
		tx_buf[1] = 0x03;
		tx_buf[2] = 0x07;
		
		tx_buf[3] = ctrlParasPtr->agvStatus;				// AGV STATUS
		//printf("agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
		tempValue = (MOTOR_LEFT_X3_In << 2) + (MOTOR_LEFT_X2_In << 1) + MOTOR_LEFT_X1_In;
		printf("%x, %x, %x, tempValue = %x\r\n", MOTOR_LEFT_X3_In, MOTOR_LEFT_X2_In, MOTOR_LEFT_X1_In, tempValue);
		tx_buf[4] = tempValue;
		//tx_buf[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);		// LeftMotorEN
		tx_buf[5] = MOTOR_LEFT_EN_IN;		// LeftMotorEN
		//printf("MOTOR_LEFT_EN_IN = %d\r\n", MOTOR_LEFT_EN_IN);
		tx_buf[6] = MOTOR_LEFT_FR_IN;		// LeftMotorFR
		//printf("MOTOR_LEFT_FR_IN = %d\r\n", MOTOR_LEFT_FR_IN);
		
		tempValue = (MOTOR_RIGHT_X3_In << 2) + (MOTOR_RIGHT_X2_In << 1 ) + MOTOR_RIGHT_X1_In;
		printf("%x, %x, %x, tempValue = %x\r\n", MOTOR_RIGHT_X3_In, MOTOR_RIGHT_X2_In, MOTOR_RIGHT_X1_In, tempValue);
		tx_buf[7] = tempValue;
		
		tx_buf[8] = MOTOR_RIGHT_EN_IN;		// RightMotorEN
		//printf("MOTOR_RIGHT_EN_IN = %d\r\n", MOTOR_RIGHT_EN_IN);
		tx_buf[9] = MOTOR_RIGHT_FR_IN;		// RightMotorFR
		//printf("MOTOR_RIGHT_FR_IN = %d\r\n", MOTOR_RIGHT_FR_IN);
		
		SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	}
	
}




void NRF24L01_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;  // PC0 1 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      		//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);			  //���� ȡ��SPI����Ƭѡ

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	      //PC2 ��������  ��IO�ж� NRF�Ƿ��е͵�ƽ�ź�
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void NRF24L01_SPI_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	
	SPI_Cmd(SPI1, DISABLE); // SPI���費ʹ��			  NRF��SPIҪ��������һ��

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI����
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	//���ͽ���8λ֡�ṹ

	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;			//ʱ�����յ�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;		//���ݲ����ڵ�1��ʱ����

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			//NSS�ź����������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;			//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  				//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); 								//ʹ��SPI����
}



void NRF24L01_TEST_Send(void)
{
	#if 0
	
	u8 temp = 0;
	
	printf("STATUS = %x\r\n", SPI_Read(READ_REG_CMD + STATUS));
	printf("FIFO_STATUS = %x\r\n", SPI_Read(READ_REG_CMD + FIFO_STATUS));
	
	//NRF24L01_TxPacket((u8 *)tx_buf);
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	//SPI_Write_Buf(WR_TX_PLOAD, tx_buf, 1);
	//NRF24L01_I_SPI_Write_Buf(WR_TX_PLOAD, tx_buf, 1);
	Delay_ms(100);

	temp = SPI_Read(READ_REG_CMD + STATUS);
	printf("STATUS3 = %x\r\n", temp);
	printf("FIFO_STATUS3 = %x\r\n", SPI_Read(READ_REG_CMD + FIFO_STATUS));
	SPI_RW_Reg(WRITE_REG_CMD + STATUS, temp);
	temp = SPI_Read(READ_REG_CMD + STATUS);
	printf("STATUS_A = %x\r\n\r\n", temp);
	
	#else
	Change_To_TX_Mode_Fast();
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	
	#endif
	
}

void nrf_up(void)
{
	Change_To_TX_Mode_Fast();
	tx_buf[3] = 0x01;
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
}

void nrf_down(void)
{
	Change_To_TX_Mode_Fast();
	tx_buf[3] = 0x02;
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
}

void nrf_left(void)
{
	Change_To_TX_Mode_Fast();
	tx_buf[3] = 0x03;
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
}

void nrf_right(void)
{
	Change_To_TX_Mode_Fast();
	tx_buf[3] = 0x04;
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
}


void NRF24L01_TEST_Recv(void)
{
	int i = 0, status = 0;
	printf("%d\r\n", SPI_Read(READ_REG_CMD + STATUS));
	if(0x40 == SPI_Read(READ_REG_CMD + STATUS))
	//if(1)
	{	
		printf("STATUS = %x\r\n", SPI_Read(READ_REG_CMD + STATUS));
		printf("FIFO_STATUS = %x\r\n", SPI_Read(READ_REG_CMD + FIFO_STATUS));
		
		status = SPI_Read_Buf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);
		printf("r_status = %x\r\n", status);
		//Delay_ms(100);

		status = 0;
		status = SPI_Read(READ_REG_CMD + STATUS);
		printf("STATUS4 = %x\r\n", status);
		printf("FIFO_STATUS4 = %x\r\n", SPI_Read(READ_REG_CMD + FIFO_STATUS));
		for(i = 0; i < RX_PLOAD_WIDTH; i++)
		{
			printf("rx_buf[%d] = %d\r\n", i, rx_buf[i]);
		}
		
		SPI_RW_Reg(WRITE_REG_CMD + STATUS, status);
		status = SPI_Read(READ_REG_CMD + STATUS);
		printf("STATUS4 = %x\r\n\r\n", status);
		
	}
}



void NFR24L01_Init(void)
{
	int i = 0;
	
	NRF24L01_GPIO_Init();
	NRF24L01_SPI_Init();
	if(0 != NRF24L01_Check())
	{
		printf("NRF24L01 Check Error!\r\n");
		return;
	}
	else
	{
		PCout(5) = 0;
		printf("NRF24L01 Check Success!\r\n");
	}
	
	TX_Mode();
	//RX_Mode();
	Change_To_RX_Mode_Fast();
	for(i = 0; i < TX_PLOAD_WIDTH; i++)
	{
		tx_buf[i] = 0;
	}
	
	tx_buf[0] = 0x01;
	tx_buf[1] = 0x01;
	tx_buf[2] = 0x02;
	
}


#else

/*
������ĺ��������GPIO ģ��SPI �Ĺ��ܡ�������ֽڣ� MOSI����MSB
ѭ�������ͬʱ�������ֽڣ� MISO����LSB ѭ�����롣�����ز������½������ݱ仯��
CPOL(ʱ�Ӽ���) = 0, CPHA(ʱ����λ) = 0;
SCKʱ�ӵĵ�һ����(����Ϊ������)��������λ��������SCK ����ʼ��Ϊ�͵�ƽ�����жϳ�����
*/
u8 NRF24L01_I_SPI_RW(u8 byte)
{
	u8 bit_ctr;
	
	for(bit_ctr = 0; bit_ctr < 8; bit_ctr++) 	// output 8-bit
	{
		NRF24L01_MOSI_I = OUT_PUT_MSB(byte); 	// output 'byte', MSB to MOSI
		byte = (byte << 1); 					// shift next bit into MSB..
		NRF24L01_SCK_I = 1; 					// Set SCK high..
		Delay_us(1);
		
		byte |= NRF24L01_MISO_I; 				// capture current MISO bit
		NRF24L01_SCK_I = 0; 					// ..then set SCK low again
		Delay_us(1);
	}
	
	return (byte); 								// return read byte
}


/*
�Ĵ������ʺ�������������24L01 �ļĴ�����ֵ������˼·����ͨ��
WRITE_REG ���Ҳ����0x20+�Ĵ�����ַ����Ҫ�趨��ֵд����Ӧ�ļĴ�����ַ����ȥ������
ȡ����ֵ�����ں�����˵Ҳ���ǰ�value ֵд��reg �Ĵ����С�
��Ҫע����ǣ�����NRF24L01 ֮ǰ����Ҫenable оƬ�� CSN=0���������������Ժ���disableоƬ�� CSN=1������
*/
u8 NRF24L01_I_SPI_RW_Reg(u8 cmd_and_reg, u8 value)
{
	u8 status;
	
	NRF24L01_CSN_I = 0; 								// CSN low, init SPI transaction
	status = NRF24L01_I_SPI_RW(cmd_and_reg); 			// select register
	NRF24L01_I_SPI_RW(value); 							// ..and write value to it..
	NRF24L01_CSN_I = 1; 								// CSN high again
	
	return (status); 									// return nRF24L01 status byte
}


/*
��ȡ�Ĵ���ֵ�ĺ���������˼·����ͨ��READ_REG ���Ҳ����0x00+�Ĵ�����ַ������
�Ĵ����е�ֵ�����������ں�����˵Ҳ���ǰ�reg �Ĵ�����ֵ����reg_val ��ȥ��
*/
u8 NRF24L01_I_SPI_Read(u8 cmd_and_reg)
{
	u8 reg_val;
	
	NRF24L01_CSN_I = 0; 						// CSN low, initialize SPI communication
	NRF24L01_I_SPI_RW(cmd_and_reg); 			// Select register to read from..
	reg_val = NRF24L01_I_SPI_RW(0); 			// ..then read registervalue
	NRF24L01_CSN_I = 1; 						// CSN high, terminate SPI communication
	
	return (reg_val); 							// return register value
}


/*
���ջ��������ʺ�������Ҫ�����ڽ���ʱ��ȡFIFO �������е�ֵ������˼
·����ͨ��READ_REG ��������ݴӽ���FIFO�� RD_RX_PLOAD���ж������浽��������ȥ��
*/
u8 NRF24L01_I_SPI_Read_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
	u8 status, byte_ctr;
	
	NRF24L01_CSN_I = 0; 						// Set CSN low, init SPI tranaction
	status = NRF24L01_I_SPI_RW(reg); 						// Select register to write to and read status byte
	
	for(byte_ctr = 0; byte_ctr < bytes; byte_ctr++)
	{
		pBuf[byte_ctr] = NRF24L01_I_SPI_RW(0); 			// Perform SPI_RW to read byte from nRF24L01
	}
	
	NRF24L01_CSN_I = 1; 						// Set CSN high again
	
	return (status); 							// return nRF24L01 status byte
}

/*
���仺�������ʺ�������Ҫ����������������ŵ�����FIFO�������С�
����˼·����ͨ��WRITE_REG ��������ݴ浽����FIFO��WR_TX_PLOAD����ȥ��
*/
u8 NRF24L01_I_SPI_Write_Buf(u8 reg, const u8 *pBuf, u8 bytes)
{
	u8 status,byte_ctr;
	
	NRF24L01_CSN_I = 0; 							// Set CSN low, init SPI tranaction
	status = NRF24L01_I_SPI_RW(reg); 				// Select register to write to and read status byte
	//Uart_Delay(10);
	Delay_us(10);
	
	for(byte_ctr = 0; byte_ctr < bytes; byte_ctr++) // then write all byte in buffer(*pBuf)
	{
		NRF24L01_I_SPI_RW(*pBuf++);
	}
	
	NRF24L01_CSN_I = 1; 							// Set CSN high again
	
	return (status); 								// return nRF24L01 status byte
}



/*
�ú�����ʼ��NRF24L01��TXģʽ
����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
PWR_UP,CRCʹ��
��CE��ߺ�,������RXģʽ,�����Խ���������		   
CEΪ�ߴ���10us,����������.
*/
void NRF24L01_I_TX_Mode(void)
{
	u8 temp = 0xff;
	
	NRF24L01_CE_I = 0;
	NRF24L01_I_SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);
	NRF24L01_I_SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, (u8*)TX_ADDRESS, TX_ADR_WIDTH);
	//SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			// Writes data to TX payload
	
#if 0
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01); 						// Enable Auto.Ack:Pipe0
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_AA);
	printf("EN_AA = %x\r\n", temp);
	temp = 0xff;

	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01); 					// Enable Pipe0
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("EN_RXADDR = %x\r\n", temp);
	temp = 0xff;

	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + SETUP_RETR, 0x1a); 					// 500us + 86us, 10 retrans...
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + SETUP_RETR);
	printf("SETUP_RETR = %x\r\n", temp);
	temp = 0xff;
	
#else

	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x00);
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_I_EN_AA = %x\r\n", temp);
	temp = 0xff;
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x00);
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("NRF24L01_I_EN_RXADDR = %x\r\n", temp);
	temp = 0xff;
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + SETUP_RETR, 0x00);
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + SETUP_RETR);
	printf("NRF24L01_I_SETUP_RETR = %x\r\n", temp);
	temp = 0xff;
	
#endif

	printf("\r\n");
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 40); 					// Select RF channel 40
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x0f); 				// TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0e); 				// Set PWR_UP bit, enable CRC(2 bytes) & Prim:TX. MAX_RT & TX_DS enabled..
	NRF24L01_CE_I = 1;
	
	Delay_us(100);
}



/*
�ú�����ʼ��NRF24L01��RXģʽ
����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
��CE��ߺ�,������RXģʽ,�����Խ���������
*/
void NRF24L01_I_RX_Mode(void)
{
	u8 temp = 0xff;
	
	NRF24L01_CE_I = 0;
	NRF24L01_I_SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, (u8*)TX_ADDRESS, TX_ADR_WIDTH);
	
#if 0
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01); 			// Enable Auto.Ack:Pipe0
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_AA);
	printf("EN_AA = %x\r\n", temp);
	temp = 0xff;
	
#else
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x00);
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_I_EN_AA = %x\r\n", temp);
	temp = 0xff;
	
#endif

	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01);		// Enable Pipe0
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("NRF24L01_I_EN_RXADDR = %x\r\n", temp);
	temp = 0xff;

	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 40); 				// Select RF channel 40
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RX_PW_P0, TX_PLOAD_WIDTH);
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x0f);
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0f); 			// Set PWR_UP bit, enable CRC(2 bytes) & Prim:RX. RX_DR enabled..
	NRF24L01_CE_I = 1; 									// Set CE pin high to enable RX device

	Delay_us(100);
	// This device is now ready to receive one packet of 16 bytes payload from a TX device sending to address
	// '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.
}


void NRF24L01_I_TransmitData(void)
{
	
}

void NRF24L01_I_ReceiveData(void)
{
	
}

/****************************************************************************************************************************/


/*
������ĺ��������GPIO ģ��SPI �Ĺ��ܡ�������ֽڣ� MOSI����MSB
ѭ�������ͬʱ�������ֽڣ� MISO����LSB ѭ�����롣�����ز������½������ݱ仯��
CPOL(ʱ�Ӽ���) = 0, CPHA(ʱ����λ) = 0;
SCKʱ�ӵĵ�һ����(����Ϊ������)��������λ��������SCK ����ʼ��Ϊ�͵�ƽ�����жϳ�����
*/
u8 NRF24L01_II_SPI_RW(u8 byte)
{
	u8 bit_ctr;
	
	for(bit_ctr = 0; bit_ctr < 8; bit_ctr++) 	// output 8-bit
	{
		NRF24L01_MOSI_II = OUT_PUT_MSB(byte); 	// output 'byte', MSB to MOSI
		byte = (byte << 1); 					// shift next bit into MSB..
		NRF24L01_SCK_II = 1; 					// Set SCK high..
		Delay_us(1);
		
		byte |= NRF24L01_MISO_II; 				// capture current MISO bit
		NRF24L01_SCK_II = 0; 					// ..then set SCK low again
		Delay_us(1);
	}
	
	return (byte); 								// return read byte
}


/*
�Ĵ������ʺ�������������24L01 �ļĴ�����ֵ������˼·����ͨ��
WRITE_REG ���Ҳ����0x20+�Ĵ�����ַ����Ҫ�趨��ֵд����Ӧ�ļĴ�����ַ����ȥ������
ȡ����ֵ�����ں�����˵Ҳ���ǰ�value ֵд��reg �Ĵ����С�
��Ҫע����ǣ�����NRF24L01 ֮ǰ����Ҫenable оƬ�� CSN=0���������������Ժ���disableоƬ�� CSN=1������
*/
u8 NRF24L01_II_SPI_RW_Reg(u8 cmd_and_reg, u8 value)
{
	u8 status;
	
	NRF24L01_CSN_II = 0; 								// CSN low, init SPI transaction
	status = NRF24L01_I_SPI_RW(cmd_and_reg); 			// select register
	NRF24L01_I_SPI_RW(value); 							// ..and write value to it..
	NRF24L01_CSN_II = 1; 								// CSN high again
	
	return (status); 									// return nRF24L01 status byte
}


/*
��ȡ�Ĵ���ֵ�ĺ���������˼·����ͨ��READ_REG ���Ҳ����0x00+�Ĵ�����ַ������
�Ĵ����е�ֵ�����������ں�����˵Ҳ���ǰ�reg �Ĵ�����ֵ����reg_val ��ȥ��
*/
u8 NRF24L01_II_SPI_Read(u8 cmd_and_reg)
{
	u8 reg_val;
	
	NRF24L01_CSN_II = 0; 						// CSN low, initialize SPI communication
	NRF24L01_I_SPI_RW(cmd_and_reg); 						// Select register to read from..
	reg_val = NRF24L01_I_SPI_RW(0); 						// ..then read registervalue
	NRF24L01_CSN_II = 1; 						// CSN high, terminate SPI communication
	
	return (reg_val); 							// return register value
}


/*
���ջ��������ʺ�������Ҫ�����ڽ���ʱ��ȡFIFO �������е�ֵ������˼
·����ͨ��READ_REG ��������ݴӽ���FIFO�� RD_RX_PLOAD���ж������浽��������ȥ��
*/
u8 NRF24L01_II_SPI_Read_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
	u8 status, byte_ctr;
	
	NRF24L01_CSN_II = 0; 						// Set CSN low, init SPI tranaction
	status = NRF24L01_I_SPI_RW(reg); 						// Select register to write to and read status byte
	
	for(byte_ctr = 0; byte_ctr < bytes; byte_ctr++)
	{
		pBuf[byte_ctr] = NRF24L01_I_SPI_RW(0); 			// Perform SPI_RW to read byte from nRF24L01
	}
	
	NRF24L01_CSN_II = 1; 						// Set CSN high again
	
	return (status); 							// return nRF24L01 status byte
}

/*
���仺�������ʺ�������Ҫ����������������ŵ�����FIFO�������С�
����˼·����ͨ��WRITE_REG ��������ݴ浽����FIFO��WR_TX_PLOAD����ȥ��
*/
u8 NRF24L01_II_SPI_Write_Buf(u8 reg, const u8 *pBuf, u8 bytes)
{
	u8 status,byte_ctr;
	
	NRF24L01_CSN_II = 0; 							// Set CSN low, init SPI tranaction
	status = NRF24L01_I_SPI_RW(reg); 				// Select register to write to and read status byte
	//Uart_Delay(10);
	Delay_us(10);
	
	for(byte_ctr = 0; byte_ctr < bytes; byte_ctr++) // then write all byte in buffer(*pBuf)
	{
		NRF24L01_I_SPI_RW(*pBuf++);
	}
	
	NRF24L01_CSN_II = 1; 							// Set CSN high again
	
	return (status); 								// return nRF24L01 status byte
}



/*
�ú�����ʼ��NRF24L01��TXģʽ
����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
PWR_UP,CRCʹ��
��CE��ߺ�,������RXģʽ,�����Խ���������		   
CEΪ�ߴ���10us,����������.
*/
void NRF24L01_II_TX_Mode(void)
{
	u8 temp = 0xff;
	
	NRF24L01_CE_II = 0;
	NRF24L01_I_SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	NRF24L01_I_SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
	//SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			// Writes data to TX payload
	
#if 0
	
	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01); 						// Enable Auto.Ack:Pipe0
	temp = SPI_Read(READ_REG_CMD + EN_AA);
	printf("EN_AA = %x\r\n", temp);
	temp = 0xff;

	SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01); 					// Enable Pipe0
	temp = SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("EN_RXADDR = %x\r\n", temp);
	temp = 0xff;

	SPI_RW_Reg(WRITE_REG_CMD + SETUP_RETR, 0x1a); 					// 500us + 86us, 10 retrans...
	temp = SPI_Read(READ_REG_CMD + SETUP_RETR);
	printf("SETUP_RETR = %x\r\n", temp);
	temp = 0xff;
	
#else

	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x00);
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_II_EN_AA = %x\r\n", temp);
	temp = 0xff;
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x00);
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("NRF24L01_II_EN_RXADDR = %x\r\n", temp);
	temp = 0xff;
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + SETUP_RETR, 0x00);
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + SETUP_RETR);
	printf("NRF24L01_II_SETUP_RETR = %x\r\n", temp);
	temp = 0xff;
	
#endif

	printf("\r\n");
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 40); 					// Select RF channel 40
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x0f); 				// TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0e); 				// Set PWR_UP bit, enable CRC(2 bytes) & Prim:TX. MAX_RT & TX_DS enabled..
	NRF24L01_CE_II = 1;
	
}



/*
�ú�����ʼ��NRF24L01��RXģʽ
����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
��CE��ߺ�,������RXģʽ,�����Խ���������
*/
void NRF24L01_II_RX_Mode(void)
{
	u8 temp = 0xff;
	
	NRF24L01_CE_II = 0;
	NRF24L01_I_SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
	
#if 0
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01); 			// Enable Auto.Ack:Pipe0
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_AA);
	printf("EN_AA = %x\r\n", temp);
	temp = 0xff;
	
#else
	
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x00);
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_II_EN_AA = %x\r\n", temp);
	temp = 0xff;
	
#endif

	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01);		// Enable Pipe0
	temp = NRF24L01_I_SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("NRF24L01_II_EN_RXADDR = %x\r\n", temp);
	temp = 0xff;

	printf("\r\n");

	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 40); 				// Select RF channel 40
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RX_PW_P0, TX_PLOAD_WIDTH);
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x0f);
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0f); 			// Set PWR_UP bit, enable CRC(2 bytes) & Prim:RX. RX_DR enabled..
	NRF24L01_CE_II = 1; 											// Set CE pin high to enable RX device
	
	// This device is now ready to receive one packet of 16 bytes payload from a TX device sending to address
	// '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.
}




/*
���24L01�Ƿ����
����ֵ:0���ɹ�;1��ʧ��	
*/
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	u8 i;
		 
	NRF24L01_I_SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, buf, 5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_I_SPI_Read_Buf(TX_ADDR , buf, 5); //����д��ĵ�ַ
	
	for(i=0;i<5;i++)
	{
		if(buf[i]!=0XA5)
		{
			return 1;				// ���24L01����	
		}	
	}
	
	return 0;		 				// ��⵽24L01
}	 	 

#if 1
/*
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
*/
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
	 
	NRF24L01_CE_I = 0;
  	NRF24L01_I_SPI_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);	//д���ݵ�TX BUF  32���ֽ�
 	NRF24L01_CE_I=1;											//��������
 	
	while(NRF24L01_IRQ_I != 0);									//�ȴ��������
	sta = NRF24L01_II_SPI_Read(STATUS);  						//��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + STATUS, sta); 		//���TX_DS��MAX_RT�жϱ�־
	if(sta & 0x10)//�ﵽ����ط�����
	{
		NRF24L01_I_SPI_RW_Reg(FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
		return 0x10; 
	}
	if(sta & 0x20)//�������
	{
		return 0x20;
	}
	return 0xff;//����ԭ����ʧ��
}

/*
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
*/
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;
	
	sta = NRF24L01_II_SPI_Read(STATUS);  									//��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_II_SPI_RW_Reg(WRITE_REG_CMD + STATUS, sta); 					//���TX_DS��MAX_RT�жϱ�־
	if(sta & 0x40)															//���յ�����
	{
		NRF24L01_II_SPI_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);		//��ȡ����
		NRF24L01_II_SPI_RW_Reg(FLUSH_RX, 0xff);								//���RX FIFO�Ĵ��� 
		
		return 0; 
	}
	
	return 1;																//û�յ��κ�����
}				
#endif

void NRF24L01_TEST(void)
{
	printf("NRF24L01_I_STATUS2 = %x\r\n", NRF24L01_I_SPI_Read(STATUS));
	printf("NRF24L01_I_FIFO_STATUS2 = %x\r\n", NRF24L01_I_SPI_Read(FIFO_STATUS));

	//NRF24L01_TxPacket((u8 *)tx_buf);
	NRF24L01_I_SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	//NRF24L01_I_SPI_Write_Buf(WR_TX_PLOAD, tx_buf, 1);
	Delay_ms(100);
	
	printf("NRF24L01_I_STATUS3 = %x\r\n", NRF24L01_I_SPI_Read(STATUS));
	printf("NRF24L01_I_FIFO_STATUS3 = %x\r\n", NRF24L01_I_SPI_Read(FIFO_STATUS));

	#if 1
	if(1 == NRF24L01_RxPacket((u8 *)rx_buf))
	{
		printf("no data\r\n");
	}
	else
	{
		printf("have data\r\n");
	}
	#endif
}

void NRF24L01_GPIO_INIT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	printf("NRF24L01_I_STATUS1 = %x\r\n", NRF24L01_I_SPI_Read(STATUS));
	
	NRF24L01_I_TX_Mode();
	
	NRF24L01_II_RX_Mode();
}



#endif



