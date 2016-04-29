#include "nrf24l01_opts.h"
#include "timer_opts.h"
#include "spi_opts.h"


#ifdef NRF_USE_SPI

u8 const TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//���ص�ַ
u8 const RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//���յ�ַ

//��NRF��ȡһ���ֽ�����
//reg �Ĵ�����ַ
u8 SPI_Read(u8 reg)
{
	u8 reg_val;
	
	NRF24L01_CSN = 0;                //Ƭѡʹ��  
	
    SPI1_ReadWriteByte(reg);
	reg_val = SPI1_ReadWriteByte(0xff);    // ��ȡ���ݵ�reg_val
	
	NRF24L01_CSN = 1;                // ȡ��Ƭѡ
	
	return(reg_val);        // ���ض�ȡ������
}
   


//��NRFд��һ���ֽ�����
//reg �Ĵ�����ַ  value Ҫд�������
u8 SPI_RW_Reg(u8 reg, u8 value)
{
	u8 status;
	
	NRF24L01_CSN = 0;                   // CSN low, init SPI transaction
	status = SPI1_ReadWriteByte(reg);
    SPI1_ReadWriteByte(value);
	NRF24L01_CSN = 1;                   // CSN high again
		
	return(status);            // return nRF24L01 status uchar
}



//��NRF��ȡ����ֽ�����
//reg �Ĵ�����ַ  *pBuf ��ȡ���ݴ洢ָ��  uchars ��ȡ���ֽڸ���
u8 SPI_Read_Buf(u8 reg, u8 *pBuf, u8 uchars)
{
	u8 status,uchar_ctr;
	
	NRF24L01_CSN = 0;                    		// Set CSN low, init SPI tranaction

	status = SPI1_ReadWriteByte(reg);
	for(uchar_ctr=0;uchar_ctr<uchars;uchar_ctr++)		  //ѭ�� uchars��
	{
    	pBuf[uchar_ctr] = SPI1_ReadWriteByte(0xff); 				  //�ֱ�	 SPI_RW(0)���������ݵ�ַ ����������
	}
	
	NRF24L01_CSN = 1;                           
	
	return status;                    // return nRF24L01 status uchar
}


//��NRFд�����ֽ�����
//reg �Ĵ�����ַ  *pBuf Ҫд�������  uchars д����ֽڸ���
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 uchars)
{
	u8 status,uchar_ctr;
	
	NRF24L01_CSN = 0;            //SPIʹ��       

	status = SPI1_ReadWriteByte(reg);
	
	for(uchar_ctr=0; uchar_ctr<uchars; uchar_ctr++) //	 �������ݸ���ѭ��	
	{
		SPI1_ReadWriteByte(*pBuf++);						//����������� ����д��
	}

	NRF24L01_CSN = 1;           //�ر�SPI
	return(status);    // 
}


//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 NRF24L01_Check(void)
{
	u8 buf[5] = {0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
	
    SPI1_SetSpeed(SPI_BaudRatePrescaler_8);       //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, buf, 5);   //д��5���ֽڵĵ�ַ.	
	SPI_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	
	for(i=0;i<5;i++)
	{
		if(buf[i]!=0XA5)
		{
			break;
		}
	}
	
	if(i!=5)
	{
		return 1;//���24L01����	
	}
	
	return 0;		 //��⵽24L01
}

//NRF�������ݺ���
//rx_buf  ���ݻ�����
//�ú������NRF״̬�Ĵ���״̬ �����ж������������ݵ�rx_buf������
u8 nRF24L01_RxPacket(u8 *rx_buf)
{	 
    u8 sta;
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz�� 

	sta = SPI_Read(STATUS);	    				// ��ȡ״̬�Ĵ������ж����ݽ���״��
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

	NRF24L01_CSN=1;		 				//�ø�CE���������ݷ���
	
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


//NRF24L01��ʼ��
//m 1 ����ģʽ   0 ����ģʽ
void RX_TX_Mode(u8 m)	        //���� or ���� ģʽ ��ʼ��
{
 	NRF24L01_CE = 0;    // chip enable
 	NRF24L01_CSN = 1;   // Spi disable 
 	
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);    // д���ص�ַ	
	SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // д���ն˵�ַ
	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01);      //  Ƶ��0�Զ�	ACKӦ������	
	SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01);  //  ������յ�ַֻ��Ƶ��0�������Ҫ��Ƶ�����Բο�Page21  
	SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 0);        //   �����ŵ�����Ϊ2.4GHZ���շ�����һ��
	SPI_RW_Reg(WRITE_REG_CMD + RX_PW_P0, RX_PLOAD_WIDTH); //���ý������ݳ��ȣ���������Ϊ32�ֽ�
	SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x07);   		//���÷�������Ϊ1MHZ�����书��Ϊ���ֵ0dB

	if(m == 1)
	{
		SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0e);   		 // IRQ�շ�����ж���Ӧ��16λCRC��������
	}
	else if(m == 0)
	{
		SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0f);   		// IRQ�շ�����ж���Ӧ��16λCRC	��������
	}

	NRF24L01_CE = 1;
}



void NRF24L01_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  // PC0 1 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1);			  //���� ȡ��SPI����Ƭѡ

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	      //PC2 ��������  ��IO�ж� NRF�Ƿ��иߵ�ƽ�ź�
    GPIO_Init(GPIOC, &GPIO_InitStructure);
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
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;			//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  				//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); 								//ʹ��SPI����
}

void NFR24L01_Init(void)
{
	NRF24L01_GPIO_Init();
	NRF24L01_SPI_Init();

	NRF24L01_CE = 0;
	NRF24L01_CSN = 1;
}

#else

u8 const TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; // Define a static TX address
u8 rx_buf[RX_PLOAD_WIDTH];
u8 tx_buf[TX_PLOAD_WIDTH] = {0x02, 0x02, 0x03, 0x05, 0x09};

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



