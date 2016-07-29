#include "ads1256.h"


//***************************
//		Pin assign	   	
//			STM32			ADS1256		
//		GPIOC_Pin_0 		<--- DRDY
//		GPIOB_Pin_12 		---> CS
//		GPIOB_Pin_13 		---> SCK
//		GPIOB_Pin_14(MISO)  <--- DOUT
//		GPIOB_Pin_15(MOSI)  ---> DIN
//***************************	


//-----------------------------------------------------------------//
//	��    �ܣ�  SPIͨ��
//	��ڲ���: /	���͵�SPI����
//	���ڲ���: /	���յ�SPI����
//	ȫ�ֱ���: /
//	��    ע: 	���ͽ��պ���
//-----------------------------------------------------------------//
u8 SPI_WriteByte(u8 TxData)
{
	u8 RxData=0;
	
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==RESET);
	
	SPI_I2S_SendData(SPI2,TxData);
		
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)==RESET);
	
	RxData=SPI_I2S_ReceiveData(SPI2);
	
	return RxData;
} 


//-----------------------------------------------------------------//
//	��    �ܣ�ADS1256 д����
//	��ڲ���: /
//	���ڲ���: /
//	ȫ�ֱ���: /
//	��    ע: ��ADS1256�е�ַΪregaddr�ļĴ���д��һ���ֽ�databyte
//-----------------------------------------------------------------//
void ADS1256WREG(u8 regaddr, u8 databyte)
{
    
	CS_0();
	while(ADS1256_DRDY);//��ADS1256_DRDYΪ��ʱ����д�Ĵ���
	//��Ĵ���д�����ݵ�ַ
    SPI_WriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
    //д�����ݵĸ���n-1
    SPI_WriteByte(0x00);
    //��regaddr��ַָ��ļĴ���д������databyte
    SPI_WriteByte(databyte);
	CS_1();
}


//��ȡADֵ
s32 ADS1256ReadData(u8 channel)
{

    u32 sum = 0;
	
	while(ADS1256_DRDY);//��ADS1256_DRDYΪ��ʱ����д�Ĵ��� 
	ADS1256WREG(ADS1256_MUX, channel);		//����ͨ��
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SYNC);
	SPI_WriteByte(ADS1256_CMD_WAKEUP);	               
	SPI_WriteByte(ADS1256_CMD_RDATA);
   	sum |= (SPI_WriteByte(0xff) << 16);
	sum |= (SPI_WriteByte(0xff) << 8);
	sum |= SPI_WriteByte(0xff);
	CS_1();
	
	if (sum > 0x7FFFFF)           // if MSB=1, 
	{
		sum -= 0x1000000;       // do 2's complement

	}
	
    return sum;
}


void ADS1256_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_DRDY, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = PIN_DRDY; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);
	//SPI2 NSS 
	CS_1();
	GPIO_InitStructure.GPIO_Pin = PIN_CS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PORT_CS, &GPIO_InitStructure);
	 
}


void ADS1256_SPI2_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	/****Initial SPI2******************/

	/* Enable SPI2 and GPIOB clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	/* Configure SPI2 pins: NSS, SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI2 configuration */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 		//SPI1����Ϊ����ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					  		//����SPI2Ϊ��ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;						//SPI���ͽ���8λ֡�ṹ
	
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 				  				//����ʱ���ڲ�����ʱ��ʱ��Ϊ�͵�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;				  			//��һ��ʱ���ؿ�ʼ��������
	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;								//NSS�ź��������ʹ��SSIλ������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 	//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 	  					//���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;		  						//CRCֵ����Ķ���ʽ
	
	SPI_Init(SPI2, &SPI_InitStructure);
	/* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE);  
}


void ADS1256_Init(void)
{
	ADS1256_GPIO_Init();
	ADS1256_SPI2_Init();
}


