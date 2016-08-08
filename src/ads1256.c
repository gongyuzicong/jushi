#include "ads1256.h"
#include "timer_opts.h"


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
//	功    能：  SPI通信
//	入口参数: /	发送的SPI数据
//	出口参数: /	接收的SPI数据
//	全局变量: /
//	备    注: 	发送接收函数
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
//	功    能：ADS1256 写数据
//	入口参数: /
//	出口参数: /
//	全局变量: /
//	备    注: 向ADS1256中地址为regaddr的寄存器写入一个字节databyte
//-----------------------------------------------------------------//
void ADS1256WREG(u8 regaddr, u8 databyte)
{
    
	CS_0();
	while(ADS1256_DRDY);//当ADS1256_DRDY为低时才能写寄存器
	//向寄存器写入数据地址
    SPI_WriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
    //写入数据的个数n-1
    SPI_WriteByte(0x00);
    //向regaddr地址指向的寄存器写入数据databyte
    SPI_WriteByte(databyte);
	CS_1();
}


//读取AD值
s32 ADS1256ReadData(u8 channel)
{

    u32 sum = 0;
	
	while(ADS1256_DRDY);//当ADS1256_DRDY为低时才能写寄存器 
	ADS1256WREG(ADS1256_MUX, channel);		//设置通道
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


//初始化ADS1256
void ADS1256_Auto_Init(void)
{
	//*************自校准****************
   	while(ADS1256_DRDY);
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SELFCAL);
	while(ADS1256_DRDY);
	CS_1();
	//**********************************

	ADS1256WREG(ADS1256_STATUS,0x06);               // 高位在前、使用缓冲
//	ADS1256WREG(ADS1256_STATUS,0x04);               // 高位在前、不使用缓冲

//	ADS1256WREG(ADS1256_MUX,0x08);                  // 初始化端口A0为‘+’，AINCOM位‘-’
	ADS1256WREG(ADS1256_ADCON,ADS1256_GAIN_1);                // 放大倍数1
	ADS1256WREG(ADS1256_DRATE,ADS1256_DRATE_10SPS);  // 数据10sps
	ADS1256WREG(ADS1256_IO,0x00);               

	//*************自校准****************
	while(ADS1256_DRDY);
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SELFCAL);
	while(ADS1256_DRDY);
	CS_1(); 
	//**********************************
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
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 		//SPI1设置为两线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					  		//设置SPI2为主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;						//SPI发送接收8位帧结构
	
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 				  				//串行时钟在不操作时，时钟为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;				  			//第一个时钟沿开始采样数据
	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;								//NSS信号由软件（使用SSI位）管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 	//定义波特率预分频的值:波特率预分频值为8
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 	  					//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;		  						//CRC值计算的多项式
	
	SPI_Init(SPI2, &SPI_InitStructure);
	/* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE);  
}


void ADS1256_Init(void)
{
	ADS1256_GPIO_Init();
	ADS1256_SPI2_Init();
	Delay_ms(100);
	ADS1256_Auto_Init();
}


