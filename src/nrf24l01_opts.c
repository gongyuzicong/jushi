#include "nrf24l01_opts.h"
#include "timer_opts.h"
#include "spi_opts.h"
#include "buffer.h"
#include "motion_control.h"

//NrfOptStruct nrfOpts;
//NrfOptStruct_P NRF24L01OptsPtr = &nrfOpts;
u8 need2SendInfo = 0;

u8 const TX_ADDRESS[TX_ADR_WIDTH] = {0x12, 0x34, 0x56, 0x78, 0x9A};	//本地地址
u8 const RX_ADDRESS[RX_ADR_WIDTH] = {0x12, 0x34, 0x56, 0x78, 0x9A};	//接收地址
u8 tx_buf[TX_PLOAD_WIDTH] = {0x03, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
u8 rx_buf[RX_PLOAD_WIDTH] = {0x00, 0x00, 0x00, 0x00, 0x00};


#ifdef NRF_USE_SPI

//从NRF读取一个字节数据
//reg 寄存器地址
u8 SPI_Read(u8 regAddr)
{
	u8 reg_val;
	
	NRF24L01_CSN = 0;                		//片选使能  
	
	SPI1_ReadWriteByte(regAddr);
	reg_val = SPI1_ReadWriteByte(0xff);    	// 读取数据到reg_val
	
	NRF24L01_CSN = 1;                		// 取消片选
	
	return(reg_val);        				// 返回读取的数据
}
   


//向NRF写入一个字节数据
//reg 寄存器地址  value 要写入的数据
u8 SPI_RW_Reg(u8 reg, u8 value)
{
	u8 status;
	
	NRF24L01_CSN = 0;                   	// CSN low, init SPI transaction
	status = SPI1_ReadWriteByte(reg);
    SPI1_ReadWriteByte(value);
	NRF24L01_CSN = 1;                   	// CSN high again
		
	return(status);            				// return nRF24L01 status uchar
}



//从NRF读取多个字节数据
//reg 寄存器地址  *pBuf 读取数据存储指针  uchars 读取的字节个数
u8 SPI_Read_Buf(u8 reg, u8 *pBuf, u8 uchars)
{
	u8 status,uchar_ctr;
	
	NRF24L01_CSN = 0;                    		// Set CSN low, init SPI tranaction

	status = SPI1_ReadWriteByte(reg);
	for(uchar_ctr = 0; uchar_ctr < uchars; uchar_ctr++)		  	//循环 uchars次
	{
    	pBuf[uchar_ctr] = SPI1_ReadWriteByte(0xff); 			//分别将SPI_RW(0)读出的数据地址 放入数组中
	}
	
	NRF24L01_CSN = 1;                           
	
	return status;                    // return nRF24L01 status uchar
}


//向NRF写入多个字节数据
//reg 寄存器地址  *pBuf 要写入的数据  uchars 写入的字节个数
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 uchars)
{
	u8 status,uchar_ctr;
	
	NRF24L01_CSN = 0;            // SPI使能       

	status = SPI1_ReadWriteByte(reg);
	
	for(uchar_ctr=0; uchar_ctr < uchars; uchar_ctr++) 		// 根据数据个数循环	
	{
		SPI1_ReadWriteByte(*pBuf++);						// 将数组的数据 依次写入
	}

	NRF24L01_CSN = 1;           //关闭SPI
	return(status);    // 
}


//检测24L01是否存在
//返回值:0，成功;1，失败	
u8 NRF24L01_Check(void)
{
	u8 buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	u8 i;
	
    //SPI1_SetSpeed(SPI_BaudRatePrescaler_8);       //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   	 
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, buf, 5);   //写入5个字节的地址.	
	SPI_Read_Buf(TX_ADDR, buf, 5); //读出写入的地址  
	
	for(i=0;i<5;i++)
	{
		if(buf[i] != 0xA5)
		{
			break;
		}
	}
	
	if(i != 5)
	{
		return 1;//检测24L01错误	
	}
	
	return 0;		 //检测到24L01
}


//NRF24L01初始化
//m 1 发送模式   0 接收模式
void TX_Mode(void)	        //接收 or 发射 模式 初始化
{
	u8 temp = 0;
	
 	NRF24L01_CE = 0;    // chip enable
 	
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址	
	SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址
	
	SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x0f);   		//设置发射速率为2MHZ，发射功率为最大值0dB
	SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 0);        //   设置信道工作为2.4GHZ，收发必须一致
	SPI_RW_Reg(WRITE_REG_CMD + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
	
	#if 1

	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01);      //  频道0自动	ACK应答允许	
	temp = SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_EN_AA = %x\r\n", temp);
	temp = 0xff;
	
	SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01);  //  允许接收地址只有频道0，如果需要多频道可以参考Page21  
	temp = SPI_Read(READ_REG_CMD + EN_RXADDR);
	printf("NRF24L01_EN_RXADDR = %x\r\n", temp);
	temp = 0xff;
	
	SPI_RW_Reg(WRITE_REG_CMD + SETUP_RETR, 0x1a);	// 自动重发10次, 间隔500us
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
	
	SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送

	NRF24L01_CE = 1;
}

void Change_To_TX_Mode_Fast(void)
{
	NRF24L01_CE = 0;    // chip enable
 	
	SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送

	NRF24L01_CE = 1;
}

void RX_Mode(void)
{
	u8 temp = 0;
	
	NRF24L01_CE = 0;	// chip enable
	
	SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);	  // 写本地地址 
	SPI_Write_Buf(WRITE_REG_CMD + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址
	SPI_RW_Reg(WRITE_REG_CMD + EN_RXADDR, 0x01);			//	允许接收地址只有频道0，如果需要多频道可以参考Page21  
	SPI_RW_Reg(WRITE_REG_CMD + RF_SETUP, 0x0f); 			//设置发射速率为2MHZ，发射功率为最大值0dB
	SPI_RW_Reg(WRITE_REG_CMD + RF_CH, 0);		 			//   设置信道工作为2.4GHZ，收发必须一致
	SPI_RW_Reg(WRITE_REG_CMD + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
	
	#if 1

	
	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x01);				//	频道0自动	ACK应答允许 
	temp = SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_EN_AA = %x\r\n\r\n", temp);
	
	#else
	
	SPI_RW_Reg(WRITE_REG_CMD + EN_AA, 0x00);
	temp = SPI_Read(READ_REG_CMD + EN_AA);
	printf("NRF24L01_EN_AA = %x\r\n", temp);
	//temp = 0xff;
	
	#endif
	
	SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0f);			// IRQ收发完成中断响应，16位CRC，主接收
	
	NRF24L01_CE = 1;
}

void Change_To_RX_Mode_Fast(void)
{
	NRF24L01_CE = 0;    // chip enable
 	//NRF24L01_CSN = 1;   // Spi disable 
 	
	SPI_RW_Reg(WRITE_REG_CMD + CONFIG, 0x0f);   		 // IRQ收发完成中断响应，16位CRC，主发送

	NRF24L01_CE = 1;
}

//NRF接收数据函数
//rx_buf  数据缓存区
//该函数检测NRF状态寄存器状态 当有中断立即接收数据到rx_buf缓存区
u8 nRF24L01_RxPacket(u8 *rx_buf)
{	 
    u8 sta;
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz） 

	sta = SPI_Read(READ_REG_CMD + STATUS);	    				// 读取状态寄存器来判断数据接收状况
	SPI_RW_Reg(WRITE_REG_CMD + STATUS, sta);   //清中断 （接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志）
	if(sta & RX_OK)								// 判断是否接收到数据
	{

		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		SPI_RW_Reg(FLUSH_RX,0xff);

		return 0; 
	}
	
	return 1;
}


//将缓存区tx_buf中的数据发送出去
//tx_buf  要发送的数据缓存区
u8 nRF24L01_TxPacket(u8 *tx_buf)
{	 
    u8 st;
	
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz） 
	
	NRF24L01_CSN = 0;			//StandBy I模式	

	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据	

	NRF24L01_CSN = 1;		 				//置高CE，激发数据发送
	
	while(NRF24L01_IRQ != 0);		   //等待发送完成
	
	st=	SPI_Read(STATUS);			  //读NRF寄存器状态
	SPI_RW_Reg(WRITE_REG_CMD + STATUS, st);  //清中断

	if(st & MAX_TX)//达到最大重发次数
	{
		SPI_RW_Reg(FLUSH_TX, 0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}

	if(st & TX_OK)				 //发送成功
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
	
	
	SPI_Read_Buf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);	//读取数据

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

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;  // PC0 1 推挽 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      		//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);			  //上拉 取消SPI总线片选

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	      //PC2 下拉输入  该IO判断 NRF是否有低电平信号
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void NRF24L01_SPI_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	
	SPI_Cmd(SPI1, DISABLE); // SPI外设不使能			  NRF的SPI要特殊配置一下

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI主机
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	//发送接收8位帧结构

	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;			//时钟悬空低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;		//数据捕获于第1个时钟沿

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			//NSS信号由软件控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;			//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  				//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); 								//使能SPI外设
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
最基本的函数，完成GPIO 模拟SPI 的功能。将输出字节（ MOSI）从MSB
循环输出，同时将输入字节（ MISO）从LSB 循环移入。上升沿采样，下降沿数据变化。
CPOL(时钟极性) = 0, CPHA(时钟相位) = 0;
SCK时钟的第一边沿(这里为上升沿)进行数据位采样（从SCK 被初始化为低电平可以判断出）。
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
寄存器访问函数：用来设置24L01 的寄存器的值。基本思路就是通过
WRITE_REG 命令（也就是0x20+寄存器地址）把要设定的值写到相应的寄存器地址里面去，并读
取返回值。对于函数来说也就是把value 值写到reg 寄存器中。
需要注意的是，访问NRF24L01 之前首先要enable 芯片（ CSN=0；），访问完了以后再disable芯片（ CSN=1；）。
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
读取寄存器值的函数：基本思路就是通过READ_REG 命令（也就是0x00+寄存器地址），把
寄存器中的值读出来。对于函数来说也就是把reg 寄存器的值读到reg_val 中去。
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
接收缓冲区访问函数：主要用来在接收时读取FIFO 缓冲区中的值。基本思
路就是通过READ_REG 命令把数据从接收FIFO（ RD_RX_PLOAD）中读出并存到数组里面去。
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
发射缓冲区访问函数：主要用来把数组里的数放到发射FIFO缓冲区中。
基本思路就是通过WRITE_REG 命令把数据存到发射FIFO（WR_TX_PLOAD）中去。
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
该函数初始化NRF24L01到TX模式
设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
PWR_UP,CRC使能
当CE变高后,即进入RX模式,并可以接收数据了		   
CE为高大于10us,则启动发送.
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
该函数初始化NRF24L01到RX模式
设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
当CE变高后,即进入RX模式,并可以接收数据了
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
最基本的函数，完成GPIO 模拟SPI 的功能。将输出字节（ MOSI）从MSB
循环输出，同时将输入字节（ MISO）从LSB 循环移入。上升沿采样，下降沿数据变化。
CPOL(时钟极性) = 0, CPHA(时钟相位) = 0;
SCK时钟的第一边沿(这里为上升沿)进行数据位采样（从SCK 被初始化为低电平可以判断出）。
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
寄存器访问函数：用来设置24L01 的寄存器的值。基本思路就是通过
WRITE_REG 命令（也就是0x20+寄存器地址）把要设定的值写到相应的寄存器地址里面去，并读
取返回值。对于函数来说也就是把value 值写到reg 寄存器中。
需要注意的是，访问NRF24L01 之前首先要enable 芯片（ CSN=0；），访问完了以后再disable芯片（ CSN=1；）。
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
读取寄存器值的函数：基本思路就是通过READ_REG 命令（也就是0x00+寄存器地址），把
寄存器中的值读出来。对于函数来说也就是把reg 寄存器的值读到reg_val 中去。
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
接收缓冲区访问函数：主要用来在接收时读取FIFO 缓冲区中的值。基本思
路就是通过READ_REG 命令把数据从接收FIFO（ RD_RX_PLOAD）中读出并存到数组里面去。
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
发射缓冲区访问函数：主要用来把数组里的数放到发射FIFO缓冲区中。
基本思路就是通过WRITE_REG 命令把数据存到发射FIFO（WR_TX_PLOAD）中去。
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
该函数初始化NRF24L01到TX模式
设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
PWR_UP,CRC使能
当CE变高后,即进入RX模式,并可以接收数据了		   
CE为高大于10us,则启动发送.
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
该函数初始化NRF24L01到RX模式
设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
当CE变高后,即进入RX模式,并可以接收数据了
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
检测24L01是否存在
返回值:0，成功;1，失败	
*/
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	u8 i;
		 
	NRF24L01_I_SPI_Write_Buf(WRITE_REG_CMD + TX_ADDR, buf, 5);//写入5个字节的地址.	
	NRF24L01_I_SPI_Read_Buf(TX_ADDR , buf, 5); //读出写入的地址
	
	for(i=0;i<5;i++)
	{
		if(buf[i]!=0XA5)
		{
			return 1;				// 检测24L01错误	
		}	
	}
	
	return 0;		 				// 检测到24L01
}	 	 

#if 1
/*
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
*/
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
	 
	NRF24L01_CE_I = 0;
  	NRF24L01_I_SPI_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);	//写数据到TX BUF  32个字节
 	NRF24L01_CE_I=1;											//启动发送
 	
	while(NRF24L01_IRQ_I != 0);									//等待发送完成
	sta = NRF24L01_II_SPI_Read(STATUS);  						//读取状态寄存器的值	   
	NRF24L01_I_SPI_RW_Reg(WRITE_REG_CMD + STATUS, sta); 		//清除TX_DS或MAX_RT中断标志
	if(sta & 0x10)//达到最大重发次数
	{
		NRF24L01_I_SPI_RW_Reg(FLUSH_TX, 0xff);//清除TX FIFO寄存器 
		return 0x10; 
	}
	if(sta & 0x20)//发送完成
	{
		return 0x20;
	}
	return 0xff;//其他原因发送失败
}

/*
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
*/
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;
	
	sta = NRF24L01_II_SPI_Read(STATUS);  									//读取状态寄存器的值    	 
	NRF24L01_II_SPI_RW_Reg(WRITE_REG_CMD + STATUS, sta); 					//清除TX_DS或MAX_RT中断标志
	if(sta & 0x40)															//接收到数据
	{
		NRF24L01_II_SPI_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);		//读取数据
		NRF24L01_II_SPI_RW_Reg(FLUSH_RX, 0xff);								//清除RX FIFO寄存器 
		
		return 0; 
	}
	
	return 1;																//没收到任何数据
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



