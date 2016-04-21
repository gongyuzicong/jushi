#include "nrf24l01_opts.h"
#include "timer_opts.h"


u8 const TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; // Define a static TX address
u8 rx_buf[RX_PLOAD_WIDTH];
u8 tx_buf[TX_PLOAD_WIDTH] = {0x02, 0x02, 0x03, 0x05, 0x09};


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



