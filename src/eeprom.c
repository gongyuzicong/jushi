#include "eeprom.h" 
#include "timer_opts.h"	


/*向指定地址写入一个字节*/
void EEPROM_Write_Byte(u16 addr,u8 data)
{
	IIC_Start();
	IIC_Send_Byte(0xa0);			// 设备地址(ID)
	IIC_Wait_Ack();
	IIC_Send_Byte(addr >> 8);		// 写入地址
	IIC_Wait_Ack();
	IIC_Send_Byte(addr & 0x00FF);	// 写入地址
	IIC_Wait_Ack();
	IIC_Send_Byte(data);
	IIC_Wait_Ack();
	IIC_Stop();
	Delay_ms(10);
}
/*从指定地址读一个字节*/
u8 EEPROM_Read_Byte(u16 addr)
{
	u8 temp;
	IIC_Start();
	IIC_Send_Byte(0xa0);
	IIC_Wait_Ack();
	IIC_Send_Byte( addr >> 8 );
	IIC_Wait_Ack();
	IIC_Send_Byte( addr & 0x00FF );
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte( 0xa1 );
	IIC_Wait_Ack();
	temp = IIC_Read_Byte(0);
	IIC_Stop();
	return temp;
}

#if 0
/*从指定地址开始连续写入len个字节数据*/
void EEPROM_Write_Data(u16 addr, u8 *databuf, u16 len)
{
	u16 i;

	for(i = 0; i < len; i++)
	{
		EEPROM_Write_Byte(addr + i, databuf[i]);
	}

}
/*从指定地址开始连续读取len字节数据*/
void EEPROM_Read_Data(u16 addr, u8 *databuf, u16 len)
{
	u16 i;
	
	for(i = 0; i < len; i++)
	{
		databuf[i] = EEPROM_Read_Byte(addr + i);
	}

}
#else

/*从指定地址开始连续写入len个字节数据*/
void EEPROM_Write_Data(u16 addr, u8 *databuf, u16 len)
{
	u16 i;

	for(i = 0; i < len; i++)
	{
		EEPROM_Write_Byte(addr + i, *(databuf + i));
	}

}
/*从指定地址开始连续读取len字节数据*/
void EEPROM_Read_Data(u16 addr, u8 *databuf, u16 len)
{
	u16 i;
	
	for(i = 0; i < len; i++)
	{
		*(databuf + i) = EEPROM_Read_Byte(addr + i);
	}

}


#endif

void Clean_EEPROM(void)
{
	u16 cir = 0;

	for(cir = 0; cir <= 0xFFFF; cir++)
	{
		EEPROM_Write_Byte(cir, 0x00);
	}
	
}

#if 0

/*向指定地址写入一个字节*/
void EEPROM_Write_Byte2(u16 addr,u8 data)
{
	I2C_Start();
	I2C_Send_Byte(0xa0);
	I2C_Wait_ACK();
	I2C_Send_Byte(addr >> 8);
	I2C_Wait_ACK();
	I2C_Send_Byte(addr & 0x00FF);
	I2C_Wait_ACK();
	I2C_Send_Byte(data);
	I2C_Wait_ACK();
	I2C_Stop();
	Delay_ms(10);
}

/*从指定地址读一个字节*/
u8 EEPROM_Read_Byte2(u16 addr)
{
	u8 temp;
	I2C_Start();
	I2C_Send_Byte(0xa0);
	I2C_Wait_ACK();
	I2C_Send_Byte(addr >> 8);
	I2C_Wait_ACK();
	I2C_Send_Byte(addr & 0x00FF);
	I2C_Wait_ACK();
	I2C_Start();
	I2C_Send_Byte(0xa1);
	I2C_Wait_ACK();
	temp = I2C_Read_Byte(0);
	I2C_Stop();
	return temp;
}

#endif


