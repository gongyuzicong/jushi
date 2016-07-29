#include "eeprom.h" 
#include "timer_opts.h"	


/*向指定地址写入一个字节*/
void EEPROM_Write_Byte(u8 addr,u8 data)
{
	IIC_Start();
	IIC_Send_Byte(0xa0);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(data);
	IIC_Wait_Ack();
	IIC_Stop();
	Delay_ms(10);
}
/*从指定地址读一个字节*/
u8 EEPROM_Read_Byte(u8 addr)
{
	u8 temp;
	IIC_Start();
	IIC_Send_Byte(0xa0);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0xa1);
	IIC_Wait_Ack();
	temp=IIC_Read_Byte(0);
	IIC_Stop();
	return temp;
}
/*从指定地址开始连续写入len个字节数据*/
void EEPROM_Write_Data(u8 addr, u8 *databuf, u16 len)
{
	u16 i;

	for(i=0;i<len;i++)
	{
		EEPROM_Write_Byte(addr+i,databuf[i]);
	}

}
/*从指定地址开始连续读取len字节数据*/
void EEPROM_Read_Data(u8 addr, u8 *databuf, u16 len)
{
	u16 i;
	for(i=0;i<len;i++)
	{
		databuf[i] = EEPROM_Read_Byte(addr+i);
	}

}






