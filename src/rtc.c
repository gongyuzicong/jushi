#include "rtc.h" 



void SET_date(u8 *date)//设定日期
{
	IIC_Start();
	IIC_Send_Byte(0xa2);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x05);
	IIC_Wait_Ack();
	IIC_Send_Byte(date[2]);		// 
	IIC_Wait_Ack();
	IIC_Send_Byte(0x00);
	IIC_Wait_Ack();
	IIC_Send_Byte(date[1]);
	IIC_Wait_Ack();
	IIC_Send_Byte(date[0]);
	IIC_Wait_Ack();
	IIC_Stop();
}
void SET_time(u8 *time)//设定时间
{
	IIC_Start();
	IIC_Send_Byte(0xa2);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x02);
	IIC_Wait_Ack();
	IIC_Send_Byte(time[2]);
	IIC_Wait_Ack();
	IIC_Send_Byte(time[1]);
	IIC_Wait_Ack();
	IIC_Send_Byte(time[0]);
	IIC_Wait_Ack();
	IIC_Stop();
}

void READ_datetime(u8 *date,u8 *time)//读取日期和时间
{
		IIC_Start();
		IIC_Send_Byte(0xa2);
		IIC_Wait_Ack();
		IIC_Send_Byte(0x02);
		IIC_Wait_Ack();
		IIC_Start();
		IIC_Send_Byte(0xa3);
		IIC_Wait_Ack();
		time[2]=IIC_Read_Byte(1);
		time[1]=IIC_Read_Byte(1);
		time[0]=IIC_Read_Byte(1);
		date[2]=IIC_Read_Byte(1);
		IIC_Read_Byte(1);
		date[1]=IIC_Read_Byte(1);
		date[0]=IIC_Read_Byte(0);
		IIC_Stop();
}



void UartSetHandle(u8 data)
{
	
}




