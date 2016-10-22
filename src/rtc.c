#include "rtc.h" 

#if USE_NEW_RTC


RTC_Info_Str RTC_PCF8563;
RTC_Info_Str_P RTC_PCF8563_Ptr = &RTC_PCF8563;


void PCF8563_Set_Date(u8 year, u8 month, u8 week, u8 day)//设定日期
{
	IIC_Start();
	IIC_Send_Byte(0xa2);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x05);
	IIC_Wait_Ack();
	IIC_Send_Byte(day);
	IIC_Wait_Ack();
	IIC_Send_Byte(week);
	IIC_Wait_Ack();
	IIC_Send_Byte(month);
	IIC_Wait_Ack();
	IIC_Send_Byte(year);
	IIC_Wait_Ack();
	IIC_Stop();
}


void PCF8563_Set_Time(u8 hour, u8 minute, u8 second)//设定时间
{
	IIC_Start();
	IIC_Send_Byte(0xa2);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x02);
	IIC_Wait_Ack();
	IIC_Send_Byte(second);
	IIC_Wait_Ack();
	IIC_Send_Byte(minute);
	IIC_Wait_Ack();
	IIC_Send_Byte(hour);
	IIC_Wait_Ack();
	IIC_Stop();
}

void PCF8563_Set_All(RTC_Info_Str_P ptr)
{
	IIC_Start();
	IIC_Send_Byte(0xa2);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x02);
	IIC_Wait_Ack();
	IIC_Send_Byte(ptr->date.second);
	IIC_Wait_Ack();
	IIC_Send_Byte(ptr->date.minute);
	IIC_Wait_Ack();
	IIC_Send_Byte(ptr->date.hour);
	IIC_Wait_Ack();
	IIC_Send_Byte(ptr->date.day);
	IIC_Wait_Ack();
	IIC_Send_Byte(ptr->date.week);
	IIC_Wait_Ack();
	IIC_Send_Byte(ptr->date.month);
	IIC_Wait_Ack();
	IIC_Send_Byte(ptr->date.year);
	IIC_Wait_Ack();
	IIC_Stop();

}

void Set_Rtc_PCF8563(RTC_Info_Str_P ptr)
{
	PCF8563_Set_All(ptr);
}

void SetDate(void)
{
	RTC_PCF8563_Ptr->date.year = 0x16;
	RTC_PCF8563_Ptr->date.month = 0x10;
	RTC_PCF8563_Ptr->date.week = 0x05;
	RTC_PCF8563_Ptr->date.day = 0x21;
	RTC_PCF8563_Ptr->date.hour = 0x09;
	RTC_PCF8563_Ptr->date.minute = 0x41;
	RTC_PCF8563_Ptr->date.second = 0x00;

	RTC_PCF8563_Ptr->Set_RTC(RTC_PCF8563_Ptr);
	
}

void RTC_PCF8563_Read(DateInfo_P ptr)
{	
	IIC_Start();
	IIC_Send_Byte(0xa2);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x02);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0xa3);
	IIC_Wait_Ack();
	ptr->second = (IIC_Read_Byte(1) & 0x7F);
	ptr->minute = (IIC_Read_Byte(1) & 0x7F);
	ptr->hour = (IIC_Read_Byte(1) & 0x3F);
	ptr->day = (IIC_Read_Byte(1) & 0x3F);
	ptr->week = (IIC_Read_Byte(1) & 0x7F);
	ptr->month = (IIC_Read_Byte(1) & 0x1F);
	ptr->year = (IIC_Read_Byte(0) & 0x7F);
	IIC_Stop();
		
}

void RTC_PCF8563_Init(void)
{

	RTC_PCF8563_Ptr->date.year = 0x00;
	RTC_PCF8563_Ptr->date.month = 0x00;
	RTC_PCF8563_Ptr->date.week = 0x00;
	RTC_PCF8563_Ptr->date.day = 0x00;

	RTC_PCF8563_Ptr->date.hour = 0x00;
	RTC_PCF8563_Ptr->date.minute = 0x00;
	RTC_PCF8563_Ptr->date.second = 0x00;

	RTC_PCF8563_Ptr->Set_RTC = Set_Rtc_PCF8563;
	RTC_PCF8563_Ptr->Read_RTC = RTC_PCF8563_Read;
	
}



#endif





