#ifndef __RTC_H
#define __RTC_H


#include "iic.h"   

#define BCD_CHANGE2HEX_U8(u8_data)		(((u8_data) & 0x0F) + (((u8_data) >> 4) * 10))
#define HEX_CHANGE2BCD_U8(u8_data)		((((u8_data) / 10) << 4) | ((u8_data) % 10))

#define USE_NEW_RTC 1

typedef struct
{
	u8 year;
	u8 month;
	u8 week;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
}DateInfo, *DateInfo_P;


#if USE_NEW_RTC

typedef struct RTC_Info_Struct
{
	DateInfo date;

	void (*Set_RTC)(struct RTC_Info_Struct *);
	void (*Read_RTC)(DateInfo_P);
}RTC_Info_Str, *RTC_Info_Str_P;

void SetDate(void);
void RTC_PCF8563_Init(void);


extern RTC_Info_Str_P RTC_PCF8563_Ptr;

#endif



#endif
















