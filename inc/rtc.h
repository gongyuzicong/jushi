#ifndef __RTC_H
#define __RTC_H


#include "iic.h"   

#define BCD_CHANGE2HEX_U8(u8_data)		(((u8_data) & 0x0F) + (((u8_data) >> 4) * 10))
#define HEX_CHANGE2BCD_U8(u8_data)		((((u8_data) / 10) << 4) | ((u8_data) % 10))


void SET_date(u8 *date);//设定日期
void SET_time(u8 *time);//设定时间
void READ_datetime(u8 *date,u8 *time);//读取日期和时间




#endif
















