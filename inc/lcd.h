#ifndef _LCD_H_
#define _LCD_H_

#include "common_include.h"


void Uart_Send_Char(u8 data);
void Uart_Send_Str(u8* str);


//设置日期//
void Set_date(u8 y,u8 m,u8 d);
//设置时间//
void Set_time(u8 h,u8 m,u8 s);
//设置电量//
void Set_batter(u8 batter);
//更新主页任务状态//
void Set_main_state(u16 *buf);
//更新称重//
void Set_scale_weight(u8 z,u8 x);
void UART1_REC(u8);

//处理从LCD接受的数据//
void Lcd_Handle(void);
void LCD_INIT(void);
void Weight_Screen_Show(void);
void Screen_Save_Power_Mode(void);
void Light_Up_Screen(void);


#endif
