#ifndef _LCD_H_
#define _LCD_H_

#include "common_include.h"

typedef enum
{
	LCD_MainPage,
	LCD_WeightPage,
	LCD_SystemPage,
	LCD_ManualPage,
	LCD_HelpPage,
	LCD_AuthorityPage,
	LCD_DataoutPage,
	LCD_RfidPage,
	LCD_TaskPage,
}LCD_View_Page;

typedef struct PageClass
{
	void (*show)(void);
	void (*battery_report)(void);
}Page_Class_Struct, *Page_Class_Struct_P;

typedef struct LCD_Info_Struct
{
	u8 LCD_Ready;
	LCD_View_Page ViewPage;
	Page_Class_Struct mainPage;
	Page_Class_Struct weightPage;
	Page_Class_Struct systemPage;
	Page_Class_Struct manualPage;
	Page_Class_Struct helpPage;
	Page_Class_Struct dataOutPage;
	Page_Class_Struct authorityPage;
	Page_Class_Struct rfidPage;
	Page_Class_Struct taskPage;
}LCD_Info_Str, *LCD_Info_Str_P;


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
void LCD_Page_Report(void);


extern LCD_Info_Str_P LCD_Info_Ptr;


#endif
