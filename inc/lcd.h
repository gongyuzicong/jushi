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

typedef struct PageCommonClass
{
	void (*show)(void);
	void (*battery_report)(void);
}Page_Common_Class, *Page_Common_Class_P;

typedef struct MainPageClass
{
	Page_Common_Class commonReport;
	void (*mainstateReport)(void);
}Main_Page_Class, *Main_Page_Class_P;

typedef struct WeightPageClass
{
	Page_Common_Class commonReport;
	
}Weight_Page_Class, *Weight_Page_Class_P;


typedef struct SystemPageClass
{
	Page_Common_Class commonReport;
	
}System_Page_Class, *System_Page_Class_P;

typedef struct ManualPageClass
{
	Page_Common_Class commonReport;
	
}Manual_Page_Class, *Manual_Page_Class_P;

typedef struct HelpPageClass
{
	Page_Common_Class commonReport;
	
}Help_Page_Class, *Help_Page_Class_P;

typedef struct DataOutPageClass
{
	Page_Common_Class commonReport;
	
}Data_Out_Page_Class, *Data_Out_Page_Class_P;

typedef struct AuthorityPageClass
{
	Page_Common_Class commonReport;
	
}Authority_Page_Class, *Authority_Page_Class_P;

typedef struct RfidPageClass
{
	Page_Common_Class commonReport;
	
}Rfid_Page_Class, *Rfid_Page_Class_P;

typedef struct TaskPageClass
{
	Page_Common_Class commonReport;
	void (*taskstateReport)(u16 *);
}Task_Page_Class, *Task_Page_Class_P;

typedef struct LCD_Info_Struct
{
	u8 LCD_Ready;
	LCD_View_Page ViewPage;
	Main_Page_Class mainPage;
	Weight_Page_Class weightPage;
	System_Page_Class systemPage;
	Manual_Page_Class manualPage;
	Help_Page_Class helpPage;
	Data_Out_Page_Class dataOutPage;
	Authority_Page_Class authorityPage;
	Rfid_Page_Class rfidPage;
	Task_Page_Class taskPage;
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
