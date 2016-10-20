#ifndef _LCD_H_
#define _LCD_H_

#include "common_include.h"

typedef enum
{
	LCD_MainPage,
	LCD_WeightPage,
}LCD_View_Page;

typedef struct LCD_Info_Struct
{
	LCD_View_Page ViewPage;
}LCD_Info_Str, *LCD_Info_Str_P;


void Uart_Send_Char(u8 data);
void Uart_Send_Str(u8* str);


//��������//
void Set_date(u8 y,u8 m,u8 d);
//����ʱ��//
void Set_time(u8 h,u8 m,u8 s);
//���õ���//
void Set_batter(u8 batter);
//������ҳ����״̬//
void Set_main_state(u16 *buf);
//���³���//
void Set_scale_weight(u8 z,u8 x);
void UART1_REC(u8);

//�����LCD���ܵ�����//
void Lcd_Handle(void);
void LCD_INIT(void);
void Weight_Screen_Show(void);
void Screen_Save_Power_Mode(void);
void Light_Up_Screen(void);
void LCD_Page_Report(void);


extern LCD_Info_Str_P LCD_Info_Ptr;


#endif
