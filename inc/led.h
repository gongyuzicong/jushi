#ifndef __LED_H__
#define __LED_H__


#include "common_include.h"
#include "motion_control.h"
#include "cfg_gpio.h"

#define Board_LED_IN						PCin(5)
#define Board_LED							PCout(5)
#define Board_LED_ON()						{Board_LED = 0;}
#define Board_LED_OFF()						{Board_LED = 1;}


#define Warning_LED_RED_FLAG				(CREATE_BIT_FLAG(0))
#define Warning_LED_RED_SET()				{SET_BIT_FLAG(WarningLedCtrlPtr->LED_status, Warning_LED_RED_FLAG);}
#define Warning_LED_RED_CLEAN()				{CLEAN_BIT_FLAG(WarningLedCtrlPtr->LED_status, Warning_LED_RED_FLAG);}
#define Warning_LED_RED_CHECK				(CHECK_BIT_VALUE_TOGGLE(WarningLedCtrlPtr->LED_status, Warning_LED_RED_FLAG))

#define Warning_LED_GREEN_FLAG				(CREATE_BIT_FLAG(1))
#define Warning_LED_GREEN_SET()				{SET_BIT_FLAG(WarningLedCtrlPtr->LED_status, Warning_LED_GREEN_FLAG);}
#define Warning_LED_GREEN_CLEAN()			{CLEAN_BIT_FLAG(WarningLedCtrlPtr->LED_status, Warning_LED_GREEN_FLAG);}
#define Warning_LED_GREEN_CHECK				(CHECK_BIT_VALUE_TOGGLE(WarningLedCtrlPtr->LED_status, Warning_LED_GREEN_FLAG))

#define Warning_LED_ORANGE_FLAG				(CREATE_BIT_FLAG(2))
#define Warning_LED_ORANGE_SET()			{SET_BIT_FLAG(WarningLedCtrlPtr->LED_status, Warning_LED_ORANGE_FLAG);}
#define Warning_LED_ORANGE_CLEAN()			{CLEAN_BIT_FLAG(WarningLedCtrlPtr->LED_status, Warning_LED_ORANGE_FLAG);}
#define Warning_LED_ORANGE_CHECK			(CHECK_BIT_VALUE_TOGGLE(WarningLedCtrlPtr->LED_status, Warning_LED_ORANGE_FLAG))



#define Warning_LED_RED_IN					PDin(3)
#define Warning_LED_RED						PDout(3)
#define Warning_LED_RED_ON()				{Warning_LED_RED = 0;}
#define Warning_LED_RED_OFF()				{Warning_LED_RED = 1;}
#define Warning_LED_RED_ON_CHECK()			{Warning_LED_RED = Warning_LED_RED_CHECK;}

#define Warning_LED_GREEN_IN				PCin(6)
#define Warning_LED_GREEN					PCout(6)
#define Warning_LED_GREEN_ON()				{Warning_LED_GREEN = 0;}
#define Warning_LED_GREEN_OFF()				{Warning_LED_GREEN = 1;}
#define Warning_LED_GREEN_ON_CHECK()		{Warning_LED_GREEN = Warning_LED_GREEN_CHECK;}

#define Warning_LED_ORANGE_IN				PCin(7)
#define Warning_LED_ORANGE					PCout(7)
#define Warning_LED_ORANGE_ON()				{Warning_LED_ORANGE = 0;}
#define Warning_LED_ORANGE_OFF()			{Warning_LED_ORANGE = 1;}
#define Warning_LED_ORANGE_ON_CHECK()		{Warning_LED_ORANGE = Warning_LED_ORANGE_CHECK;}


#define Warning_LED_NORMAL_STATUS()			{WarningLedCtrlPtr->twinkleFlag = 0; WarningLedCtrlPtr->mode = LED_Twinkle_Count_Enable; Warning_LED_RED_CLEAN(); Warning_LED_GREEN_SET(); Warning_LED_ORANGE_CLEAN(); Warning_LED_GREEN_ON(); WarningLedCtrlPtr->intervalTime_ms = 100;}
#define Warning_LED_FLMT_SW_ERR_STATUS()	{WarningLedCtrlPtr->twinkleFlag = 1; WarningLedCtrlPtr->mode = LED_Twinkle_Count_Disable; Warning_LED_RED_SET(); Warning_LED_GREEN_CLEAN(); Warning_LED_ORANGE_CLEAN(); WarningLedCtrlPtr->intervalTime_ms = 100;}
#define Warning_LED_RESET_STATUS()			{WarningLedCtrlPtr->twinkleFlag = 1; WarningLedCtrlPtr->mode = LED_Twinkle_Count_Disable; Warning_LED_RED_SET(); Warning_LED_GREEN_CLEAN(); Warning_LED_ORANGE_CLEAN(); WarningLedCtrlPtr->intervalTime_ms = 100;}
#define Warning_LED_ECV_TIMEOUT_STATUS()	{WarningLedCtrlPtr->twinkleFlag = 1; WarningLedCtrlPtr->mode = LED_Twinkle_Count_Disable; Warning_LED_RED_SET(); Warning_LED_GREEN_CLEAN(); Warning_LED_ORANGE_CLEAN(); WarningLedCtrlPtr->intervalTime_ms = 500;}


typedef enum
{
	LED_Twinkle_Count_Enable,
	LED_Twinkle_Count_Disable,
}LED_Twinkle_Count_Mode;

typedef struct LED_Control_Struct
{
	// 设置变量
	u8 	twinkleFlag;
	u8 	twinkleNum;
	u8	LED_status;
	u16 intervalTime_ms;
	LED_Twinkle_Count_Mode mode;

	// 临时变量
	u8 twinkleCounter;
	u8 step;
	u32 timRec;
	
	void (*twinkleCtrlFunc)(struct LED_Control_Struct *);

	void (*LED_ON)(void);
	void (*LED_OFF)(void);
	void (*LED_ON_CHECK)(void);
}LED_Control_Str, *LED_Control_Str_P;


void SIMU_PWM_BreathBoardLED_Ctrl(void);
void LED_Init(void);
void LED_Status_Handle(void);


extern LED_Control_Str_P WarningLedCtrlPtr;


#endif






