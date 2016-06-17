#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

#include "data_type.h"
#include "common_include.h"

#define RESPONSE_TIME_CALU(x)		(1000 - x * 100)

#define MAX_HALL_COUNT		1

#define MOTOR_RIGHT_CCR_DEF(X) 			Motor_Right_CR(X)
#define MOTOR_LEFT_CCR_DEF(X) 			Motor_Left_CCR(X)

#define MOTOR_RIGHT_CR_DEF(X) 			Motor_Right_CCR(X)
#define MOTOR_LEFT_CR_DEF(X) 			Motor_Left_CR(X)

#define CHECK_MOTOR_SET_DUTY(duty)		(((duty >= 0) && (duty <= 100)) ? 1 : 0)

#define MAX_SPEED_LIMIT (100 - MAX_STEP_SPEED_INC)
#define MAX_STEP_SPEED_INC	1

#define X3X2X1_MAX_SPEED_LIMIT		0x07

#define MOTOR_SPEED_RESPON_TIME		(100)
#define MAX_GEAR_NUM		21
#define MAX_GEAR_OFFSET		11

#define MOTOR_RIGHT_CR_PIN_SET()		{MOTOR_RIGHT_BK = 1; MOTOR_RIGHT_FR = 0; MOTOR_RIGHT_EN = 0;}
#define MOTOR_RIGHT_CCR_PIN_SET()		{MOTOR_RIGHT_BK = 1; MOTOR_RIGHT_FR = 1; MOTOR_RIGHT_EN = 0;}
#define MOTOR_RIGHT_STOP_PIN_SET()		{MOTOR_RIGHT_BK = 0; MOTOR_RIGHT_FR = 1; MOTOR_RIGHT_EN = 1;}
#define MOTOR_LEFT_CR_PIN_SET()			{MOTOR_LEFT_BK = 1; MOTOR_LEFT_FR = 0; MOTOR_LEFT_EN = 0;}
#define MOTOR_LEFT_CCR_PIN_SET()		{MOTOR_LEFT_BK = 1; MOTOR_LEFT_FR = 1; MOTOR_LEFT_EN = 0;}
#define MOTOR_LEFT_STOP_PIN_SET()		{MOTOR_LEFT_BK = 0; MOTOR_LEFT_FR = 1; MOTOR_LEFT_EN = 1;}

#define CHANGE_TO_GO_STRAIGHT_MODE()		{MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = goStraightStatus;}
#define CHANGE_TO_BACK_MODE()				{MOTOR_RIGHT_CCR_PIN_SET(); MOTOR_LEFT_CCR_PIN_SET(); ctrlParasPtr->agvStatus = backStatus;}
#define CHANGE_TO_CIR_LEFT_MODE()			{MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CCR_PIN_SET(); ctrlParasPtr->agvStatus = cirLeft;}
#define CHANGE_TO_CIR_RIGHT_MODE()			{MOTOR_RIGHT_CCR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = cirRight;}
#define CHANGE_TO_STOP_MODE()				{MOTOR_RIGHT_STOP_PIN_SET(); MOTOR_LEFT_STOP_PIN_SET(); ctrlParasPtr->agvStatus = stopStatus;}
#define CHANGE_TO_TEST_MODE()				{MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = testStatus;}

/***********MOTOR RIGHT: START***************/
/****MOTOR OUT: START****/
#define MOTOR_RIGHT_EN		PDout(0)
#define MOTOR_RIGHT_FR		PDout(1)
#define MOTOR_RIGHT_BK		PDout(2)
#define MOTOR_RIGHT_X1		PDout(3)
#define MOTOR_RIGHT_X2		PDout(4)
#define MOTOR_RIGHT_X3		PDout(5)

#define MOTOR_RIGHT_EN_IN	PDin(0)
#define MOTOR_RIGHT_FR_IN	PDin(1)
#define MOTOR_RIGHT_X1_In	PDin(3)
#define MOTOR_RIGHT_X2_In	PDin(4)
#define MOTOR_RIGHT_X3_In	PDin(5)


#define MOTOR_RIGHT_SV		PBout(0)
/****MOTOR OUT: END****/

/****MOTOR IN: START****/
#define MOTOR_RIGHT_PG	PEin(14)
#define MOTOR_RIGHT_ALM	PEin(15)
/****MOTOR IN: END****/
/***********MOTOR RIGHT: END***************/



/***********MOTOR LEFT: START***************/
/****MOTOR OUT: START****/
#define MOTOR_LEFT_EN		PDout(6)
#define MOTOR_LEFT_FR		PDout(7)
#define MOTOR_LEFT_BK		PDout(8)
#define MOTOR_LEFT_X1		PDout(9)
#define MOTOR_LEFT_X2		PDout(10)
#define MOTOR_LEFT_X3		PDout(11)

#define MOTOR_LEFT_EN_IN	PDin(6)
#define MOTOR_LEFT_FR_IN	PDin(7)
#define MOTOR_LEFT_X1_In	PDin(9)
#define MOTOR_LEFT_X2_In	PDin(10)
#define MOTOR_LEFT_X3_In	PDin(11)

#define MOTOR_LEFT_SV		PBout(1)
/****MOTOR OUT: END****/

/****MOTOR IN: START****/
#define MOTOR_LEFT_PG	PEin(12)
#define MOTOR_LEFT_ALM	PEin(13)
/****MOTOR IN: END****/

#define MOTOR_POWER		PDout(15)
/***********MOTOR LEFT: END***************/

typedef enum
{
	stopStatus = 0,
	goStraightStatus,
	backStatus,
	cirLeft,
	cirRight,
	testStatus,
}AgvStatus, *AgvStatus_P;

typedef enum
{
	PWM_MODE = 0,
	X1X2X3_I_MODE,
	X1X2X3_II_MODE,
	INTER_MODE,
}AgvSpeedMode;

typedef enum
{
	AutomaticMode = 0,
	ManualMode,
}AgvWalkMode;

typedef enum
{
	DampingNone = 0,
	DampingLeft,
	DampingRight,
}Damper;

typedef struct
{
	u8 settedSpeed;
	u8 rightMotorSettedSpeed;
	u8 leftMotorSettedSpeed;		
	u8 rightMotorRealSpeed;	// 电机实际速度
	u8 leftMotorRealSpeed;	// 电机实际速度
	u8 rightInc;
	u8 leftInc;
	AgvStatus agvStatus;
	AgvSpeedMode speedMode;
	AgvWalkMode agvWalkingMode;
	u8 speedModeValue_Right;
	u8 speedModeValue_Left;
	s8 rightMotorSpeedOffset;
	s8 leftMotorSpeedOffset;

	u8* rightMotorSettedSpeed_p;
	u8* leftMotorSettedSpeed_p;
	s8* rightMotorSpeedOffset_p;
	s8* leftMotorSpeedOffset_p;
	
	u32 rightHallIntervalTime;
	u32 leftHallIntervalTime;
	u32 comflag;

	u32 HLavg;
	u32 HRavg;
	u8 avgFlag;
	u8 avgFlagCount;
	u8 gear;

	u8 changeModeFlag;

	u8 FSflag;
	u8 BSflag;

	Damper dampingFlag;
	Damper dampingTimeRec;
}ControlerParaStruct, *ControlerParaStruct_P;

typedef struct
{
	void (*motor_up)(void);
	void (*motor_down)(void);
	void (*motor_left)(void);
	void (*motor_right)(void);
	void (*motor_stop)(void);
	void (*agv_walk_test2)(void);
	void (*agv_walk_stop)(void);
}MotionOperaterStruct, *MotionOperaterStruct_P;


void Motion_Ctrl_Init(void);
void AGV_Walking(void);
void AGV_Change_Mode(void);
void LeftOrRight_Counter(void);
void AGV_Walking_Test(void);
void AGV_Correct_back_1(void);
void AGV_Correct_back_2(void);
void AGV_Correct_gS(void);
void AGV_Correct_gS_1(u8);
void AGV_Correct_gS_2(u8);
void AGV_Correct_gS_3(u8);
void AGV_Correct(void);
void AGV_Correct_back_3(u8);
void AVG_Calu_Program(void);
void CleanAllSpeed(void);
void AGV_Correct_1(void);
void AGV_Correct_gS_4(u8);
void AGV_Correct_gS_5(u8);
void AGV_Correct_back_4(u8);
void AGV_Correct_gS_6(u8 gear);
void AGV_Correct_gS_test(u8 gear);
void AGV_Correct_2(u8 gear);
void AGV_Correct_gS_7(u8 gear);



extern ControlerParaStruct_P ctrlParasPtr;
extern MotionOperaterStruct_P motionOptsPtr;
extern u32 responseTime;
extern u8 FLeftCompDuty[101];
extern u8 FRightCompDuty[101];
extern u8 BLeftCompDuty[101];
extern u8 BRightCompDuty[101];
extern u8 AgvGear[MAX_GEAR_NUM];

#endif




