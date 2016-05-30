#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

#include "data_type.h"
#include "common_include.h"

#define MOTOR_RIGHT_CCR_DEF(X) 	Motor_Right_CR(X)
#define MOTOR_LEFT_CCR_DEF(X) 	Motor_Left_CCR(X)

#define MOTOR_RIGHT_CR_DEF(X) 	Motor_Right_CCR(X)
#define MOTOR_LEFT_CR_DEF(X) 	Motor_Left_CR(X)


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

typedef struct
{
	u8 settedSpeed;
	u8 rightMotorSettedSpeed;
	u8 leftMotorSettedSpeed;		
	u8 rightMotorSpeed;	// 电机实际速度
	u8 leftMotorSpeed;	// 电机实际速度
	u8 rightInc;
	u8 leftInc;
	AgvStatus agvStatus;
	AgvSpeedMode speedMode;
	AgvWalkMode agvWalkingMode;
	u8 speedModeValue_Right;
	u8 speedModeValue_Left;
}ControlerParaStruct, *ControlerParaStruct_P;

typedef struct
{
	void (*motor_up)(void);
	void (*motor_down)(void);
	void (*motor_left)(void);
	void (*motor_right)(void);
	void (*motor_stop)(void);
	void (*agv_walk)(void);
	void (*agv_walk_test)(void);
	void (*agv_walk_stop)(void);
}MotionOperaterStruct, *MotionOperaterStruct_P;


void Motion_Ctrl_Init(void);

extern ControlerParaStruct_P ctrlParasPtr;
extern MotionOperaterStruct_P motionOptsPtr;

#endif




