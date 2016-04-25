#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

#include "data_type.h"
#include "common_include.h"

/***********MOTOR RIGHT: START***************/
/****MOTOR OUT: START****/
#define MOTOR_RIGHT_EN	PCout(11)
#define MOTOR_RIGHT_FR	PCout(12)
#define MOTOR_RIGHT_BK	PBout(5)
#define MOTOR_RIGHT_X1	PBout(6)
#define MOTOR_RIGHT_X2	PBout(7)
#define MOTOR_RIGHT_X3	PBout(8)

#define MOTOR_RIGHT_SV	PAout(6)
/****MOTOR OUT: END****/

/****MOTOR IN: START****/
#define MOTOR_RIGHT_PG	PBin(9)
#define MOTOR_RIGHT_ALM	PBin(14)
/****MOTOR IN: END****/
/***********MOTOR RIGHT: END***************/



/***********MOTOR LEFT: START***************/
/****MOTOR OUT: START****/
#define MOTOR_LEFT_EN	PAout(8)
#define MOTOR_LEFT_FR	PAout(9)
#define MOTOR_LEFT_BK	PAout(10)
#define MOTOR_LEFT_X1	PAout(11)
#define MOTOR_LEFT_X2	PAout(12)
#define MOTOR_LEFT_X3	PAout(13)

#define MOTOR_LEFT_SV	PAout(7)
/****MOTOR OUT: END****/

/****MOTOR IN: START****/
#define MOTOR_LEFT_PG	PCin(8)
#define MOTOR_LEFT_ALM	PCin(9)
/****MOTOR IN: END****/
/***********MOTOR LEFT: END***************/

typedef enum
{
	stopStatus = 0,
	goStraightStatus,
	backMode,
}AgvStatus, *AgvStatus_P;

typedef struct
{
	u8 settedSpeed;
	u8 rightMotorSettedSpeed;
	u8 leftMotorSettedSpeed;		
	u8 rightMotorSpeed;	// 电机实际速度
	u8 leftMotorSpeed;	// 电机实际速度
	AgvStatus agvStatus;
}ControlerParaStruct, *ControlerParaStruct_P;

typedef struct
{
	u8 a1;
}MotionOperaterStruct, *MotionOperaterStruct_P;



void Motion_Ctrl_Init(void);

extern ControlerParaStruct_P ctrlParasPtr;
extern MotionOperaterStruct_P motionOptsPtr;

#endif




