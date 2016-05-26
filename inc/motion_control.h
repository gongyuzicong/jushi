#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

#include "data_type.h"
#include "common_include.h"

#define MOTOR_RIGHT_CCR_DEF(X) 	Motor_Right_CR(X)
#define MOTOR_LEFT_CCR_DEF(X) 	Motor_Left_CCR(X)

#define MOTOR_RIGHT_CR_DEF(X) 	Motor_Right_CCR(X)
#define MOTOR_LEFT_CR_DEF(X) 	Motor_Left_CR(X)


#define MOTOR_RIGHT_X1			PCout(4)
#define MOTOR_RIGHT_X2			PCout(5)
#define MOTOR_RIGHT_X3			PCout(6)

#define MOTOR_LEFT_X1			PBout(5)
#define MOTOR_LEFT_X2			PBout(6)
#define MOTOR_LEFT_X3			PBout(7)

#define MOTOR_RIGHT_X1_In		PCin(4)
#define MOTOR_RIGHT_X2_In		PCin(5)
#define MOTOR_RIGHT_X3_In		PCin(6)

#define MOTOR_LEFT_X1_In		PBin(5)
#define MOTOR_LEFT_X2_In		PBin(6)
#define MOTOR_LEFT_X3_In		PBin(7)


/***********MOTOR RIGHT: START***************/
/****MOTOR OUT: START****/
#define MOTOR_RIGHT_EN	PCout(11)
#define MOTOR_RIGHT_FR	PCout(12)
#define MOTOR_RIGHT_BK	PCout(13)

//#define MOTOR_RIGHT_X1	PBout(6)
//#define MOTOR_RIGHT_X2	PBout(7)
//#define MOTOR_RIGHT_X3	PBout(8)
#define MOTOR_RIGHT_EN_IN	PCin(11)
#define MOTOR_RIGHT_FR_IN	PCin(12)

#define MOTOR_RIGHT_SV	PAout(2)
/****MOTOR OUT: END****/

/****MOTOR IN: START****/
//#define MOTOR_RIGHT_PG	PBin(9)
//#define MOTOR_RIGHT_ALM	PBin(14)
/****MOTOR IN: END****/
/***********MOTOR RIGHT: END***************/



/***********MOTOR LEFT: START***************/
/****MOTOR OUT: START****/
#define MOTOR_LEFT_EN	PBout(10)
#define MOTOR_LEFT_FR	PBout(11)
#define MOTOR_LEFT_BK	PBout(12)
//#define MOTOR_LEFT_X1	PAout(11)
//#define MOTOR_LEFT_X2	PAout(12)
//#define MOTOR_LEFT_X3	PAout(13)
#define MOTOR_LEFT_EN_IN	PBin(10)
#define MOTOR_LEFT_FR_IN	PBin(11)

#define MOTOR_LEFT_SV	PAout(3)
/****MOTOR OUT: END****/

/****MOTOR IN: START****/
//#define MOTOR_LEFT_PG	PCin(8)
//#define MOTOR_LEFT_ALM	PCin(9)
/****MOTOR IN: END****/
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
}MotionOperaterStruct, *MotionOperaterStruct_P;



void Motion_Ctrl_Init(void);

extern ControlerParaStruct_P ctrlParasPtr;
extern MotionOperaterStruct_P motionOptsPtr;

#endif




