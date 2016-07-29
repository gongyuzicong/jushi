#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

#include "data_type.h"
#include "common_include.h"
#include "magn_d_algo.h"


#define Max_Station_Num				11
#define RESPONSE_TIME_CALU(x)		(1000 - x * 100)

#define MAX_HALL_COUNT		1

#define Warning_LED		PDout(3)
#define Warning_LED_IN	PDin(3)
#define ProtectSW_F		PEin(4)
#define ProtectSW_R		PDin(5)


#define MOTOR_RIGHT_CCR_DEF(X) 	Motor_Right_CR(X)
#define MOTOR_LEFT_CCR_DEF(X) 	Motor_Left_CCR(X)

#define MOTOR_RIGHT_CR_DEF(X) 	Motor_Right_CCR(X)
#define MOTOR_LEFT_CR_DEF(X) 	Motor_Left_CR(X)
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
#define MOTOR_LEFT_CR_PIN_SET()			{MOTOR_LEFT_BK = 1;  MOTOR_LEFT_FR = 0;  MOTOR_LEFT_EN = 0;}
#define MOTOR_LEFT_CCR_PIN_SET()		{MOTOR_LEFT_BK = 1;  MOTOR_LEFT_FR = 1;  MOTOR_LEFT_EN = 0;}
#define MOTOR_LEFT_STOP_PIN_SET()		{MOTOR_LEFT_BK = 0;  MOTOR_LEFT_FR = 1;  MOTOR_LEFT_EN = 1;}

#define CHANGE_TO_GO_STRAIGHT_MODE()		{MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = goStraightStatus;}
#define CHANGE_TO_BACK_MODE()				{MOTOR_RIGHT_CCR_PIN_SET(); MOTOR_LEFT_CCR_PIN_SET(); ctrlParasPtr->agvStatus = backStatus;}
#define CHANGE_TO_CIR_LEFT_MODE()			{MOTOR_RIGHT_CCR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = cirLeft;}
#define CHANGE_TO_CIR_RIGHT_MODE()			{MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CCR_PIN_SET(); ctrlParasPtr->agvStatus = cirRight;}
#define CHANGE_TO_STOP_MODE()				{MOTOR_RIGHT_STOP_PIN_SET(); MOTOR_LEFT_STOP_PIN_SET(); ctrlParasPtr->agvStatus = stopStatus;}
#define CHANGE_TO_TEST_MODE()				{MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = testStatus;}
#define CHANGE_TO_GO_STRAIGHT_SLOW_MODE()	{MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = gSslow;}
#define CHANGE_TO_BACK_SLOW_MODE()			{MOTOR_RIGHT_CCR_PIN_SET(); MOTOR_LEFT_CCR_PIN_SET(); ctrlParasPtr->agvStatus = bSslow;}

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
#define ECV1_POWER			PDout(13)
#define ECV2_POWER			PDout(14)
#define ECV3_POWER			PEout(6)	//
#define MOTOR_POWER			PDout(15)
#define MOTOR_POWER_ON()	{MOTOR_POWER = 0;}
#define MOTOR_POWER_OFF()	{MOTOR_POWER = 1;}
/***********MOTOR LEFT: END***************/

#define ECV1_PWM		PBout(8)		// 前电缸
#define ECV1_DIR		PCout(14)		// 电缸方向 0: 缩   1: 推

#define ECV2_PWM		PBout(9)		// 后电缸
#define ECV2_DIR		PCout(15)		// 电缸方向 0: 推   1: 缩

#define ECV3_PWM		PEout(8)		// 
#define ECV3_DIR		PEout(7)		// 

#define LMT_INR			PCin(3)			// 响应为 0
#define LMT_INL			PCin(4)			// 响应为 0

#define LMT_SW			PEin(0)			// 响应为 0
#define Return_SW		PDin(10)		// 响应为 0

#define BUZZER_1		PEout(9)
#define BUZZER_2		PEout(10)

#define ECV_POWER_ON()	{ECV1_POWER = 0; ECV2_POWER = 0; ECV3_POWER = 0;}
#define ECV_POWER_OFF()	{ECV1_POWER = 1; ECV2_POWER = 1; ECV3_POWER = 1;}

#define FECV_UP()		{ECV2_DIR = 0;   ECV2_PWM = 1;  }		//
#define FECV_DOWN()		{ECV2_DIR = 1;   ECV2_PWM = 1;  }
#define FECV_STOP()		{ECV2_PWM = 0;					}

#define BECV_UP()		{ECV1_DIR = 1;   ECV1_PWM = 1;  }
#define BECV_DOWN()		{ECV1_DIR = 0;   ECV1_PWM = 1;  }
#define BECV_STOP()		{ECV1_PWM = 0;					}

#define WECV_UP()		{ECV3_DIR = 1;   ECV3_PWM = 1;  }
#define WECV_DOWN()		{ECV3_DIR = 0;   ECV3_PWM = 1;  }
#define WECV_STOP()		{ECV3_PWM = 0;					}


#define TRIGGER_PIN_O	PEout(11)
#define TRIGGER_PIN_I	PEin(11)

/**************ECV**************/


typedef enum
{
	StatusStart = 0,
	stopStatus,
	goStraightStatus,
	backStatus,
	cirLeft,
	cirRight,
	testStatus,
	gSslow,
	bSslow,
	StatusEnd,
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

typedef enum
{
	ControlCenter = 0,
	SpinStation_1,
	SpinStation_2,
	SpinStation_3,
	SpinStation_4,
	SpinStation_5,
	SpinStation_6,
	SpinStation_7,
	SpinStation_8,
	SpinStation_9,
	SpinStation_10,
}SpinStation;


typedef enum
{
	step_gS = 1,
	step_gVeer,
	step_entry,
	step_catch,
	step_exit,
	step_weigh,
	step_bVeer,
	step_gB,
	step_wFTans,
	step_origin,
	step_stop,
}WalkStep;


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

	u32 rightHallCounter;
	u32 leftHallCounter;
	
	u32 HLavg;
	u32 HRavg;
	u8 avgFlag;
	u8 avgFlagCount;
	u8 gear;


	u8 FSflag;
	u8 BSflag;

	Damper dampingFlag;
	u32 dampingTimeRec;

	u32 goalRFIDnode;
	
	SpinStation goalStation;

	WalkStep walkingstep;

	u8 crossRoadCountF;
	u8 crossRoadUpdateF;

	u8 crossRoadCountR;
	u8 crossRoadUpdateR;

	u8 T1DF;

	u8 T1dutyRec;

	u8 LP_duty;
	u8 RP_duty;
	u8 LD_duty;
	u8 RD_duty;

	u8 start_origin_mode;
	u8 originFlag;
}ControlerParaStruct, *ControlerParaStruct_P;


typedef struct
{
	u32 trec[3];
	Agv_MS_Location amlrec[100];
	u8 amlH;
}TimRec, *TimRec_P;

typedef struct
{
	u32 T1;
	u32 T2;
	u32 T3;
	u8 T1_update;
	u8 T2_update;
	u8 T3_update;
	u8 All_update;
}Trec, *Trec_P;

typedef struct
{
	u32 HallCountLeft;
	u32 HallCountRight;
}HallCount, *HallCount_P;

typedef enum
{
	Good = 0,
	Big,
	Small,
}Result_Judge;

typedef struct
{
	Result_Judge result;
	u32 timRec;
	u8 duty;
	u8 goodDuty;
	u8 upLock;
	u8 lowLock;
	u8 goodLock;
	u8 upLimDuty;
	u8 lowLimDuty;
}T1_AutoAdapt_Info, *T1_AutoAdapt_Info_P;

typedef struct
{
	u8 duty;
	Agv_MS_Location ms1;
	Agv_MS_Location ms2;
	Agv_MS_Location ms3;
	Agv_MS_Location ms4;
	Agv_MS_Location ms5;
	u8 msabs;
}Duty;

typedef struct
{
	Result_Judge result;
	//u32 timRec;
	u8 dutyr;
	u8 lock;
	Duty dt[20];
	u8 dt_head;
}T1_AutoAdapt_Info2, *T1_AutoAdapt_Info2_P;



typedef struct
{
	Result_Judge result;
	u8 lock;
	u8 duty;
	u8 goodDuty;
}Damp_AutoAdapt_Info, *Damp_AutoAdapt_Info_P;

typedef struct
{
	s16 ax;
	s16 ay;
	s16 az;
	
	s16 gx;
	s16 gy;
	s16 gz;

	s16 tempture;

	s16 ayMax;
	s16 ayMin;
	s16 ayOffset;
	s16 ayMid;
	s16 ayAverage;

	u8 ayAverageUpdate;

	s32 sumPlus;
	s32 sumMinus;
	u16 plusCount;
	u16 minusCount;

	s32 sum;
	u16 sumCount;
	
}MPU6050_Para, *MPU6050_Para_P;



typedef struct
{
	u8 twinkleFlag;
	u16 intervalTime_ms;
	u8 twinkleNum;
	void (*twinkleCtrlFunc)(void);
}LED_Twinkle, *LED_Twinkle_P;

typedef struct
{
	u8 buzzerFlag;
	u16 buzzerTime_ms;
	u8 buzzerNum;
	void (*buzzerCtrlFunc)(void);
}Buzzer_Ctrl, *Buzzer_Ctrl_P;

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
void AVG_Calu_Program(void);
void CleanAllSpeed(void);
void AGV_Correct_1(void);
void walking_cirLeft(u8);
void RFID_Goal_Node_Analy(void);
void Walking_Step_Controler(void);
void walking_cirRight(u8);
void AGV_Correct_gS_8ug(u8 gear);
void gS_back_mode(u8);
void gS_startup_mode(u8);
void gS_slow(u8);
void back_slow(u8);
void AGV_Correct_back_ug(u8);
void Hall_Count(void);
void AGV_Proc(void);
void MPU6050_Data_init(void);
void MPU6050_Data(void);
void MPU6050_Data1(void);
void MPU6050_Data_init3(void);
void CrossRoad_Count(void);
void Get_Zigbee_Info_From_Buf(void);
void ProtectFunc(void);
void gS_slow2(u8);
void back_slow2(u8);
void step_origin_Func(void);
void startup_origin_Func(void);
void originP(void);
void SIMU_PWM_Breath_Ctrl(void);


extern ControlerParaStruct_P ctrlParasPtr;
extern u32 responseTime;
extern u8 FLeftCompDuty[101];
extern u8 FRightCompDuty[101];
extern u8 BLeftCompDuty[101];
extern u8 BRightCompDuty[101];
extern u8 AgvGear[MAX_GEAR_NUM];
extern u16 ZBandRFIDmapping[11];
extern MPU6050_Para_P mpu6050DS_ptr;
extern LED_Twinkle_P WarningLedCtrlPtr;
extern Buzzer_Ctrl_P BuzzerCtrlPtr;

#endif




