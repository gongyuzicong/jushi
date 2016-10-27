#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

#include "data_type.h"
#include "common_include.h"
#include "magn_d_algo.h"
#include "led.h"


#define Max_Station_Num				11
#define RESPONSE_TIME_CALU(x)		(1000 - x * 100)

#define MAX_HALL_COUNT		1

#define USE_HALL_CTRL		0


#define ProtectSW_F				PDin(5)
#define ProtectSW_F_RESPOND		(0 == ProtectSW_F)
#define ProtectSW_F_UNRESPOND	(1 == ProtectSW_F)
#define ProtectSW_R				PEin(4)
#define ProtectSW_R_RESPOND		(0 == ProtectSW_R)
#define ProtectSW_R_UNRESPOND	(1 == ProtectSW_R)

//#define MOTOR1_HALL_COUNT_FLAG	ctrlParasPtr->rightHallCounterFlag
//#define MOTOR2_HALL_COUNT_FLAG	ctrlParasPtr->leftHallCounterFlag
#define MOTOR1_HALL_COUNT_FLAG		ctrlParasPtr->HallCounterFlag
#define MOTOR2_HALL_COUNT_FLAG		ctrlParasPtr->HallCounterFlag

#define MOTOR_RIGHT_CCR_DEF(X) 	Motor_Right_CR(X)
#define MOTOR_LEFT_CCR_DEF(X) 	Motor_Left_CCR(X)

#define MOTOR_RIGHT_CR_DEF(X) 	Motor_Right_CCR(X)
#define MOTOR_LEFT_CR_DEF(X) 	Motor_Left_CR(X)
//#define CHECK_MOTOR_SET_DUTY(duty)		(((duty >= 0) && (duty <= 100)) ? 1 : 0)
#define CHECK_MOTOR_SET_DUTY(duty)		((duty <= 100) ? 1 : 0)

#define EXTRA_CROSS_ROAD_R	2

#define USE_MPU6050 0

#define MAX_SPEED_LIMIT (100 - MAX_STEP_SPEED_INC)
#define MAX_STEP_SPEED_INC	1

#define X3X2X1_MAX_SPEED_LIMIT		0x07

#define MOTOR_SPEED_RESPON_TIME		(100)
#define MAX_GEAR_NUM		21
#define MAX_GEAR_OFFSET		11

#define MOTOR_RIGHT_CR_PIN_SET()			{MOTOR_RIGHT_BK = 1; MOTOR_RIGHT_FR = 0; MOTOR_RIGHT_EN = 0;}
#define MOTOR_RIGHT_CCR_PIN_SET()			{MOTOR_RIGHT_BK = 1; MOTOR_RIGHT_FR = 1; MOTOR_RIGHT_EN = 0;}
#define MOTOR_RIGHT_STOP_PIN_SET()			{MOTOR_RIGHT_BK = 0; MOTOR_RIGHT_FR = 1; MOTOR_RIGHT_EN = 1;}
#define MOTOR_LEFT_CR_PIN_SET()				{MOTOR_LEFT_BK = 1;  MOTOR_LEFT_FR = 0;  MOTOR_LEFT_EN = 0;}
#define MOTOR_LEFT_CCR_PIN_SET()			{MOTOR_LEFT_BK = 1;  MOTOR_LEFT_FR = 1;  MOTOR_LEFT_EN = 0;}
#define MOTOR_LEFT_STOP_PIN_SET()			{MOTOR_LEFT_BK = 0;  MOTOR_LEFT_FR = 1;  MOTOR_LEFT_EN = 1;}

#define CHANGE_TO_GO_STRAIGHT_MODE()		{RFID_Info_Ptr->lock = 0; MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = goStraightStatus;}
#define CHANGE_TO_BACK_MODE()				{RFID_Info_Ptr->lock = 0; MOTOR_RIGHT_CCR_PIN_SET(); MOTOR_LEFT_CCR_PIN_SET(); ctrlParasPtr->agvStatus = backStatus;}
#define CHANGE_TO_CIR_LEFT_MODE()			{RFID_Info_Ptr->lock = 0; MOTOR_RIGHT_CCR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = cirLeft;}
#define CHANGE_TO_CIR_RIGHT_MODE()			{RFID_Info_Ptr->lock = 0; MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CCR_PIN_SET(); ctrlParasPtr->agvStatus = cirRight;}
#define CHANGE_TO_STOP_MODE()				{RFID_Info_Ptr->lock = 1; MOTOR_RIGHT_STOP_PIN_SET(); MOTOR_LEFT_STOP_PIN_SET(); ctrlParasPtr->agvStatus = stopStatus;}
#define CHANGE_TO_TEST_MODE()				{RFID_Info_Ptr->lock = 0; MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET();}
#define CHANGE_TO_GO_STRAIGHT_SLOW_MODE()	{RFID_Info_Ptr->lock = 0; MOTOR_RIGHT_CR_PIN_SET(); MOTOR_LEFT_CR_PIN_SET(); ctrlParasPtr->agvStatus = gSslow;}
#define CHANGE_TO_BACK_SLOW_MODE()			{RFID_Info_Ptr->lock = 0; MOTOR_RIGHT_CCR_PIN_SET(); MOTOR_LEFT_CCR_PIN_SET(); ctrlParasPtr->agvStatus = bSslow;}

/***********MOTOR RIGHT: START***************/
/****MOTOR OUT: START****/
#define MOTOR_RIGHT_EN		PDout(0)
#define MOTOR_RIGHT_FR		PDout(1)
//#define MOTOR_RIGHT_BK		PDout(2)
#define MOTOR_RIGHT_BK		PCout(13)
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
#define MOTOR_POWER			PDout(15)
#define MOTOR_POWER_ON()	{MOTOR_POWER = 0;}
#define MOTOR_POWER_OFF()	{MOTOR_POWER = 1;}
/***********MOTOR LEFT: END***************/




#define TRIGGER_PIN_O	PEout(11)
#define TRIGGER_PIN_I	PEin(11)

/************** USE ECV Start **************/

#define CREATE_BIT_FLAG(BIT)		(0x01 << BIT)

#define SET_BIT_FLAG(DATA, FLAG)			{DATA |= FLAG;}
#define CLEAN_BIT_FLAG(DATA, FLAG)			{DATA &= ~(FLAG);}
#define CHECK_BIT_FLAG(DATA, FLAG)			(DATA & FLAG)
#define CHECK_BIT_VALUE(DATA, FLAG)			(((DATA & FLAG) > 0) ? 1 : 0)
#define CHECK_BIT_VALUE_TOGGLE(DATA, FLAG)	(((DATA & FLAG) > 0) ? 0 : 1)

#define MACHINE_ARM_INIT_BIT_FLAG			(CREATE_BIT_FLAG(0))
#define MACHINE_ARM_INIT_SET()				{SET_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_INIT_BIT_FLAG);}
#define MACHINE_ARM_INIT_CLEAN()			{CLEAN_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_INIT_BIT_FLAG);}
#define MACHINE_ARM_INIT_CHECK				(CHECK_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_INIT_BIT_FLAG))

#define MACHINE_ARM_TOGGLE_BIT_FLAG			(CREATE_BIT_FLAG(1))
#define MACHINE_ARM_TOGGLE_SET()			{SET_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_TOGGLE_BIT_FLAG);}
#define MACHINE_ARM_TOGGLE_CLEAN()			{CLEAN_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_TOGGLE_BIT_FLAG);}
#define MACHINE_ARM_TOGGLE_CHECK			(CHECK_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_TOGGLE_BIT_FLAG))

#define MACHINE_ARM_CATCH_BIT_FLAG			(CREATE_BIT_FLAG(2))
#define MACHINE_ARM_CATCH_SET()				{SET_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_CATCH_BIT_FLAG);}
#define MACHINE_ARM_CATCH_CLEAN()			{CLEAN_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_CATCH_BIT_FLAG);}
#define MACHINE_ARM_CATCH_CHECK				(CHECK_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_CATCH_BIT_FLAG))

#define MACHINE_ARM_WEIGHT_FIBER_BIT_FLAG	(CREATE_BIT_FLAG(3))
#define MACHINE_ARM_WEIGHT_FIBER_SET()		{SET_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_WEIGHT_FIBER_BIT_FLAG);}
#define MACHINE_ARM_WEIGHT_FIBER_CLEAN()	{CLEAN_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_WEIGHT_FIBER_BIT_FLAG);}
#define MACHINE_ARM_WEIGHT_CHECK			(CHECK_BIT_FLAG(ctrlParasPtr->ECV_StepFlag, MACHINE_ARM_WEIGHT_FIBER_BIT_FLAG))

void Machinearm_Control_Handle(void);

/************** USE ECV End **************/


/****************** WARNING STATUS START ***********************/

#define WARNING_STATUS_FLMT_SW_ERR_BIT_FLAG	(CREATE_BIT_FLAG(0))
#define WARNING_STATUS_FLMT_SW_ERR_SET()	{SET_BIT_FLAG(ctrlParasPtr->LED_Warning, WARNING_STATUS_FLMT_SW_ERR_BIT_FLAG);}
#define WARNING_STATUS_FLMT_SW_ERR_CLEAN()	{CLEAN_BIT_FLAG(ctrlParasPtr->LED_Warning, WARNING_STATUS_FLMT_SW_ERR_BIT_FLAG);}
#define WARNING_STATUS_FLMT_SW_CHECK		(CHECK_BIT_FLAG(ctrlParasPtr->LED_Warning, WARNING_STATUS_FLMT_SW_ERR_BIT_FLAG))

#define WARNING_STATUS_NORMAL_BIT_FLAG		(CREATE_BIT_FLAG(1))
#define WARNING_STATUS_NORMAL_SET()			{SET_BIT_FLAG(ctrlParasPtr->LED_Warning, WARNING_STATUS_NORMAL_BIT_FLAG);}
#define WARNING_STATUS_NORMAL_CLEAN()		{CLEAN_BIT_FLAG(ctrlParasPtr->LED_Warning, WARNING_STATUS_NORMAL_BIT_FLAG);}
#define WARNING_STATUS_NORMAL_CHECK			(CHECK_BIT_FLAG(ctrlParasPtr->LED_Warning, WARNING_STATUS_NORMAL_BIT_FLAG))

/******************** WARNING STATUS END *********************/


typedef enum
{
	StatusStart = 0,
	stopStatus,
	goStraightStatus,
	backStatus,
	cirLeft,
	cirRight,
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
	TestMode,
	RFID_Setting_Mode,
}AgvWalkMode;

typedef enum
{
	Man_Forward = 0,
	Man_Backward,
	Man_CirL,
	Man_CirR,
	Man_Stop,
}ManualMode_Ctrl;

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


#define MOTOR1_HallCtrlFlag()		{MotorLeftHallCtrlPtr->MotorHallCtrlFlag = 1;}
#define MOTOR2_HallCtrlFlag()		{MotorLeftHallCtrlPtr->MotorHallCtrlFlag = 1;}
#define MOTORLeft_HallCtrlFlag()	{MOTOR1_HallCtrlFlag();}
#define MotorRight_HallCtrlFlag()	{MOTOR2_HallCtrlFlag();}
#define Motor_HallCtrlFlag()		{MOTORLeft_HallCtrlFlag(); MotorRight_HallCtrlFlag();}



typedef struct
{
	u8 	settedSpeed;
	u8 	rightMotorSettedSpeed;
	u8 	leftMotorSettedSpeed;	
	u8 	rightMotorRealSpeed;	// 电机实际速度
	u8 	leftMotorRealSpeed;		// 电机实际速度
	AgvStatus agvStatus;
	AgvSpeedMode speedMode;
	AgvWalkMode agvWalkingMode;
	ManualMode_Ctrl manualCtrl;
	s8 	rightMotorSpeedOffset;
	s8 	leftMotorSpeedOffset;
	
	u32 rightHallIntervalTime;
	u32 leftHallIntervalTime;
	u32 comflag;

	u32 rightHallCounter;
	u32 leftHallCounter;

	u32 rightHallCounterCMP;
	u32 leftHallCounterCMP;

	u8	rightHallCounterFlag;
	u8	leftHallCounterFlag;
	
	u32 HLavg;
	u32 HRavg;
	u8 	gear;


	u8 	FSflag;
	u8 	BSflag;
	u8 	fgvflag;

	u32 goalRFIDnode;
	
	SpinStation goalStation;

	WalkStep walkingstep;

	u8 	crossRoadCountF;
	u8 	crossRoadUpdateF;

	u8 	crossRoadCountR;
	u8 	crossRoadUpdateR;

	u8 	LP_duty;
	u8 	RP_duty;
	u8 	LD_duty;
	u8 	RD_duty;

	u8 	start_origin_mode;
	u8 	originFlag;

	u8 	cirDuty;
	u8 	rifdAdaptFlag;

	u16 CrossRoadHallCountL;
	u16 CrossRoadHallCountR;
	u8 	CrossRoadHallCountFlag;

	u8 	HallCounterFlag;

	u8 	Use_WECV;

	u8 	StartupFlag;
	u8  CancelFlag;

	u16 ECV_StepFlag;
	u8 Catch_Goods_Flag;
	u8 Machine_ARM_Toggle_Flag;
	u8 LED_Warning;
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
	u8 	T1_update;
	u8 	T2_update;
	u8 	T3_update;
	u8 	All_update;
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
	u8 	duty;
	u8 	goodDuty;
	u8 	upLock;
	u8 	lowLock;
	u8 	goodLock;
	u8 	upLimDuty;
	u8 	lowLimDuty;
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
	u8 	buzzerFlag;
	u16 buzzerTime_ms;
	u8 	buzzerNum;
	void (*buzzerCtrlFunc)(void);
}Buzzer_Ctrl, *Buzzer_Ctrl_P;

typedef enum
{
	TypeUnknow,
	TypeManuReq,
	TypeAutoReq,
}ReqType;

typedef struct ReqQueueStruct
{
	u8 Req_Station;
	ReqType Req_Type;
}ReqQueueStr, *ReqQueueStr_P;


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
void RFID_Goal_Node_Analy(void);
void Walking_Step_Controler(void);
void AGV_Correct_gS_8ug(u8 gear);
void gS_startup_mode(u8);
void AGV_Correct_back_ug(u8);
void CrossRoad_Hall_Count_Start(void);
void CrossRoad_Hall_Count_Stop(void);
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
void SIMU_PWM_BreathWarningLED_Ctrl(void);
void ProtectSW_GPIO_Config(void);
void walking_cir(u8);
u8 Origin_PatCtrl(u8);
void AutoRunningFunc(void);
void ManualModeFunc(ManualMode_Ctrl);
void AGV_Correct_2(void);
void Recv_RFID_CrossRoad(u8);
u8 Origin_PatCtrl2(u8);
void set_duty_Com(u8, u8);
void ZigbeeRecv_Simu(void);
void ManualModeEcvCtrlFunc(void);



extern ControlerParaStruct_P ctrlParasPtr;
extern u32 responseTime;
extern u8 FLeftCompDuty[101];
extern u8 AgvGear[MAX_GEAR_NUM];
extern u16 ZBandRFIDmapping[11];
extern MPU6050_Para_P mpu6050DS_ptr;
extern Buzzer_Ctrl_P BuzzerCtrlPtr;


#endif




