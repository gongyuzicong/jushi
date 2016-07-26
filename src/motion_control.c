#include "motion_control.h"
#include "cfg_gpio.h"
#include "timer_opts.h"
#include "pwm_opts.h"
#include "magn_sensor.h"
#include "zigbee.h"
#include "mpu6050.h"
#include "i2c_opts.h"
#include "zigbee.h"
#include "buffer.h"


#define ABSOLU(value)	(value >= 0 ? value : (-value))
#define MAX_ADAPT_NUM	20
#define MAX_DAMP_ADAPT_NUM	15

#define MAX_IN		21
#define MAX_OUT 	21
#define ShiftTrigTime	15000

TimRec rec[30];
u8 recH = 0;

HallCount HallCountArr[6];
HallCount CrossRoadHallCountArrGS[6];
HallCount CrossRoadHallCountArrGB[6];


u16 ZBandRFIDmapping[11];

u8 AgvGear[MAX_GEAR_NUM] = {0, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
//u8 AgvGearCompDutyLF[MAX_GEAR_NUM] = {0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
u8 AgvGearCompDutyLF[MAX_GEAR_NUM] = {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
u8 AgvGearCompDutyRF[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 AgvGearCompDutyLB[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 AgvGearCompDutyRB[MAX_GEAR_NUM] = {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
u8 AgvGearK[MAX_GEAR_NUM] = {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 16, 17, 18, 19, 20, 21, 22};
u8 AgvGear7CDLF[MAX_GEAR_OFFSET] = {0, 2, 7, 8, 10, 12, 14, 16, 18, 20, 20};
T1_AutoAdapt_Info adaptInfo[MAX_ADAPT_NUM];
T1_AutoAdapt_Info adaptInfoB[MAX_ADAPT_NUM];
Damp_AutoAdapt_Info dampAdapetInfo[MAX_DAMP_ADAPT_NUM];
Damp_AutoAdapt_Info dampAdapetInfoB[MAX_DAMP_ADAPT_NUM];
T1_AutoAdapt_Info2 adaptInfo2[MAX_ADAPT_NUM];
T1_AutoAdapt_Info2 adaptInfoB2[MAX_ADAPT_NUM];

MPU6050_Para mpu6050DS;
MPU6050_Para_P mpu6050DS_ptr = &mpu6050DS;


u8 FLG[6][MAX_GEAR_NUM] = 	  {{0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10}};

u8 FRG[6][MAX_GEAR_NUM] = 	  {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10}};

u8 BLG[6][MAX_GEAR_NUM] = 	  {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10}};

u8 BRG[6][MAX_GEAR_NUM] = 	  {{0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},\
							   {0, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10}};




u8 FLeftCompDuty[101] = {0,\
						 0, 0, 0, 0, 0, 0, 4, 4, 4, 4,\
						 4, 4, 4, 4, 4, 2, 2, 2, 2, 2,\
						 2, 2, 2, 2, 3, 2, 2, 2, 2, 3,\
						 2, 2, 2, 2, 3, 2, 2, 2, 2, 3,\
						 2, 2, 2, 2, 3, 2, 2, 2, 2, 3,\
						 2, 2, 2, 2, 3, 2, 2, 2, 2, 3,\
						 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,\
						 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,\
						 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,\
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

u8 FRightCompDuty[101] = {0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


u8 BLeftCompDuty[101] = {0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
 						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

u8 BRightCompDuty[101] = {0,\
						  0, 0, 0, 0, 0, 0, 4, 4, 4, 4,\
						  4, 4, 4, 4, 4, 2, 2, 2, 2, 2,\
						  2, 2, 2, 2, 2, 2, 2, 2, 2, 2,\
						  2, 2, 2, 2, 2, 2, 2, 2, 2, 2,\
						  2, 2, 2, 2, 2, 2, 2, 2, 2, 2,\
						  2, 2, 2, 2, 2, 2, 2, 2, 2, 2,\
						  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,\
						  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,\
						  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,\
						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
						  



u8 LTM[9] = {181, 104, 73, 55, 45, 37, 32, 29, 26};
u8 RTM[9] = {132, 86, 63, 49, 41, 34, 30, 27, 24};

u8 DutyTableLow[10] = {2, 4, 8, 10, 12, 16, 16, 16, 16, 16};
u8 DutyTable[10] = {2, 8, 15, 20, 20, 20, 20, 20, 20, 20};

//u8 DutyTable[10] = {2, 4, 6, 8, 8, 8, 8, 8, 8, 8};

u8 DutyTable_Duty10[10] = {0, 0, 2, 3, 4, 5, 6, 7, 8, 9};

LED_Twinkle WarningLedCtrl;
LED_Twinkle_P WarningLedCtrlPtr = &WarningLedCtrl;

Buzzer_Ctrl BuzzerCtrl;
Buzzer_Ctrl_P BuzzerCtrlPtr = &BuzzerCtrl;

ControlerParaStruct ctrlParas;
ControlerParaStruct_P ctrlParasPtr = &ctrlParas;
//MotionOperaterStruct motionOpts;
//MotionOperaterStruct_P motionOptsPtr = &motionOpts;

u32 responseTime = 0;

void (*agv_walking_func[StatusEnd]) (u8);

#define MOTOR_RIGHT_DUTY_SET(speed)			(pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, speed))
#define MOTOR_LEFT_DUTY_SET(speed)			(pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, speed))

#define MOTOR_RIGHT_DUTY_SET_Setted(speed)			{ctrlParasPtr->rightMotorSettedSpeed = speed; pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, ctrlParasPtr->rightMotorSettedSpeed);}
#define MOTOR_LEFT_DUTY_SET_Setted(speed)			{ctrlParasPtr->leftMotorSettedSpeed = speed; pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, ctrlParasPtr->leftMotorSettedSpeed);}


#define MOTOR_RIGHT_DUTY_OFFSET()		{pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, ctrlParasPtr->settedSpeed + ctrlParasPtr->rightMotorSpeedOffset);}
#define MOTOR_LEFT_DUTY_OFFSET()		{pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, ctrlParasPtr->settedSpeed + ctrlParasPtr->leftMotorSpeedOffset);}




/**********Motor Basic Control Unit: Begin****************/

void Motor_Right_Set(u8 speed)
{
	if((speed > 0) && (speed <= 100))
	{
		MOTOR_RIGHT_DUTY_SET(speed);
	}
}

void Motor_Left_Set(u8 speed)
{
	if((speed > 0) && (speed <= 100))
	{
		MOTOR_LEFT_DUTY_SET(speed);
	}
}

void Motor_Right_CR(u8 speed)		// 正转
{
	
	if((speed > 0) && (speed <= 100))
	{
		pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, speed);
	}

	MOTOR_RIGHT_CR_PIN_SET();
}

void Motor_Right_CCR(u8 speed)	// 反转
{
	

	#if 1
	if((speed > 0) && (speed <= 100))
	{
		pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, speed);
	}
	#else
	pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, speed);
	#endif
	
	MOTOR_RIGHT_CCR_PIN_SET();
}

void Motor_Left_CR(u8 speed)		// 正转
{
	
	
	if((speed > 0) && (speed <= 100))
	{
		pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, speed);
	}

	
	MOTOR_LEFT_CR_PIN_SET();
}

void Motor_Left_CCR(u8 speed)		// 反转
{
	
	
	if((speed > 0) && (speed <= 100))
	{
		pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, speed);
	}
	
	MOTOR_LEFT_CCR_PIN_SET();
}


void Mode_Pin_Ctrl(ControlerParaStruct_P para)
{
	switch(para->agvStatus)
	{
		case stopStatus:
			MOTOR_RIGHT_DUTY_SET(para->rightMotorSettedSpeed);
			MOTOR_LEFT_DUTY_SET(para->leftMotorSettedSpeed);
			Delay_ms(500);
			MOTOR_RIGHT_FR = 1;
			MOTOR_LEFT_FR = 1;
			MOTOR_RIGHT_EN = 1;
			MOTOR_LEFT_EN = 1;
			break;

		case goStraightStatus:
			MOTOR_RIGHT_DUTY_SET(para->rightMotorSettedSpeed);
			MOTOR_LEFT_DUTY_SET(para->leftMotorSettedSpeed);
			Delay_ms(2000);
			MOTOR_RIGHT_FR = 1;
			MOTOR_LEFT_FR = 0;
			MOTOR_RIGHT_EN = 0;
			MOTOR_LEFT_EN = 0;
			break;

		case backStatus:
			MOTOR_RIGHT_DUTY_SET(para->rightMotorSettedSpeed);
			MOTOR_LEFT_DUTY_SET(para->leftMotorSettedSpeed);
			Delay_ms(2000);
			MOTOR_RIGHT_FR = 0;
			MOTOR_LEFT_FR = 1;
			MOTOR_RIGHT_EN = 0;
			MOTOR_LEFT_EN = 0;
			break;

		case cirLeft:
			MOTOR_RIGHT_DUTY_SET(para->rightMotorSettedSpeed);
			MOTOR_LEFT_DUTY_SET(para->leftMotorSettedSpeed);
			Delay_ms(2000);
			MOTOR_RIGHT_FR = 1;
			MOTOR_LEFT_FR = 1;
			MOTOR_RIGHT_EN = 0;
			MOTOR_LEFT_EN = 0;
			break;

		case cirRight:
			MOTOR_RIGHT_DUTY_SET(para->rightMotorSettedSpeed);
			MOTOR_LEFT_DUTY_SET(para->leftMotorSettedSpeed);
			Delay_ms(2000);
			MOTOR_RIGHT_FR = 0;
			MOTOR_LEFT_FR = 0;
			MOTOR_RIGHT_EN = 0;
			MOTOR_LEFT_EN = 0;
			break;

		default:
			break;
	}
}

/**********Motor Basic Control Unit: End****************/



/**********Motor Basic Control Mode: Begin****************/


void set_duty(u8 lmSpeed, u8 rmSpeed)
{
	if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
	{
		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

		MOTOR_LEFT_DUTY_SET(lmSpeed);
		MOTOR_RIGHT_DUTY_SET(rmSpeed);
	}
	else
	{
		printf("dutyErr! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
	}
}

void set_duty2(u8 lmSpeed, u8 rmSpeed)
{
	if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
	{
		ctrlParasPtr->leftMotorSettedSpeed = rmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = lmSpeed;

		MOTOR_LEFT_DUTY_SET(rmSpeed);
		MOTOR_RIGHT_DUTY_SET(lmSpeed);
	}
	else
	{
		printf("dutyErr! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
	}
}

void set_duty_Com(u8 lmSpeed, u8 rmSpeed)
{
	if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
	{
		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
		if(goStraightStatus == ctrlParasPtr->agvStatus)
		{

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			//printf("goStraightStatus\r\n");
		}
		else if(backStatus == ctrlParasPtr->agvStatus)
		{

			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			//printf("backStatus\r\n");
		}
		
	}
	else
	{
		printf("dutyErr1! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
	}
	
	
}


void damping_func(u32 time, u8 gearRecod, u8 lmSpeed, u8 rmSpeed)
{
	
	if(DampingNone != ctrlParasPtr->dampingFlag)		// 如果启动阻尼
	{
		ctrlParasPtr->comflag = 66;
		
		if((SystemRunningTime - ctrlParasPtr->dampingTimeRec) > time)		// 时间到, 恢复
		{
			//printf("tim = %d, vxt = %d\r\n", (SystemRunningTime - ctrlParasPtr->dampingTimeRec), (FMSDS_Ptr->VelocityXt));
			ctrlParasPtr->comflag = 661;
			
			if(DampingLeft == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6611;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
				
				printf("in left\r\n");
			}
			else if(DampingRight == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6612;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				//printf("tim = %d, vxt = %d\r\n", (SystemRunningTime - ctrlParasPtr->dampingTimeRec), (FMSDS_Ptr->VelocityXt));
				//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
				
				printf("in right\r\n");
			}
			
		}
	}

	set_duty(lmSpeed, rmSpeed);
}


void goStraight_change(void)
{
	ctrlParasPtr->settedSpeed = 0;
	ctrlParasPtr->leftMotorSettedSpeed = 0;
	ctrlParasPtr->rightMotorSettedSpeed = 0;
	ctrlParasPtr->leftMotorSpeedOffset = 0;
	ctrlParasPtr->rightMotorSpeedOffset = 0;
	MOTOR_RIGHT_DUTY_SET(0);
	MOTOR_LEFT_DUTY_SET(0);
	CHANGE_TO_STOP_MODE();
	
	Delay_ms(1000);
	
	
	CHANGE_TO_GO_STRAIGHT_MODE();
	ctrlParasPtr->settedSpeed = 10;
	MOTOR_RIGHT_DUTY_SET(10);
	MOTOR_LEFT_DUTY_SET(10);

	
	
}

void backStatus_change(void)
{
	ctrlParasPtr->settedSpeed = 0;
	ctrlParasPtr->leftMotorSettedSpeed = 0;
	ctrlParasPtr->rightMotorSettedSpeed = 0;
	ctrlParasPtr->leftMotorSpeedOffset = 0;
	ctrlParasPtr->rightMotorSpeedOffset = 0;
	MOTOR_RIGHT_DUTY_SET(0);
	MOTOR_LEFT_DUTY_SET(0);
	CHANGE_TO_STOP_MODE();
	
	Delay_ms(1000);
	
	
	CHANGE_TO_BACK_MODE();
	ctrlParasPtr->settedSpeed = 10;
	MOTOR_RIGHT_DUTY_SET(10);
	MOTOR_LEFT_DUTY_SET(10);
	
	

	
}

void CleanAllSpeed(void)
{
	ctrlParasPtr->leftMotorSpeedOffset = 0;
	ctrlParasPtr->leftMotorSettedSpeed = 0;
	
	ctrlParasPtr->rightMotorSpeedOffset = 0;
	ctrlParasPtr->rightMotorSettedSpeed = 0;

	 MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
	 MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
}

void MOTOR_LEFT_DUTY_SET_F(u8 left)
{
	if(left >= 155)
	{
		left = 0;
	}
	else if((left > 100) && (left < 155))
	{
		left = 100;
	}

	ctrlParasPtr->leftMotorSettedSpeed = left;

	MOTOR_LEFT_DUTY_SET(left);
	
}

void MOTOR_RIGHT_DUTY_SET_F(u8 right)
{
	if(right >= 155)
	{
		right = 0;
	}
	else if((right > 100) || (right < 155))
	{
		right = 100;
	}

	ctrlParasPtr->rightMotorSettedSpeed = right;

	MOTOR_RIGHT_DUTY_SET(right);
}

void MOTOR_DUTY_SET(u8 left, u8 right)
{
	MOTOR_LEFT_DUTY_SET_F(left);
	MOTOR_RIGHT_DUTY_SET_F(right);
}

void walking_goStraight(u8 gear)
{
	static u32 counter = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 2, lreco = 0, rreco = 0;
	u8 offset = 0;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	if(0)
	{
		ctrlParasPtr->comflag = 11;
		
		//if(Agv_MS_Center == RMSDS_Ptr->AgvMSLocation)
		if((RMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (RMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
		{
			counter++;
			//printf("counter = %d\r\n", counter);
			if(counter >= 300)
			{
				counter = 0;
				// 加速操作
				if(gearRecod < 4)
				{
					ctrlParasPtr->comflag = 111;
					gearRecod++;
					
					lmSpeed = AgvGear[gearRecod] + offset;
					rmSpeed = AgvGear[gearRecod];
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);
				}
			}
		}
		else
		{
			counter = 0;
		}
		
	}
	else
	{
		counter = 0;
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 12;
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3))
			{
				ctrlParasPtr->comflag = 121;
				if(gearRecod > 1)
				{
					//gearRecod--;
				}
				lreco++;
				//printf("lreco = %d\r\n", lreco);
				lmSpeed = AgvGear[gearRecod] + offset;
				rmSpeed = AgvGear[gearRecod];
			}
			else
			{
				ctrlParasPtr->comflag = 122;
				lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + offset + lreco;
				rmSpeed = AgvGear[gearRecod];
			}
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 13;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->comflag = 131;
				if(gearRecod > 1)
				{
					//gearRecod--;
				}
				rreco++;
				//printf("rreco = %d\r\n", rreco);
				lmSpeed = AgvGear[gearRecod] + offset;
				rmSpeed = AgvGear[gearRecod];
			}
			else
			{
				//printf("gearRecod = %d\r\n", gearRecod);
				ctrlParasPtr->comflag = 132;
				lmSpeed = AgvGear[gearRecod] + offset;
				rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
			}
			

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
		else if(Agv_MS_Right_Outside == FMSDS_Ptr->AgvMSLocation)
		{

			ctrlParasPtr->comflag = 14;
			
			lmSpeed = 10 + offset;
			rmSpeed = 10;

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);

		}
		else if(Agv_MS_Left_Outside == FMSDS_Ptr->AgvMSLocation)
		{	
			ctrlParasPtr->comflag = 15;
			
			lmSpeed = 10 + offset;
			rmSpeed = 10;

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
		else
		{
			ctrlParasPtr->comflag = 16;

			lmSpeed = 10 + offset;
			rmSpeed = 10;

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
	}
	
}


void walking_backStatus(u8 gear)
{
	static u32 counter = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 1;
	u8 offset = 0;
	
	ctrlParasPtr->comflag = 6;
	
	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		ctrlParasPtr->comflag = 21;
		//printf("AgvMSLocation = %d\r\n", RMSDS_Ptr->AgvMSLocation);
		//if(Agv_MS_Center == RMSDS_Ptr->AgvMSLocation)
		if((RMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_2) && (RMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_2))
		{
			counter++;
			//printf("counter = %d\r\n", counter);
			if(counter >= 300)
			{
				counter = 0;
				// 加速操作
				if(gearRecod < 4)
				{
					ctrlParasPtr->comflag = 211;
					gearRecod++;
					
					lmSpeed = AgvGear[4] + offset;
					rmSpeed = AgvGear[4];
					
					ctrlParasPtr->leftMotorSettedSpeed = rmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = lmSpeed;

					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);
				}
			}
		}
		else
		{
			counter = 0;
		}
		
	}
	else
	{
		counter = 0;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 22;
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_5))
			{
				ctrlParasPtr->comflag = 221;
				if(gearRecod > 1)
				{
					gearRecod--;
				}
				
				lmSpeed = AgvGear[gearRecod] + offset;
				rmSpeed = AgvGear[gearRecod];
			}
			else
			{
				ctrlParasPtr->comflag = 222;
				lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + offset;
				rmSpeed = AgvGear[gearRecod];
			}
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 23;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->comflag = 231;
				if(gearRecod > 1)
				{
					gearRecod--;
				}
				
				lmSpeed = AgvGear[gearRecod] + offset;
				rmSpeed = AgvGear[gearRecod];
			}
			else
			{
				ctrlParasPtr->comflag = 232;
				lmSpeed = AgvGear[gearRecod] + offset;
				rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
			}
			

			ctrlParasPtr->leftMotorSettedSpeed = rmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = lmSpeed;
			
			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			
		}
		else if(Agv_MS_Right_Outside == FMSDS_Ptr->AgvMSLocation)
		{

			ctrlParasPtr->comflag = 24;
			
			lmSpeed = 10 + offset;
			rmSpeed = 10;

			ctrlParasPtr->leftMotorSettedSpeed = rmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = lmSpeed;
			
			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);

		}
		else if(Agv_MS_Left_Outside == FMSDS_Ptr->AgvMSLocation)
		{	
			ctrlParasPtr->comflag = 25;
			
			lmSpeed = 10 + offset;
			rmSpeed = 10;

			ctrlParasPtr->leftMotorSettedSpeed = rmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = lmSpeed;
			
			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			
		}
		else
		{
			ctrlParasPtr->comflag = 26;

			lmSpeed = 10 + offset;
			rmSpeed = 10;

			ctrlParasPtr->leftMotorSettedSpeed = rmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = lmSpeed;
			
			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			
		}
	}
	
}


void walking_cirLeft(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0;

	lmSpeed = AgvGear[gear] + FLeftCompDuty[AgvGear[gear]];
	rmSpeed = AgvGear[gear] + FRightCompDuty[AgvGear[gear]];
	
	ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
	ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
	
	MOTOR_LEFT_DUTY_SET(lmSpeed);
	MOTOR_RIGHT_DUTY_SET(rmSpeed);

	
}



void walking_cirRight(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0;

	lmSpeed = AgvGear[gear] + FLeftCompDuty[AgvGear[gear]];
	rmSpeed = AgvGear[gear] + FRightCompDuty[AgvGear[gear]];
	
	ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
	ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
	
	MOTOR_LEFT_DUTY_SET(lmSpeed);
	MOTOR_RIGHT_DUTY_SET(rmSpeed);
}

void walking_stopStatus(u8 gear)
{
	//motion_stop_pwm();
}

void NullFunc(u8 gear)
{
	
}

/**********Motor Basic Control Mode: End****************/



u8 getLTM_Tim(u8 speed)
{
	u8 ltm_tim = 0;
	
	if((speed >= 10) && (speed < 15))
	{
		if(10 == speed)
		{
			ltm_tim = LTM[0];
		}
		else
		{
			ltm_tim = (LTM[0] + LTM[1]) / 2;
		}
	}
	else if((speed >= 15) && (speed < 20))
	{
		if(15 == speed)
		{
			ltm_tim = LTM[1];
		}
		else
		{
			ltm_tim = (LTM[1] + LTM[2]) / 2;
		}
	}
	else if((speed >= 20) && (speed < 25))
	{
		if(20 == speed)
		{
			ltm_tim = LTM[2];
		}
		else
		{
			ltm_tim = (LTM[2] + LTM[3]) / 2;
		}
	}
	else if((speed >= 25) && (speed < 30))
	{
		if(25 == speed)
		{
			ltm_tim = LTM[3];
		}
		else
		{
			ltm_tim = (LTM[3] + LTM[4]) / 2;
		}
	}
	else if((speed >= 30) && (speed < 35))
	{
		if(30 == speed)
		{
			ltm_tim = LTM[4];
		}
		else
		{
			ltm_tim = (LTM[4] + LTM[5]) / 2;
		}
	}
	else if((speed >= 35) && (speed < 40))
	{
		if(35 == speed)
		{
			ltm_tim = LTM[5];
		}
		else
		{
			ltm_tim = (LTM[5] + LTM[6]) / 2;
		}
	}
	else if((speed >= 40) && (speed < 45))
	{
		if(40 == speed)
		{
			ltm_tim = LTM[6];
		}
		else
		{
			ltm_tim = (LTM[6] + LTM[7]) / 2;
		}
	}
	else if((speed >= 45) && (speed <= 50))
	{
		if(45 == speed)
		{
			ltm_tim = LTM[7];
		}
		else if(50 == speed)
		{
			ltm_tim = LTM[8];
		}
		else
		{
			ltm_tim = (LTM[7] + LTM[8]) / 2;
		}
		
	}
	return ltm_tim;
}

u8 getRTM_Tim(u8 speed)
{
	u8 rtm_tim = 0;
	
	if((speed >= 10) && (speed < 15))
	{
		if(10 == speed)
		{
			rtm_tim = RTM[0];
		}
		else
		{
			rtm_tim = (RTM[0] + RTM[1]) / 2;
		}
	}
	else if((speed >= 15) && (speed < 20))
	{
		if(15 == speed)
		{
			rtm_tim = RTM[1];
		}
		else
		{
			rtm_tim = (RTM[1] + RTM[2]) / 2;
		}
	}
	else if((speed >= 20) && (speed < 25))
	{
		if(20 == speed)
		{
			rtm_tim = RTM[2];
		}
		else
		{
			rtm_tim = (RTM[2] + RTM[3]) / 2;
		}
	}
	else if((speed >= 25) && (speed < 30))
	{
		if(25 == speed)
		{
			rtm_tim = RTM[3];
		}
		else
		{
			rtm_tim = (RTM[3] + RTM[4]) / 2;
		}
	}
	else if((speed >= 30) && (speed < 35))
	{
		if(30 == speed)
		{
			rtm_tim = RTM[4];
		}
		else
		{
			rtm_tim = (RTM[4] + RTM[5]) / 2;
		}
	}
	else if((speed >= 35) && (speed < 40))
	{
		if(35 == speed)
		{
			rtm_tim = RTM[5];
		}
		else
		{
			rtm_tim = (RTM[5] + RTM[6]) / 2;
		}
	}
	else if((speed >= 40) && (speed < 45))
	{
		if(40 == speed)
		{
			rtm_tim = RTM[6];
		}
		else
		{
			rtm_tim = (RTM[6] + RTM[7]) / 2;
		}
	}
	else if((speed >= 45) && (speed <= 50))
	{
		if(45 == speed)
		{
			rtm_tim = RTM[7];
		}
		else if(50 == speed)
		{
			rtm_tim = RTM[8];
		}
		else
		{
			rtm_tim = (RTM[7] + RTM[8]) / 2;
		}
		
	}
	return rtm_tim;
}



void gS_back_mode(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
	rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - gainDuty[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
		
		
		startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

	#if 0
		// 加入阻尼模块
		// 阻尼begin	
		if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6341;
			ctrlParasPtr->dampingFlag = DampingLeft;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;
			
			//LocValu(FMSDS_Ptr->MaxRecoder);
			
			//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - gainDuty[1];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			
			//printf("lmSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6342;
			ctrlParasPtr->dampingFlag = DampingRight;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;

			//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - gainDuty[1];
			

			//printf("rmSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
		}
		// 阻尼end
		//else if(AgvNone == FMSDS_Ptr->agvDirection)
	#endif
	
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 633;
			
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
			}
			
			if(centCount > 5000)
			{
				ctrlParasPtr->FSflag = 2;
				ctrlParasPtr->comflag = 6331;
				
				startCount = 0;
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			}
			
		}


		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	damping_func(250, gearRecod, rmSpeed, lmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}


void gS_startup_mode(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeedP = 0;
		
		startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
		
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 633;
			
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
			}
			
			if(centCount > 8000)
			{
				ctrlParasPtr->FSflag = 1;
				ctrlParasPtr->comflag = 6331;
				
				startCount = 0;
				
				lmSpeedP = 0;
				rmSpeedP = 0;
			}
			
		}
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	//damping_func(250, gearRecod, lmSpeed, rmSpeed);
	set_duty(lmSpeed, rmSpeed);
	
}

void gS_startup_mode2(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	//u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8};
	
	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeedP = 0;
		
		startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		if(0 == startCount)
		{
			startCount = SystemRunningTime;
		}
		else
		{
			centCount = SystemRunningTime - startCount;
		}
		
		if(centCount > ShiftTrigTime)
		{
			ctrlParasPtr->FSflag = 1;
			ctrlParasPtr->comflag = 6331;
			
			startCount = 0;
			
			lmSpeedP = 0;
			rmSpeedP = 0;
		}
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}

	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	//damping_func(250, gearRecod, lmSpeed, rmSpeed);
	set_duty(lmSpeed, rmSpeed);
	
}


void bS_startup_mode(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeedP = 0;
		
		startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
		
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 633;
			
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
			}
			
			if(centCount > 8000)
			{
				ctrlParasPtr->BSflag = 1;
				ctrlParasPtr->comflag = 6331;
				
				startCount = 0;
				
				lmSpeedP = 0;
				rmSpeedP = 0;
			}
			
		}
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	//damping_func(250, gearRecod, lmSpeed, rmSpeed);
	set_duty(rmSpeed, lmSpeed);
	
}

void bS_startup_mode2(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	//u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeedP = 0;
		
		startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		if(0 == startCount)
		{
			startCount = SystemRunningTime;
		}
		else
		{
			centCount = SystemRunningTime - startCount;
		}
		
		if(centCount > ShiftTrigTime)
		{
			ctrlParasPtr->BSflag = 1;
			ctrlParasPtr->comflag = 6331;
			
			startCount = 0;
			
			lmSpeedP = 0;
			rmSpeedP = 0;
		}
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}

	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	//damping_func(250, gearRecod, lmSpeed, rmSpeed);
	set_duty(rmSpeed, lmSpeed);
	
}


void back_startup_mode(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
	rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];
		
		
		startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

#if 0
		// 加入阻尼模块
		// 阻尼begin	
		if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6341;
			ctrlParasPtr->dampingFlag = DampingLeft;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;
			
			//LocValu(FMSDS_Ptr->MaxRecoder);
			
			//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - gainDuty[1];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			
			//printf("lmSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6342;
			ctrlParasPtr->dampingFlag = DampingRight;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;

			//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - gainDuty[1];
			

			//printf("rmSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
		}
		// 阻尼end
		//else if(AgvNone == FMSDS_Ptr->agvDirection)
#endif
	
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 633;
			
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
			}
			
			if(centCount > 7000)
			{
				ctrlParasPtr->BSflag = 1;
				ctrlParasPtr->comflag = 6331;
				printf("BSflag = %d\r\n", ctrlParasPtr->BSflag);
				startCount = 0;
				
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];
			}
			
		}
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	damping_func(250, gearRecod, rmSpeed, lmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}




void gS_slow(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
	rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

	#if 0
		// 加入阻尼模块
		// 阻尼begin	
		if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6341;
			ctrlParasPtr->dampingFlag = DampingLeft;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;
			
			//LocValu(FMSDS_Ptr->MaxRecoder);
			
			//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - gainDuty[1];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			
			//printf("lmSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6342;
			ctrlParasPtr->dampingFlag = DampingRight;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;

			//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - gainDuty[1];
			

			//printf("rmSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
		}
		// 阻尼end
		//else if(AgvNone == FMSDS_Ptr->agvDirection)
	#endif
	
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	damping_func(250, gearRecod, lmSpeed, rmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}

void gS_slow2(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeedP = 0;
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeedP = 0;
		rmSpeedP = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}

	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	//damping_func(250, gearRecod, lmSpeed, rmSpeed);
	set_duty(lmSpeed, rmSpeed);
	
}



void back_slow(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
	rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];
		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

#if 0
		// 加入阻尼模块
		// 阻尼begin	
		if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6341;
			ctrlParasPtr->dampingFlag = DampingLeft;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;
			
			//LocValu(FMSDS_Ptr->MaxRecoder);
			
			//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - gainDuty[1];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			
			//printf("lmSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6342;
			ctrlParasPtr->dampingFlag = DampingRight;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;

			//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - gainDuty[1];
			

			//printf("rmSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
		}
		// 阻尼end
		//else if(AgvNone == FMSDS_Ptr->agvDirection)
#endif
	
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	damping_func(250, gearRecod, rmSpeed, lmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}

void back_slow2(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};

	// 启动模式
	ctrlParasPtr->comflag = 63;

	gearRecod = gear;

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
	}

	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 631;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeedP = 0;
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		
		lmSpeedP = 0;
		rmSpeedP = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}

	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	//damping_func(250, gearRecod, lmSpeed, rmSpeed);
	set_duty(rmSpeed, lmSpeed);
	
}



void gS_urgency_mode(void)
{
	static u8 lmSpeed = 0, rmSpeed = 0;
	// 紧急修正模式
	
	ctrlParasPtr->comflag = 65;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		lmSpeed = AgvGear[2] + AgvGearCompDutyLF[2] + AgvGear7CDLF[2];
		rmSpeed = AgvGear[2] + AgvGearCompDutyRF[2];		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		
		lmSpeed = AgvGear[2] + AgvGearCompDutyLF[2];
		rmSpeed = AgvGear[2] + AgvGearCompDutyRF[2] + AgvGear7CDLF[2];			
	}
	else if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		ctrlParasPtr->FSflag = 0;
	}

	set_duty(lmSpeed, rmSpeed);
	
}

void Show_Analy(Magn_Sensor_Data_Sturct_P ptr)
{
	//printf("MSD_Hex = %04d,\t", ptr->MSD_Hex);
	//MSD_Show_Bin(ptr->MSD_Hex);
	//printf(",\t");
	//printf("MSL = (%d) MS_", ptr->AgvMSLocation);
	if((ptr->AgvMSLocation > Agv_MS_Left_End) && (ptr->AgvMSLocation < Agv_MS_Center))
	{
		printf("L_");
		if((ptr->AgvMSLocation > Agv_MS_Left_5) && (ptr->AgvMSLocation < Agv_MS_Center))
		{
			if(0 == (ptr->AgvMSLocation % 2))
			{
				printf("%d.5  ", (Agv_MS_Center - ptr->AgvMSLocation) / 2);
			}
			else
			{
				printf("%d  ", (Agv_MS_Center - ptr->AgvMSLocation) / 2);
			}
			
		}
		else
		{
			printf("%d  ", Agv_MS_Center - ptr->AgvMSLocation - 5);
		}
		
	}
	else if((ptr->AgvMSLocation > Agv_MS_Center) && (ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		printf("R_");
		//printf("%d,\t", ptr->AgvMSLocation - Agv_MS_Center);
		
		if((ptr->AgvMSLocation > Agv_MS_Center) && (ptr->AgvMSLocation < Agv_MS_Right_5))
		{
			if(0 == (ptr->AgvMSLocation % 2))
			{
				printf("%d.5  ", (ptr->AgvMSLocation - Agv_MS_Center) / 2);
			}
			else
			{
				printf("%d  ", (ptr->AgvMSLocation - Agv_MS_Center) / 2);
			}
			
		}
		else
		{
			printf("%d  ", ptr->AgvMSLocation - Agv_MS_Center - 5);
		}
		
	}
	else if((ptr->AgvMSLocation >= Agv_MS_LOut_1) && (ptr->AgvMSLocation <= Agv_MS_LOut_8))
	{
		printf("LO_");
		printf("%d  ", ptr->AgvMSLocation - Agv_MS_Overline);
	}
	else if((ptr->AgvMSLocation >= Agv_MS_ROut_1) && (ptr->AgvMSLocation <= Agv_MS_ROut_8))
	{
		printf("RO_");
		printf("%d  ", ptr->AgvMSLocation - Agv_MS_LOut_8);
	}
	else if(ptr->AgvMSLocation == Agv_MS_Undefine)
	{
		printf("Undef  ");
	}
	else if(ptr->AgvMSLocation == Agv_MS_Overline)
	{
		printf("Overline  ");
	}
	else if(ptr->AgvMSLocation == Agv_MS_Right_Outside)
	{
		printf("Right_Outside  ");
	}
	else if(ptr->AgvMSLocation == Agv_MS_Left_Outside)
	{
		printf("Left_Outside  ");
	}
	else if(ptr->AgvMSLocation == Agv_MS_Center)
	{
		printf("MS_Center  ");
	}
	else if(ptr->AgvMSLocation == AgvInits)
	{
		printf("***  ");
	}
	else if(ptr->AgvMSLocation == Agv_MS_CrossRoad)
	{
		printf("CR  ");
	}
	
	//printf("LRe = %d,\t", ptr->LeftRemain);
	//printf("Zbits = %d,\t", ptr->BitNum);
	//printf("RRe = %d,\t", ptr->RightRemain);
	
}


void showTrec(void)
{
	u8 cir1 = 0, cir2 = 0;
	Magn_Sensor_Data_Sturct temp;

	for(cir1 = 0; cir1 <= recH; cir1++)
	{
		printf("%d,\t%d,\t%d,\t", rec[cir1].trec[0], rec[cir1].trec[1], rec[cir1].trec[2]);

		for(cir2 = 0; cir2 < rec[cir1].amlH; cir2++)
		{
			temp.AgvMSLocation = rec[cir1].amlrec[cir2];
			
			Show_Analy(&temp);
			
		}
		printf("\r\n");
	}
	printf("\r\n");
}


void Get_T1(Trec *now)
{
	if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		if((Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation))
		{
			now->T1 = FMSDS_Ptr->VelocityXt;
			now->T1_update = 1;			
		}
	}
	
}

void T_monitor(Trec *now)
{
	static u8 flaga = 0, flagt1 = 0;
	static u32 T1 = 0, T2 = 0, T3 = 0, countFlag = 0;
	static Agv_MS_Location locRec = AgvInits, locRec2 = AgvInits;
	
	
	if(locRec != FMSDS_Ptr->AgvMSLocation)
	{
		locRec = FMSDS_Ptr->AgvMSLocation;
		//printf("countFlag = %d, locRec = %d\r\n", countFlag, locRec);
		
		if(0 == countFlag)
		{
			if((Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation))
			{
				T1 = FMSDS_Ptr->VelocityXt;
				now->T1 = T1;
				flagt1 = 1;
				flaga = 1;
				now->T1_update = 1;
				
				countFlag = 1;
				//rec[recH].trec[0] = T1;
				printf("T1 = %d\r\n", T1);
				
			}
		}
		else if(1 == countFlag)
		{
			if((Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_0_5 == FMSDS_Ptr->AgvMSLocation))
			{
				T2 += FMSDS_Ptr->VelocityXt;
				countFlag = 2;
				now->T2 = T2;
				//rec[recH].trec[1] = T2;
				printf("T2 = %d\r\n", T2);
			}
			else
			{
				if(3 != now->T2_update)
				{
					now->T2_update = 1;
				}
				
				T2 += FMSDS_Ptr->VelocityXt;
			}
			
		}
		else if(2 == countFlag)
		{
			if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
			{
				T3 = FMSDS_Ptr->VelocityXt;
				countFlag = 0;
				//rec[recH].trec[2] = T3;
				printf("T3 = %d\r\n", T3);
				//printf("recH = %d, amlH = %d\r\n", recH, rec[recH].amlH);
				//rec[recH].amlrec[rec[recH].amlH] = AgvInits;
				//rec[recH].amlH++;
				//rec[recH].amlrec[rec[recH].amlH] = Agv_MS_Center;
				//rec[recH].amlH++;
				//showTrec();
				//recH++;
				//rec[recH].amlH = 0;
				flaga = 0;

				now->T3_update = 1;
				now->T3 = T3;
				now->All_update = 1;

				T1 = 0;
				T2 = 0;
				T3 = 0;
				
				//rec[recH].amlH = 0;
			}
			
			
		}
		
	}

	
	if(1 == flaga)
	{
		if(locRec2 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec2 = FMSDS_Ptr->AgvMSLocation;
			
			rec[recH].amlrec[rec[recH].amlH] = FMSDS_Ptr->AgvMSLocation;
			rec[recH].amlH++;

			if(1 == flagt1)
			{
				flagt1 = 0;
				rec[recH].amlrec[rec[recH].amlH] = AgvInits;
				rec[recH].amlH++;
			}
			
		}
		
	}

	

}

void show_adapt_info(T1_AutoAdapt_Info *arr)
{
	u8 cir = 0;

	if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		printf("showAdaptG:\r\n");
	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		printf("showAdaptB:\r\n");
	}
	
	for(cir = 0; cir < MAX_ADAPT_NUM; cir++)
	{
		printf("%02d: goodLock = %d, goodDuty = %d, duty = %d, upLock = %d, upLim = %d,  lowLock = %d, lowLim = %d, res = %d\r\n", cir, arr[cir].goodLock, arr[cir].goodDuty, arr[cir].duty, arr[cir].upLock, arr[cir].upLimDuty, arr[cir].lowLock, arr[cir].lowLimDuty, arr[cir].result);
	}
	
	printf("\r\n");
}

void show_adapt_info2(T1_AutoAdapt_Info2 *arr)
{
	u8 cir = 0, cir2 = 0;

	if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		printf("showAdaptG:\r\n");
	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		printf("showAdaptB:\r\n");
	}
	
	for(cir = 0; cir < MAX_ADAPT_NUM; cir++)
	{
		printf("arr %02d: dutyr = %02d, lock = %d\r\n", cir, arr[cir].dutyr, arr[cir].lock);
		
		for(cir2 = 0; cir2 < arr[cir].dt_head; cir2++)
		{
			printf("\tdt %02d: duty = %02d, ms1 = %02d, ms2 = %02d, ms3 = %02d, ms4 = %02d, ms5 = %02d, msabs = %02d\r\n", cir2, arr[cir].dt[cir2].duty, arr[cir].dt[cir2].ms1, arr[cir].dt[cir2].ms2, arr[cir].dt[cir2].ms3, arr[cir].dt[cir2].ms4, arr[cir].dt[cir2].ms5, arr[cir].dt[cir2].msabs);
		}
		
		printf("\r\n");
	}
	
	printf("\r\n");
}


void T1_Adapter_Com(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info *arr)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, flag = 0, re = 0;

	static Agv_MS_Location maxRec = AgvInits;

	static u32 recT1Tim = 0;

	u8 div = 100;
	
		
	Get_T1(&Tnow);
	

	if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		maxRec = FMSDS_Ptr->AgvMSLocation;
		
	}


	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < MAX_ADAPT_NUM))
		{
			Tnow.T1_update = 2;
			

			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				if(1 == arr[re].goodLock)
				{
					T1RSpeedin = arr[re].goodDuty;
				}
				else
				{
					T1RSpeedin = arr[re].duty;
				}
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				if(1 == arr[re].goodLock)
				{
					T1LSpeedin = arr[re].goodDuty;
				}
				else
				{
					T1LSpeedin = arr[re].duty;
				}
				
				T1RSpeedin = 0;
			}
			flag = 1;
			
			recT1Tim = SystemRunningTime;
			
			printf("re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		if(SystemRunningTime - recT1Tim >= 3000)
		{
			printf("T1 close*******\r\n");
			Tnow.T1_update = 3;
			T1LSpeedin = 0;
			T1RSpeedin = 0;
		}
		
	}

	if(3 == Tnow.T1_update)
	{
		if((AgvRight2Cent == FMSDS_Ptr->agvDirection) || (AgvLeft2Cent == FMSDS_Ptr->agvDirection))
		{
			Tnow.T1_update = 4;
			
			if(((maxRec >= Agv_MS_Left_10) && (maxRec <= Agv_MS_Left_2_5)) || ((maxRec >= Agv_MS_Right_2_5) && (maxRec <= Agv_MS_Right_10)))
			{
				arr[re].result = Small;
				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Small\r\n", re);
				}

				if(arr[re].duty < 15)
				{
					arr[re].duty++;
				}	
				
			}
			else if(((maxRec > Agv_MS_Left_1) && (maxRec < Agv_MS_Center)) || ((maxRec > Agv_MS_Center) && (maxRec < Agv_MS_Right_1)))
			{
				arr[re].result = Big;

				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Big\r\n", re);
				}

				if(arr[re].duty > 0)
				{
					arr[re].duty--;
				}
				
			}
			else if(((maxRec >= Agv_MS_Left_2) && (maxRec <= Agv_MS_Left_1)) || ((maxRec >= Agv_MS_Right_1) && (maxRec <= Agv_MS_Right_2)))
			{
				arr[re].result = Good;
				
				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Good****************\r\n\r\n", re);
				}
				arr[re].goodDuty = arr[re].duty;
				arr[re].goodLock = 1;
				
			}
			printf("showAdapt:\r\n");
			show_adapt_info(arr);
			printf("\r\n");
		}
	}
	
	if(1 == Tnow.All_update)
	{

		Tnow.All_update = 0;
		Tnow.T1 = 0;
		Tnow.T1_update = 0;
		Tnow.T2 = 0;
		Tnow.T2_update = 0;
		Tnow.T3 = 0;
		Tnow.T3_update = 0;
	}

	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}


void T1_Adapter(u8 *T1LSpeed, u8 *T1RSpeed)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, flag = 0, re = 0;

	static Agv_MS_Location maxRec = AgvInits;

	static u32 recT1Tim = 0;

	u8 div = 100;
	
		
	Get_T1(&Tnow);
	

	if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		maxRec = FMSDS_Ptr->AgvMSLocation;
		
	}


	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < MAX_ADAPT_NUM))
		{
			
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				
				if(1 == adaptInfo[re].goodLock)
				{
					T1RSpeedin = adaptInfo[re].goodDuty;
				}
				else
				{
					T1RSpeedin = adaptInfo[re].duty;
				}
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1RSpeedin = 0;
				
				if(1 == adaptInfo[re].goodLock)
				{
					T1LSpeedin = adaptInfo[re].goodDuty;
				}
				else
				{
					T1LSpeedin = adaptInfo[re].duty;
				}
				
			}
			
			flag = 1;
			
			recT1Tim = SystemRunningTime;
			
			//printf("re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		else
		{
			
		}

		Tnow.T1_update = 2;
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		if((SystemRunningTime - recT1Tim >= 3000) || (AGV_Pat_Ptr->AngleDirection < 0))
		{
			printf("T1 close*******\r\n");
			Tnow.T1_update = 3;
			T1LSpeedin = 0;
			T1RSpeedin = 0;
		}
		
	}

	if(3 == Tnow.T1_update)
	{
		if((AgvRight2Cent == FMSDS_Ptr->agvDirection) || (AgvLeft2Cent == FMSDS_Ptr->agvDirection))
		{
			Tnow.T1_update = 4;
			
			if(((maxRec >= Agv_MS_Left_10) && (maxRec <= Agv_MS_Left_2_5)) || ((maxRec >= Agv_MS_Right_2_5) && (maxRec <= Agv_MS_Right_10)))
			{
				adaptInfo[re].result = Small;
				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Small\r\n", re);
					printf("T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
				}

				if(adaptInfo[re].duty < 15)
				{
					adaptInfo[re].duty++;
				}	
				
			}
			else if(((maxRec > Agv_MS_Left_1) && (maxRec < Agv_MS_Center)) || ((maxRec > Agv_MS_Center) && (maxRec < Agv_MS_Right_1)))
			{
				adaptInfo[re].result = Big;

				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Big\r\n", re);
					printf("T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
				}

				if(adaptInfo[re].duty > 0)
				{
					adaptInfo[re].duty--;
				}
				
			}
			else if(((maxRec >= Agv_MS_Left_2) && (maxRec <= Agv_MS_Left_1)) || ((maxRec >= Agv_MS_Right_1) && (maxRec <= Agv_MS_Right_2)))
			{
				adaptInfo[re].result = Good;
				
				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Good****************\r\n\r\n", re);
					printf("T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
				}
				adaptInfo[re].goodDuty = adaptInfo[re].duty;
				adaptInfo[re].goodLock = 1;
				
			}
			printf("showAdaptG:\r\n");
			show_adapt_info(adaptInfo);
			printf("\r\n");
		}
	}
	
	if(1 == Tnow.All_update)
	{

		Tnow.All_update = 0;
		Tnow.T1 = 0;
		Tnow.T1_update = 0;
		Tnow.T2 = 0;
		Tnow.T2_update = 0;
		Tnow.T3 = 0;
		Tnow.T3_update = 0;
	}

	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}


void T1_Adapter_back(u8 *T1LSpeed, u8 *T1RSpeed)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, flag = 0, re = 0;

	static Agv_MS_Location maxRec = AgvInits;

	static u32 recT1Tim = 0;

	u8 div = 100;
	
		
	Get_T1(&Tnow);
	

	if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		maxRec = FMSDS_Ptr->AgvMSLocation;
		
	}


	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < MAX_ADAPT_NUM))
		{
			Tnow.T1_update = 2;
			

			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				if(1 == adaptInfoB[re].goodLock)
				{
					T1RSpeedin = adaptInfoB[re].goodDuty;
				}
				else
				{
					T1RSpeedin = adaptInfoB[re].duty;
				}
				
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				if(1 == adaptInfoB[re].goodLock)
				{
					T1LSpeedin = adaptInfoB[re].goodDuty;
				}
				else
				{
					T1LSpeedin = adaptInfoB[re].duty;
				}
				
				
				T1RSpeedin = 0;
			}
			
			flag = 1;
			
			recT1Tim = SystemRunningTime;
			
			//printf("B re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		if((SystemRunningTime - recT1Tim >= 3000) || (AGV_Pat_Ptr->AngleDirection < 0))
		{
			printf("B T1 close*******\r\n");
			Tnow.T1_update = 3;
			T1LSpeedin = 0;
			T1RSpeedin = 0;
		}
		
	}

	if(3 == Tnow.T1_update)
	{
		
		if((AgvRight2Cent == FMSDS_Ptr->agvDirection) || (AgvLeft2Cent == FMSDS_Ptr->agvDirection))
		{
			Tnow.T1_update = 4;
			
			if(((maxRec >= Agv_MS_Left_10) && (maxRec <= Agv_MS_Left_2_5)) || ((maxRec >= Agv_MS_Right_2_5) && (maxRec <= Agv_MS_Right_10)))
			{
				adaptInfoB[re].result = Small;
				if(1 == flag)
				{
					flag = 2;
					printf("B re = %d: Small\r\n", re);
					printf("T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
				}

				if(adaptInfoB[re].duty < 15)
				{
					adaptInfoB[re].duty++;
				}
				
			}
			else if(((maxRec > Agv_MS_Left_1) && (maxRec < Agv_MS_Center)) || ((maxRec > Agv_MS_Center) && (maxRec < Agv_MS_Right_1)))
			{
				adaptInfoB[re].result = Big;

				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Big\r\n", re);
					printf("T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
				}

				if(adaptInfoB[re].duty > 0)
				{
					adaptInfoB[re].duty--;
				}
				
			}
			else if(((maxRec >= Agv_MS_Left_2) && (maxRec <= Agv_MS_Left_1)) || ((maxRec >= Agv_MS_Right_1) && (maxRec <= Agv_MS_Right_2)))
			{
				adaptInfoB[re].result = Good;
				
				if(1 == flag)
				{
					flag = 2;
					printf("B re = %d: Good****************\r\n\r\n", re);
					printf("T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
				}
				
				adaptInfoB[re].goodLock = 1;
				adaptInfoB[re].goodDuty = adaptInfoB[re].duty;
			}
			printf("BshowAdaptB:\r\n");
			show_adapt_info(adaptInfoB);
			printf("\r\n");
		}
	}
	
	if(1 == Tnow.All_update)
	{

		Tnow.All_update = 0;
		Tnow.T1 = 0;
		Tnow.T1_update = 0;
		Tnow.T2 = 0;
		Tnow.T2_update = 0;
		Tnow.T3 = 0;
		Tnow.T3_update = 0;
	}

	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}


void T1_Adapter2(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info *arr)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, flag = 0, re = 0, side = 0, flag2 = 0;

	static Agv_MS_Location maxRec = AgvInits;

	static u32 recT1Tim = 0;

	static AgvStatus recStatus = stopStatus;

	u8 div = 100;
	//printf("agvStatus = %d, recStatus = %d\r\n", ctrlParasPtr->agvStatus, recStatus);
	if(recStatus != ctrlParasPtr->agvStatus)
	{
		recStatus = ctrlParasPtr->agvStatus;
		
		Tnow.T1_update = 0;
		T1LSpeedin = 0;
		T1RSpeedin = 0;
		side = 0;
		flag = 0;
		maxRec = AgvInits;
	}
	
	if(0 == Tnow.T1_update)
	{
		Get_T1(&Tnow);
		//printf("Get_T1\r\n");
	}
	
	if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		maxRec = FMSDS_Ptr->AgvMSLocation;
		
	}


	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < MAX_ADAPT_NUM))
		{
			
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				
				if(1 == arr[re].upLock)
				{
					T1RSpeedin = arr[re].upLimDuty - 1;
				}
				else
				{
					T1RSpeedin = arr[re].duty;
				}

				side = 1;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1RSpeedin = 0;
				
				if(1 == arr[re].upLock)
				{
					T1LSpeedin = arr[re].upLimDuty - 1;
				}
				else
				{
					T1LSpeedin = arr[re].duty;
				}

				side = 2;
			}
			
			flag = 1;
			
			recT1Tim = SystemRunningTime;
			printf("T1 start************\r\n");
			Tnow.T1_update = 2;
			//printf("re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		else
		{
			Tnow.T1_update = 0;
			printf("SB, re = %d\r\n", re);
		}
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		if(SystemRunningTime - recT1Tim >= 3000)
		{
			printf("T1 close*******\r\n");
			Tnow.T1_update = 3;
			T1LSpeedin = 0;
			T1RSpeedin = 0;
		}
		
	}

	
	if(3 == Tnow.T1_update)
	{
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			if(1 == side)
			{
				// 依旧在同侧
				flag2 = 1;
				
			}
			else if(2 == side)
			{
				// 在不同侧
				flag2 = 2;
			}
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{
			if(1 == side)
			{
				// 在不同侧
				
				flag2 = 2;
			}
			else if(2 == side)
			{
				// 依旧在同侧
				
				flag2 = 1;
			}
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			flag2 = 3;
			
		}

		
		if(1 == flag2)
		{
			if(((maxRec >= Agv_MS_Left_10) && (maxRec <= Agv_MS_Left_2_5)) || ((maxRec >= Agv_MS_Right_2_5) && (maxRec <= Agv_MS_Right_10)))
			{
				arr[re].result = Small;
				
				if(arr[re].duty < 15)
				{
					arr[re].duty++;
				}	

				if(goStraightStatus == ctrlParasPtr->agvStatus)
				{
					ctrlParasPtr->FSflag = 0;
				}
				else if(backStatus == ctrlParasPtr->agvStatus)
				{
					ctrlParasPtr->BSflag = 0;
				}

				if(1 == flag)
				{
					flag = 2;
					printf("Small\r\n");
					printf("%02d: upLock = %d, goodDuty = %d, duty = %d, up = %d, low = %d\r\n", re, arr[re].upLock, arr[re].goodDuty, arr[re].duty, arr[re].upLimDuty, arr[re].lowLimDuty);
				}
				
				Tnow.T1_update = 0;
				
			}
			else if(((maxRec >= Agv_MS_Left_2) && (maxRec <= Agv_MS_Left_1)) || ((maxRec >= Agv_MS_Right_1) && (maxRec <= Agv_MS_Right_2)))
			{
				arr[re].result = Good;

				if(0 == arr[re].lowLock)
				{
					arr[re].lowLimDuty = arr[re].duty;

					arr[re].lowLock = 1;
				}

				if(arr[re].duty < 15)
				{
					arr[re].duty++;
				}		

				if(1 == flag)
				{
					flag = 2;
					printf("Normal****************\r\n\r\n");
					printf("%02d: upLock = %d, goodDuty = %d, duty = %d, up = %d, low = %d\r\n", re, arr[re].upLock, arr[re].goodDuty, arr[re].duty, arr[re].upLimDuty, arr[re].lowLimDuty);
				}
			}
			else if(((maxRec > Agv_MS_Left_1) && (maxRec < Agv_MS_Center)) || ((maxRec > Agv_MS_Center) && (maxRec < Agv_MS_Right_1)))
			{
				arr[re].result = Good;

				if(0 == arr[re].goodLock)
				{
					arr[re].goodDuty = arr[re].duty;
				}
				
				if(1 == flag)
				{
					flag = 2;

					printf("Good\r\n");
					printf("%02d: upLock = %d, goodDuty = %d, duty = %d, up = %d, low = %d\r\n", re, arr[re].upLock, arr[re].goodDuty, arr[re].duty, arr[re].upLimDuty, arr[re].lowLimDuty);
				}
				
			}
			
		}
		else if(2 == flag2)
		{
			// 在不同侧
			arr[re].result = Big;
			arr[re].upLock = 1;
			arr[re].upLimDuty = arr[re].duty;

			if(1 == flag)
			{
				flag = 2;

				printf("Big\r\n");
				printf("%02d: upLock = %d, goodDuty = %d, duty = %d, up = %d, low = %d\r\n", re, arr[re].upLock, arr[re].goodDuty, arr[re].duty, arr[re].upLimDuty, arr[re].lowLimDuty);
			}
		}
		else if(3 == flag2)
		{
			arr[re].result = Good;
			arr[re].goodDuty = arr[re].duty;
			arr[re].goodLock = 1;
			
			if(1 == flag)
			{
				flag = 2;

				printf("GoodLock\r\n");
				printf("%02d: upLock = %d, goodDuty = %d, duty = %d, up = %d, low = %d\r\n", re, arr[re].upLock, arr[re].goodDuty, arr[re].duty, arr[re].upLimDuty, arr[re].lowLimDuty);
			}
		}

		
		show_adapt_info(arr);

		Tnow.T1_update = 0;
		side = 0;
		flag = 0;
		flag2 = 0;
		
	}
	
	

	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}


void T1_Adapter3(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info2 *arr)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, flag = 0, re = 0, side = 0, step = 1;

	static u32 recT1Tim = 0;

	u8 div = 100;
	//printf("agvStatus = %d, recStatus = %d\r\n", ctrlParasPtr->agvStatus, recStatus);
	
	if(0 == Tnow.T1_update)
	{
		Get_T1(&Tnow);
		//printf("Get_T1\r\n");
	}
	

	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;
		
		if((re >= 0) && (re < MAX_ADAPT_NUM))
		{
			
			if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End))
			{
				T1LSpeedin = 0;
				T1RSpeedin = arr[re].dutyr;
				if(1 == arr[re].lock)
				{
					printf("re = 02%d lock, T1RSpeedin = %02d\r\n", re, T1RSpeedin);
					goto END2_T1;
				}
				else
				{
					printf("Tnow.T1_update = %d, T1RSpeedin = %02d\r\n", Tnow.T1_update, T1RSpeedin);
				}

				side = 1;
			}
			else if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End) && (FMSDS_Ptr->AgvMSLocation > Agv_MS_Center))
			{
				T1RSpeedin = 0;
				T1LSpeedin = arr[re].dutyr;
				if(1 == arr[re].lock)
				{
					
					printf("re = 02%d lock, T1LSpeedin = %02d\r\n", re, T1LSpeedin);
					goto END2_T1;
				}
				else
				{
					printf("Tnow.T1_update = %d, T1LSpeedin = %02d\r\n", Tnow.T1_update, T1LSpeedin);
				}

				side = 2;
			}
			
			flag = 1;
			
			
			printf("T1 start************\r\n");
			Tnow.T1_update = 2;
			//printf("re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		else
		{
			//printf("SB, re = %d\r\n", re);
			goto END_T1;
		}
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		
		if(1 == side)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_3)
			{
				// fail small
				if(arr[re].dutyr < 15)
				{
					arr[re].dutyr++;
				}
				//Tnow.T1_update = 0;
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->BSflag = 0;
				printf("too small\r\n");
				goto END_T1;
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_0_5)
			{
				if(1 == flag)
				{
					T1RSpeedin /= 2;
					printf("T1RSpeedin / 2 = %02d\r\n", T1RSpeedin);
					flag = 2;
				}
				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				T1RSpeedin = 0;
				printf("T1RSpeedin = 0\r\n");
				Tnow.T1_update = 3;
				recT1Tim = SystemRunningTime;
			}
		}
		else if(2 == side)
		{
			if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3)
			{
				// fail small
				if(arr[re].dutyr < 15)
				{
					arr[re].dutyr++;
				}
				//Tnow.T1_update = 0;
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->BSflag = 0;
				printf("too small\r\n");
				goto END_T1;
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_0_5)
			{
				if(1 == flag)
				{
					T1LSpeedin /= 2;
					printf("T1RSpeedin / 2 = %02d\r\n", T1LSpeedin);
					flag = 2;
				}
				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				T1LSpeedin = 0;
				printf("T1LSpeedin = 0\r\n");
				Tnow.T1_update = 3;
				step = 1;
				recT1Tim = SystemRunningTime;
			}
		}
		
	}

	
	if(3 == Tnow.T1_update)
	{
		u32 clau = 0;
		if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_3) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3))
		{
			ctrlParasPtr->FSflag = 0;
			ctrlParasPtr->BSflag = 0;
			//.arr[re].lock = 1;
			printf("Go to end\r\n");
			goto END_T1;
		}

		clau = SystemRunningTime - recT1Tim;
		if(1 == step)
		{
			
			if(clau >= 1000)
			{
				arr[re].dt[arr[re].dt_head].ms1 = FMSDS_Ptr->AgvMSLocation;
				printf("T1'3 step%d, ms1 = %02d, clau = %ld\r\n", step, arr[re].dt[arr[re].dt_head].ms1, clau);
				step = 2;
			}
			
		}
		else if(2 == step)
		{
			if(clau >= 2000)
			{
				arr[re].dt[arr[re].dt_head].ms2 = FMSDS_Ptr->AgvMSLocation;
				printf("T1'3 step%d, ms1 = %02d, clau = %ld\r\n", step, arr[re].dt[arr[re].dt_head].ms2, clau);
				step = 3;
			}
			
		}
		else if(3 == step)
		{
			if(clau >= 3000)
			{
				arr[re].dt[arr[re].dt_head].ms3 = FMSDS_Ptr->AgvMSLocation;
				printf("T1'3 step%d, ms1 = %02d, clau = %ld\r\n", step, arr[re].dt[arr[re].dt_head].ms3, clau);
				step = 4;
			}
			
		}
		else if(4 == step)
		{
			if(clau >= 4000)
			{
				arr[re].dt[arr[re].dt_head].ms4 = FMSDS_Ptr->AgvMSLocation;
				printf("T1'3 step%d, ms1 = %02d, clau = %ld\r\n", step, arr[re].dt[arr[re].dt_head].ms4, clau);
				step = 5;
			}
			
		}
		else if(5 == step)
		{
			if(clau >= 5000)
			{
				arr[re].dt[arr[re].dt_head].ms5 = FMSDS_Ptr->AgvMSLocation;
				//arr[re].dt[arr[re].dt_head].msabs = arr[re].dt[arr[re].dt_head].ms1 + arr[re].dt[arr[re].dt_head].ms2 + arr[re].dt[arr[re].dt_head].ms3 + arr[re].dt[arr[re].dt_head].ms4 + arr[re].dt[arr[re].dt_head].ms5;
				arr[re].dt[arr[re].dt_head].duty = arr[re].dutyr;

				if(arr[re].dt_head < MAX_ADAPT_NUM)
				{
					arr[re].dt_head++;
					arr[re].dutyr++;
				}
				else
				{
					arr[re].lock = 1;
				}

				show_adapt_info2(arr);
				printf("T1'3 step%d, ms1 = %02d, clau = %ld\r\n", step, arr[re].dt[arr[re].dt_head].ms5, clau);
								
				step = 1;
				goto END_T1;
			}
			
		}
		
		
	}
	
		*T1LSpeed = T1LSpeedin;
		*T1RSpeed = T1RSpeedin;
		return;
	
	END_T1:
		Tnow.T1_update = 0;
		side = 0;
		flag = 0;
		step = 1;
		*T1LSpeed = 0;
		*T1RSpeed = 0;
		return;

	END2_T1:
		Tnow.T1_update = 0;
		side = 0;
		flag = 0;
		step = 1;
		*T1LSpeed = T1LSpeedin;
		*T1RSpeed = T1RSpeedin;
		return;
}

void T1_Adapter3_back(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info2 *arr)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, flag = 0, re = 0, side = 0, step = 1;

	static u32 recT1Tim = 0;

	u8 div = 100;
	//printf("agvStatus = %d, recStatus = %d\r\n", ctrlParasPtr->agvStatus, recStatus);
	
	if(0 == Tnow.T1_update)
	{
		Get_T1(&Tnow);
		//printf("Get_T1\r\n");
	}
	

	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;
		
		if((re >= 0) && (re < MAX_ADAPT_NUM))
		{
			
			if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End))
			{
				T1LSpeedin = 0;
				T1RSpeedin = arr[re].dutyr;
				if(1 == arr[re].lock)
				{
					printf("re = 02%d lock, T1RSpeedin = %02d\r\n", re, T1RSpeedin);
					goto END2_T1;
				}
				else
				{
					printf("Tnow.T1_update = %d, T1RSpeedin = %02d\r\n", Tnow.T1_update, T1RSpeedin);
				}

				side = 1;
			}
			else if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End) && (FMSDS_Ptr->AgvMSLocation > Agv_MS_Center))
			{
				T1RSpeedin = 0;
				T1LSpeedin = arr[re].dutyr;
				if(1 == arr[re].lock)
				{
					
					printf("re = 02%d lock, T1LSpeedin = %02d\r\n", re, T1LSpeedin);
					goto END2_T1;
				}
				else
				{
					printf("Tnow.T1_update = %d, T1LSpeedin = %02d\r\n", Tnow.T1_update, T1LSpeedin);
				}

				side = 2;
			}
			
			flag = 1;
			
			
			printf("T1 start************\r\n");
			Tnow.T1_update = 2;
			//printf("re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		else
		{
			//printf("SB, re = %d\r\n", re);
			goto END_T1;
		}
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		
		if(1 == side)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_3)
			{
				// fail small
				if(arr[re].dutyr < 15)
				{
					arr[re].dutyr++;
				}
				//Tnow.T1_update = 0;
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->BSflag = 0;
				printf("too small\r\n");
				goto END_T1;
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_0_5)
			{
				if(1 == flag)
				{
					T1RSpeedin /= 2;
					printf("T1RSpeedin / 2 = %02d\r\n", T1RSpeedin);
					flag = 2;
				}
				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				T1RSpeedin = 0;
				printf("T1RSpeedin = 0\r\n");
				Tnow.T1_update = 3;
				recT1Tim = SystemRunningTime;
			}
		}
		else if(2 == side)
		{
			if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3)
			{
				// fail small
				if(arr[re].dutyr < 15)
				{
					arr[re].dutyr++;
				}
				//Tnow.T1_update = 0;
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->BSflag = 0;
				printf("too small\r\n");
				goto END_T1;
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_0_5)
			{
				if(1 == flag)
				{
					T1LSpeedin /= 2;
					printf("T1RSpeedin / 2 = %02d\r\n", T1LSpeedin);
					flag = 2;
				}
				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				T1LSpeedin = 0;
				printf("T1LSpeedin = 0\r\n");
				Tnow.T1_update = 3;
				step = 1;
				recT1Tim = SystemRunningTime;
			}
		}
		
	}

	
	if(3 == Tnow.T1_update)
	{
		u32 clau = 0;
		if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_3) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3))
		{
			ctrlParasPtr->FSflag = 0;
			ctrlParasPtr->BSflag = 0;
			//arr[re].lock = 1;
			printf("Go to end\r\n");
			goto END_T1;
		}

		clau = SystemRunningTime - recT1Tim;
		if(1 == step)
		{
			
			if(clau >= 1000)
			{
				arr[re].dt[arr[re].dt_head].ms1 = FMSDS_Ptr->AgvMSLocation;
				printf("T1'3 step%d, ms1 = %02d, clau = %d\r\n", step, arr[re].dt[arr[re].dt_head].ms1, clau);
				step = 2;
			}
			
		}
		else if(2 == step)
		{
			if(clau >= 2000)
			{
				arr[re].dt[arr[re].dt_head].ms2 = FMSDS_Ptr->AgvMSLocation;
				printf("T1'3 step%d, ms1 = %02d, clau = %d\r\n", step, arr[re].dt[arr[re].dt_head].ms2, clau);
				step = 3;
			}
			
		}
		else if(3 == step)
		{
			if(clau >= 3000)
			{
				arr[re].dt[arr[re].dt_head].ms3 = FMSDS_Ptr->AgvMSLocation;
				printf("T1'3 step%d, ms1 = %02d, clau = %d\r\n", step, arr[re].dt[arr[re].dt_head].ms3, clau);
				step = 4;
			}
			
		}
		else if(4 == step)
		{
			if(clau >= 4000)
			{
				arr[re].dt[arr[re].dt_head].ms4 = FMSDS_Ptr->AgvMSLocation;
				printf("T1'3 step%d, ms1 = %02d, clau = %d\r\n", step, arr[re].dt[arr[re].dt_head].ms4, clau);
				step = 5;
			}
			
		}
		else if(5 == step)
		{
			if(clau >= 5000)
			{
				arr[re].dt[arr[re].dt_head].ms5 = FMSDS_Ptr->AgvMSLocation;
				//arr[re].dt[arr[re].dt_head].msabs = arr[re].dt[arr[re].dt_head].ms1 + arr[re].dt[arr[re].dt_head].ms2 + arr[re].dt[arr[re].dt_head].ms3 + arr[re].dt[arr[re].dt_head].ms4 + arr[re].dt[arr[re].dt_head].ms5;
				arr[re].dt[arr[re].dt_head].duty = arr[re].dutyr;

				if(arr[re].dt_head < MAX_ADAPT_NUM)
				{
					arr[re].dt_head++;
					arr[re].dutyr++;
				}
				else
				{
					arr[re].lock = 1;
				}

				show_adapt_info2(arr);
				printf("T1'3 step%d, ms1 = %02d, clau = %d\r\n", step, arr[re].dt[arr[re].dt_head].ms5, clau);
								
				step = 1;
				goto END_T1;
			}
			
		}
		
		
	}
	
		*T1LSpeed = T1LSpeedin;
		*T1RSpeed = T1RSpeedin;
		return;
	
	END_T1:
		Tnow.T1_update = 0;
		side = 0;
		flag = 0;
		step = 1;
		*T1LSpeed = 0;
		*T1RSpeed = 0;
		return;

	END2_T1:
		Tnow.T1_update = 0;
		side = 0;
		flag = 0;
		step = 1;
		*T1LSpeed = T1LSpeedin;
		*T1RSpeed = T1RSpeedin;
		return;
}




void show_damp_info(Damp_AutoAdapt_Info *arr)
{
	u8 cir = 0;

	for(cir = 0; cir < MAX_DAMP_ADAPT_NUM; cir++)
	{
		printf("%d: lock = %d, goodDuty = %d, duty = %d, result = %d\r\n", cir, arr[cir].lock, arr[cir].goodDuty, arr[cir].duty, arr[cir].result);
	}
	
}

void Damp_Adapter_Com(u8 *lmSpeedPull, u8 *rmSpeedPull, Damp_AutoAdapt_Info *arr)
{
	static u8  flag = 0, flag2 = 0, cir = 0, flag3 = 0, flag4 = 0, sideRec = 0, sideRecPre = 0, index = 0;
	static u32 startCount = 0, countTime = 0, recoverTim = 0;
	static Agv_MS_Location locRec1 = AgvInits, locRec4 = AgvInits, maxRec = AgvInits;
	static u32 time = 3000;
	u8 lmSpeedPullin = 0, rmSpeedPullin = 0;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		sideRec = 1;
		
		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			maxRec = FMSDS_Ptr->AgvMSLocation;
			
			if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_0_5)	// 这里找时间为time的点
			{
				if(0 == flag2)
				{
					if(countTime >= time)		
					{
						locRec1 = FMSDS_Ptr->AgvMSLocation;
						printf("Point: ");
						Show_Analy(FMSDS_Ptr);
						printf(",\tcountTime = %d\r\n", countTime);
						countTime = 0;
						flag2 = 1;
						cir = 0;
					}
					else
					{
						if(locRec4 != FMSDS_Ptr->AgvMSLocation)
						{
							locRec4 = FMSDS_Ptr->AgvMSLocation;
							countTime += FMSDS_Ptr->VelocityXt;
							cir++;
							printf("cir = %d, VelocityXt = %d, countTime = %d\r\n", cir, FMSDS_Ptr->VelocityXt, countTime);
						}
						
					}
				}
				
				
			}
			
		}
		else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)	// 这里回到记录的点, 开始计时并且拉
		{
			if((FMSDS_Ptr->AgvMSLocation >= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}

		
		if(1 == flag)		// 开始拉并且计时
		{
			if(SystemRunningTime - startCount <= time)		// 如果还在时间内, 拉
			{
				if(maxRec <= Agv_MS_Left_2)
				{
					index = Agv_MS_Left_1 - maxRec;
					
					if(1 == arr[index].lock)
					{
						lmSpeedPullin = arr[index].goodDuty;
						
					}
					else
					{
						lmSpeedPullin = arr[index].duty;
						
					}
					
					rmSpeedPullin = 0;
					
					if(0 == flag3)
					{
						flag3 = 1;
						printf("MaxRecoder = %d\r\n", maxRec);
						//printf("l-%d\r\n", lmSpeedPullin);
						printf("l-index = %d, lmSpeedPullin = %d\r\n", index, lmSpeedPullin);
					}
				}
				
			}
			else		// 如果超出时间了, 放
			{
				
				flag3 = 0;
				flag = 2;
				
				lmSpeedPullin = 0;
				rmSpeedPullin = 0;
				flag4 = 1;
				recoverTim = SystemRunningTime;
				
			}
			

			if(2 == flag)		// 打印拉关闭log
			{
				printf("damp close***\r\n");
				flag = 3;
			}


			printf("**ti = %d\r\n", SystemRunningTime - startCount);
			
		}

		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		sideRec = 2;
		
		if(AgvCent2Right == FMSDS_Ptr->agvDirection)		// 这里找时间为time的点
		{
			maxRec = FMSDS_Ptr->AgvMSLocation;
			
			if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_0_5)		// 这里找时间为time的点
			{
				if(0 == flag2)
				{
					if(countTime >= time)
					{
						locRec1 = FMSDS_Ptr->AgvMSLocation;
						printf("Point: ");
						Show_Analy(FMSDS_Ptr);
						printf(",\tcountTime = %d\r\n", countTime);
						countTime = 0;
						flag2 = 1;
						cir = 0;
					}
					else
					{
						if(locRec4 != FMSDS_Ptr->AgvMSLocation)
						{
							locRec4 = FMSDS_Ptr->AgvMSLocation;
							countTime += FMSDS_Ptr->VelocityXt;
							cir++;
							printf("cir = %d, VelocityXt = %d, countTime = %d\r\n", cir, FMSDS_Ptr->VelocityXt, countTime);
						}
						
					}
				}
				
			}
			
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 这里回到记录的点, 开始计时并且拉
		{
			if((FMSDS_Ptr->AgvMSLocation <= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}
		
		
		if(1 == flag)	// 开始拉并且计时
		{
			if(SystemRunningTime - startCount <= time)
			{
				if(maxRec >= Agv_MS_Right_2)
				{
					index = maxRec - Agv_MS_Right_1;

					if(1 == arr[index].lock)
					{
						rmSpeedPullin = arr[index].goodDuty;
						
					}
					else
					{
						rmSpeedPullin = arr[index].duty;
						
					}
					
					lmSpeedPullin = 0;
					
					if(0 == flag3)
					{
						flag3 = 1;
						printf("MaxRecoder = %d\r\n", maxRec);
						//printf("r-%d\r\n", rmSpeedPullin);
						printf("r-index = %d, rmSpeedPullin = %d\r\n", index, rmSpeedPullin);
					}
				}
				
				
			}
			else		// 如果超出时间了, 放
			{
				
				flag3 = 0;
				flag = 2;
				
				lmSpeedPullin = 0;
				rmSpeedPullin = 0;
				flag4 = 1;
				recoverTim = SystemRunningTime;
				
			}

			printf("**ti = %d\r\n", SystemRunningTime - startCount);
			
		}

		if(2 == flag)		// 打印拉关闭log
		{
			printf("damp close***\r\n");
			flag = 3;
			
		}


		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{

		if(1 == flag)
		{
			flag4 = 1;
		}
		
		cir = 0;
		flag3 = 0;

		flag = 0;
		flag2 = 0;

		sideRec = 0;
		countTime = 0;
		startCount = 0;
		

		maxRec = Agv_MS_Center;
	}

	/******************************************************************************/
	if(1 == flag4)
	{

		if(3 == flag)
		{
			flag = 4;
			sideRecPre = sideRec;
		}
		
		if(SystemRunningTime - recoverTim > 3000)
		{
			// 开始合格判定
			flag4 = 0;

			if(sideRecPre != sideRec)		
			{
				// 如果300ms后偏的不在同一边, 则拉太小, 不合格, 差评
				arr[index].duty++;
				arr[index].result = Small;
				printf("Small***************\r\n");
				printf("index = %d, duty = %d\r\n", index, arr[index].duty);
			}
			else 
			{
				if(0 == sideRec)
				{
					// 拉太小, 不合格, 差评, 要++
					arr[index].duty++;
					arr[index].result = Small;
					printf("Small***************\r\n");
					printf("index = %d, duty = %d\r\n", index, arr[index].duty);
				}
				else if(1 == sideRec)		// 左侧
				{
					if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End))
					{
						// 拉太大, 不合格, 差评, 要--
						arr[index].duty--;
						arr[index].result = Big;
						printf("Big***************\r\n");
						printf("index = %d, duty = %d\r\n", index, arr[index].duty);
					}
					else if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Center) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1))
					{
						// 拉的刚刚好, 合格, 要记录并且lock
						arr[index].goodDuty = arr[index].duty;
						arr[index].lock = 1;
						arr[index].result = Good;
						printf("LGood***************\r\n");
						printf("index = %d, duty = %d\r\n", index, arr[index].duty);
					}
					
				}
				else if(2 == sideRec)		// 右侧
				{
					if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1))
					{
						// 拉太大, 不合格, 差评, 要--
						
						arr[index].duty--;
						arr[index].result = Big;
						printf("Big***************\r\n");
						printf("index = %d, duty = %d\r\n", index, arr[index].duty);
					}
					else if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Center))
					{
						// 拉的刚刚好, 合格, 要记录并且lock
						
						arr[index].goodDuty = arr[index].duty;
						arr[index].lock = 1;
						arr[index].result = Good;
						printf("RGood***************\r\n");
						printf("index = %d, duty = %d\r\n", index, arr[index].duty);
					}
					
				}
				
			}

			show_damp_info(arr);
			printf("\r\n");
		}
		
	}

	
	
	*lmSpeedPull = lmSpeedPullin;
	*rmSpeedPull = rmSpeedPullin;
}


void Damp_Adapter(u8 *lmSpeedPull, u8 *rmSpeedPull)
{
	static u8  flag = 0, flag2 = 0, cir = 0, flag3 = 0, flag4 = 0, sideRec = 0, sideRecPre = 0, index = 0;
	static u32 startCount = 0, countTime = 0, recoverTim = 0;
	static Agv_MS_Location locRec1 = AgvInits, locRec4 = AgvInits, maxRec = AgvInits;
	static u32 time = 3000;
	u8 lmSpeedPullin = 0, rmSpeedPullin = 0;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		sideRec = 1;
		
		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			maxRec = FMSDS_Ptr->AgvMSLocation;
			
			if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_0_5)	// 这里找时间为time的点
			{
				if(0 == flag2)
				{
					if(countTime >= time)		
					{
						locRec1 = FMSDS_Ptr->AgvMSLocation;
						printf("Point: ");
						Show_Analy(FMSDS_Ptr);
						printf(",\tcountTime = %d\r\n", countTime);
						countTime = 0;
						flag2 = 1;
						cir = 0;
					}
					else
					{
						if(locRec4 != FMSDS_Ptr->AgvMSLocation)
						{
							locRec4 = FMSDS_Ptr->AgvMSLocation;
							countTime += FMSDS_Ptr->VelocityXt;
							cir++;
							printf("cir = %d, VelocityXt = %d, countTime = %d\r\n", cir, FMSDS_Ptr->VelocityXt, countTime);
						}
						
					}
				}
				
				
			}
			
		}
		else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)	// 这里回到记录的点, 开始计时并且拉
		{
			if((FMSDS_Ptr->AgvMSLocation >= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}

		
		if(1 == flag)		// 开始拉并且计时
		{
			if(SystemRunningTime - startCount <= time)		// 如果还在时间内, 拉
			{
				if(maxRec <= Agv_MS_Left_2)
				{
					index = Agv_MS_Left_1 - maxRec;
					
					if(1 == dampAdapetInfo[index].lock)
					{
						lmSpeedPullin = dampAdapetInfo[index].goodDuty;
						
					}
					else
					{
						lmSpeedPullin = dampAdapetInfo[index].duty;
						
					}
					
					rmSpeedPullin = 0;
					
					if(0 == flag3)
					{
						flag3 = 1;
						printf("MaxRecoder = %d\r\n", maxRec);
						//printf("l-%d\r\n", lmSpeedPullin);
						printf("l-index = %d, lmSpeedPullin = %d\r\n", index, lmSpeedPullin);
					}
				}
				
			}
			else		// 如果超出时间了, 放
			{
				
				flag3 = 0;
				flag = 2;
				
				lmSpeedPullin = 0;
				rmSpeedPullin = 0;
				flag4 = 1;
				recoverTim = SystemRunningTime;
				
			}
			

			if(2 == flag)		// 打印拉关闭log
			{
				printf("damp close***\r\n");
				flag = 3;
			}


			printf("**ti = %d\r\n", SystemRunningTime - startCount);
			
		}

		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		sideRec = 2;
		
		if(AgvCent2Right == FMSDS_Ptr->agvDirection)		// 这里找时间为time的点
		{
			maxRec = FMSDS_Ptr->AgvMSLocation;
			
			if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_0_5)		// 这里找时间为time的点
			{
				if(0 == flag2)
				{
					if(countTime >= time)
					{
						locRec1 = FMSDS_Ptr->AgvMSLocation;
						printf("Point: ");
						Show_Analy(FMSDS_Ptr);
						printf(",\tcountTime = %d\r\n", countTime);
						countTime = 0;
						flag2 = 1;
						cir = 0;
					}
					else
					{
						if(locRec4 != FMSDS_Ptr->AgvMSLocation)
						{
							locRec4 = FMSDS_Ptr->AgvMSLocation;
							countTime += FMSDS_Ptr->VelocityXt;
							cir++;
							printf("cir = %d, VelocityXt = %d, countTime = %d\r\n", cir, FMSDS_Ptr->VelocityXt, countTime);
						}
						
					}
				}
				
			}
			
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 这里回到记录的点, 开始计时并且拉
		{
			if((FMSDS_Ptr->AgvMSLocation <= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}
		
		
		if(1 == flag)	// 开始拉并且计时
		{
			if(SystemRunningTime - startCount <= time)
			{
				if(maxRec >= Agv_MS_Right_2)
				{
					index = maxRec - Agv_MS_Right_1;

					if(1 == dampAdapetInfo[index].lock)
					{
						rmSpeedPullin = dampAdapetInfo[index].goodDuty;
						
					}
					else
					{
						rmSpeedPullin = dampAdapetInfo[index].duty;
						
					}
					
					lmSpeedPullin = 0;
					
					if(0 == flag3)
					{
						flag3 = 1;
						printf("MaxRecoder = %d\r\n", maxRec);
						//printf("r-%d\r\n", rmSpeedPullin);
						printf("r-index = %d, rmSpeedPullin = %d\r\n", index, rmSpeedPullin);
					}
				}
				
				
			}
			else		// 如果超出时间了, 放
			{
				
				flag3 = 0;
				flag = 2;
				
				lmSpeedPullin = 0;
				rmSpeedPullin = 0;
				flag4 = 1;
				recoverTim = SystemRunningTime;
				
			}

			printf("**ti = %d\r\n", SystemRunningTime - startCount);
			
		}

		if(2 == flag)		// 打印拉关闭log
		{
			printf("damp close***\r\n");
			flag = 3;
			
		}


		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{

		if(1 == flag)
		{
			flag4 = 1;
		}
		
		cir = 0;
		flag3 = 0;

		flag = 0;
		flag2 = 0;

		sideRec = 0;
		countTime = 0;
		startCount = 0;
		

		maxRec = Agv_MS_Center;
	}

	/******************************************************************************/
	if(1 == flag4)
	{

		if(3 == flag)
		{
			flag = 4;
			sideRecPre = sideRec;
		}
		
		if(SystemRunningTime - recoverTim > 3000)
		{
			// 开始合格判定
			flag4 = 0;

			if(sideRecPre != sideRec)		
			{
				// 如果300ms后偏的不在同一边, 则拉太小, 不合格, 差评
				dampAdapetInfo[index].duty++;
				dampAdapetInfo[index].result = Small;
				printf("Small***************\r\n");
				printf("index = %d, duty = %d\r\n", index, dampAdapetInfo[index].duty);
			}
			else 
			{
				if(0 == sideRec)
				{
					// 拉太小, 不合格, 差评, 要++
					dampAdapetInfo[index].duty++;
					dampAdapetInfo[index].result = Small;
					printf("Small***************\r\n");
					printf("index = %d, duty = %d\r\n", index, dampAdapetInfo[index].duty);
				}
				else if(1 == sideRec)		// 左侧
				{
					if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End))
					{
						// 拉太大, 不合格, 差评, 要--
						dampAdapetInfo[index].duty--;
						dampAdapetInfo[index].result = Big;
						printf("Big***************\r\n");
						printf("index = %d, duty = %d\r\n", index, dampAdapetInfo[index].duty);
					}
					else if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Center) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1))
					{
						// 拉的刚刚好, 合格, 要记录并且lock
						dampAdapetInfo[index].goodDuty = dampAdapetInfo[index].duty;
						dampAdapetInfo[index].lock = 1;
						dampAdapetInfo[index].result = Good;
						printf("LGood***************\r\n");
						printf("index = %d, duty = %d\r\n", index, dampAdapetInfo[index].duty);
					}
					
				}
				else if(2 == sideRec)		// 右侧
				{
					if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1))
					{
						// 拉太大, 不合格, 差评, 要--
						
						dampAdapetInfo[index].duty--;
						dampAdapetInfo[index].result = Big;
						printf("Big***************\r\n");
						printf("index = %d, duty = %d\r\n", index, dampAdapetInfo[index].duty);
					}
					else if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Center))
					{
						// 拉的刚刚好, 合格, 要记录并且lock
						
						dampAdapetInfo[index].goodDuty = dampAdapetInfo[index].duty;
						dampAdapetInfo[index].lock = 1;
						dampAdapetInfo[index].result = Good;
						printf("RGood***************\r\n");
						printf("index = %d, duty = %d\r\n", index, dampAdapetInfo[index].duty);
					}
					
				}
				
			}

			show_damp_info(dampAdapetInfo);
			printf("\r\n");
		}
		
	}

	
	
	*lmSpeedPull = lmSpeedPullin;
	*rmSpeedPull = rmSpeedPullin;
}

void Get_T1_Duty(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info *arr)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, re = 0;

	static u32 recT1Tim = 0;

	u8 div = 100;

	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		Tnow.T1_update = 0;
		T1LSpeedin = 0;
		T1RSpeedin = 0;
	}
	
	if(0 == Tnow.T1_update)
	{
		Get_T1(&Tnow);
	}

	
	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < 20))
		{
			
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				T1RSpeedin = arr[re].duty;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeedin = arr[re].duty;
				T1RSpeedin = 0;
			}
			
			recT1Tim = SystemRunningTime;

			Tnow.T1_update = 2;

			//TRIGGER_PIN_O = 0;
			
			printf("T1_Start re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		/*
		if((SystemRunningTime - recT1Tim >= 3000) ||\
			(AgvLeft2Cent == FMSDS_Ptr->agvDirection) || (AgvRight2Cent == FMSDS_Ptr->agvDirection) ||\
			(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_2))
		*/
		if(SystemRunningTime - recT1Tim >= 3000)
		{
			Tnow.T1_update = 3;
			
			T1LSpeedin = 0;
			T1RSpeedin = 0;
			
			//TRIGGER_PIN_O = 1;

			printf("T1_Close************\r\n");
		}
		
	}
	

	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}

void Get_T1_Duty_back(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info *arr)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, re = 0;

	static u32 recT1Tim = 0;

	u8 div = 100;


	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		Tnow.T1_update = 0;
		T1LSpeedin = 0;
		T1RSpeedin = 0;
	}
	
	if(0 == Tnow.T1_update)
	{
		Get_T1(&Tnow);
	}
	
	
	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < 20))
		{

			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				T1RSpeedin = arr[re].duty;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeedin = arr[re].duty;
				T1RSpeedin = 0;
			}
			
			recT1Tim = SystemRunningTime;

			Tnow.T1_update = 2;
			
			printf("T1_Start re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		if((SystemRunningTime - recT1Tim >= 3000) || (AgvLeft2Cent == FMSDS_Ptr->agvDirection) || (AgvRight2Cent == FMSDS_Ptr->agvDirection))
		{
			T1LSpeedin = 0;
			T1RSpeedin = 0;
			
			Tnow.T1_update = 0;
			printf("T1_Close\r\n");
		}
		
	}
	

	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}

void Get_T1_Duty2(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info *arr)
{

	static Trec Tnow;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, re = 0;

	static u32 recT1Tim = 0;

	u8 div = 100;
	
		
	Get_T1(&Tnow);
	

	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < 20))
		{
			
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				T1RSpeedin = arr[re].duty;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeedin = arr[re].duty;
				T1RSpeedin = 0;
			}
			
			recT1Tim = SystemRunningTime;

			Tnow.T1_update = 2;
			
			printf("T1_Start re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		
		if((SystemRunningTime - recT1Tim >= 3000) || (AgvLeft2Cent == FMSDS_Ptr->agvDirection) || (AgvRight2Cent == FMSDS_Ptr->agvDirection))
		{
			Tnow.T1_update = 0;
			
			T1LSpeedin = 0;
			T1RSpeedin = 0;
			printf("T1_Close\r\n");
		}
		
	}
	

	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}

void T1Damp(u8 *T1LSpeed, u8 *T1RSpeed)
{
	static u8 flag = 0;
	u8 T1LSpeedin = 0, T1RSpeedin = 0;
	
	if(((FMSDS_Ptr->MaxRecoder >= Agv_MS_Left_0_5) && (FMSDS_Ptr->MaxRecoder <= Agv_MS_Center)) ||\
		((FMSDS_Ptr->MaxRecoder >= Agv_MS_Center) && (FMSDS_Ptr->MaxRecoder <= Agv_MS_Right_0_5)))
	{
		ctrlParasPtr->T1DF = 0;
		if(1 == flag)
		{
			printf("T1Damp close2 ******\r\n");
		}
		flag = 0;
	}
	
	if(1 == ctrlParasPtr->T1DF)
	{
		if((FMSDS_Ptr->MaxRecoder >= Agv_MS_Left_2) && (FMSDS_Ptr->MaxRecoder < Agv_MS_Left_0_5))
		{
			T1LSpeedin = ctrlParasPtr->T1dutyRec / 2;
			
			if(0 == flag)
			{
				printf("T1Damp T1LSpeedin = %d ******\r\n", T1LSpeedin);
				flag = 1;
			}
		}
		else if((FMSDS_Ptr->MaxRecoder > Agv_MS_Right_0_5) && (FMSDS_Ptr->MaxRecoder <= Agv_MS_Right_2))
		{
			T1RSpeedin = ctrlParasPtr->T1dutyRec / 2;
			
			if(0 == flag)
			{
				printf("T1Damp T1RSpeedin = %d ******\r\n", T1RSpeedin);
				flag = 1;
			}
		}
		else
		{
			ctrlParasPtr->T1DF = 0;
			
			if(0 == flag)
			{
				printf("MaxL = %d\r\n", FMSDS_Ptr->MaxRecoder);
				printf("T1Damp close ******\r\n");
				flag = 1;
			}
		}
		
	}
	
	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}

void T1Damp2(u8 *T1LSpeed, u8 *T1RSpeed)
{
	static u8 flag = 0;
	u8 T1LSpeedin = 0, T1RSpeedin = 0;
	
	if(((FMSDS_Ptr->MaxRecoder >= Agv_MS_Left_0_5) && (FMSDS_Ptr->MaxRecoder <= Agv_MS_Center)) ||\
		((FMSDS_Ptr->MaxRecoder >= Agv_MS_Center) && (FMSDS_Ptr->MaxRecoder <= Agv_MS_Right_0_5)))
	{
		ctrlParasPtr->T1DF = 0;
		if(1 == flag)
		{
			printf("T1Damp close2 ******\r\n");
		}
		flag = 0;
	}
	
	if(1 == ctrlParasPtr->T1DF)
	{
		if(FMSDS_Ptr->MaxRecoder < Agv_MS_Left_0_5)
		{
			T1LSpeedin = ctrlParasPtr->T1dutyRec / 2;
			
			if(0 == flag)
			{
				printf("T1Damp T1LSpeedin = %d ******\r\n", T1LSpeedin);
				flag = 1;
			}
		}
		else if(FMSDS_Ptr->MaxRecoder > Agv_MS_Right_0_5)
		{
			T1RSpeedin = ctrlParasPtr->T1dutyRec / 2;
			
			if(0 == flag)
			{
				printf("T1Damp T1RSpeedin = %d ******\r\n", T1RSpeedin);
				flag = 1;
			}
		}
		else
		{
			ctrlParasPtr->T1DF = 0;
			
			if(0 == flag)
			{
				printf("MaxL = %d\r\n", FMSDS_Ptr->MaxRecoder);
				printf("T1Damp close ******\r\n");
				flag = 1;
			}
		}
		
	}
	
	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}

void Get_T1_Duty3(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info *arr)
{

	static Trec Tnow;
	
	static u8 T1LSpeedin = 0, T1RSpeedin = 0, re = 0;
	
	static u32 recT1Tim = 0;
	
	u8 div = 100;

	Agv_MS_Location max = AgvInits;

	if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		max = FMSDS_Ptr->AgvMSLocation;
	}
	
	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		Tnow.T1_update = 0;
		T1LSpeedin = 0;
		T1RSpeedin = 0;
		ctrlParasPtr->T1dutyRec = 0;
	}
	
	if(0 == Tnow.T1_update)
	{
		Get_T1(&Tnow);
	}

	
	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < 20))
		{
			
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				T1RSpeedin = arr[re].duty;
				ctrlParasPtr->T1dutyRec = T1RSpeedin;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeedin = arr[re].duty;
				T1RSpeedin = 0;
				ctrlParasPtr->T1dutyRec = T1LSpeedin;
			}
			
			recT1Tim = SystemRunningTime;

			Tnow.T1_update = 2;

			//TRIGGER_PIN_O = 0;
			
			printf("T1_Start re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{		
		if((SystemRunningTime - recT1Tim >= 3000) ||\
			(AgvLeft2Cent == FMSDS_Ptr->agvDirection) || (AgvRight2Cent == FMSDS_Ptr->agvDirection))
		//if((AgvLeft2Cent == FMSDS_Ptr->agvDirection) || (AgvRight2Cent == FMSDS_Ptr->agvDirection))
		{
			Tnow.T1_update = 3;

			if((AgvLeft2Cent == FMSDS_Ptr->agvDirection) || (AgvRight2Cent == FMSDS_Ptr->agvDirection))
			{
				ctrlParasPtr->T1DF = 1;
				//printf("ready for t1damp\r\n");
			}
			else
			{
				ctrlParasPtr->T1DF = 0;
			}
			
			//FMSDS_Ptr->MaxRecoder = max;
			//max = AgvInits;
			
			T1LSpeedin = 0;
			T1RSpeedin = 0;
			
			//TRIGGER_PIN_O = 1;

			printf("T1_Close************\r\n");
		}
		
	}

	if(1 == ctrlParasPtr->T1DF)
	{
		
		//T1Damp(&T1LSpeedin, &T1RSpeedin);
	}
	

	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	
}

void Get_T1_Duty4(u8 *T1LSpeed, u8 *T1RSpeed, T1_AutoAdapt_Info *arr)
{

	static Trec Tnow;
	
	static u8 T1LSpeedin = 0, T1RSpeedin = 0, re = 0;
	
	static u32 recT1Tim = 0;
	
	u8 div = 100;

	Agv_MS_Location max = AgvInits, rec = AgvInits;

	if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		max = FMSDS_Ptr->AgvMSLocation;
	}
	
	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		//printf("clean*****\r\n");
		goto Center;
		
	}
	
	if(0 == Tnow.T1_update)
	{
		Get_T1(&Tnow);
	}

	
	if(1 == Tnow.T1_update)
	{
		re = Tnow.T1 / div;

		if((re >= 0) && (re < 20))
		{
			
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				T1RSpeedin = arr[re].duty;
				ctrlParasPtr->T1dutyRec = T1RSpeedin;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeedin = arr[re].duty;
				T1RSpeedin = 0;
				ctrlParasPtr->T1dutyRec = T1LSpeedin;
			}

			recT1Tim = SystemRunningTime;
		
			Tnow.T1_update = 2;

			ctrlParasPtr->gear = 7;
			
			printf("T1_Start re = %d, T1LSpeedin = %d, T1RSpeedin = %d **********\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{		
		//if((SystemRunningTime - recT1Tim >= 3000) ||\
			//(AgvLeft2Cent == FMSDS_Ptr->agvDirection) || (AgvRight2Cent == FMSDS_Ptr->agvDirection))
		if((AgvLeft2Cent == FMSDS_Ptr->agvDirection) || (AgvRight2Cent == FMSDS_Ptr->agvDirection))
		{
			Tnow.T1_update = 3;
			
			printf("T1_Close************\r\n");

			goto CLEANPARA;
		}
		else if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_9) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_9))
		{
			printf("too small *******\r\n");
			goto BREAK2LOW;
			
		}
		else
		{
			printf("ay = %d\r\n", mpu6050DS_ptr->ayOffset);
			
			if(mpu6050DS_ptr->ayOffset != 0)
			{
				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) || (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
				{
					if(AGV_Pat_Ptr->Angle >= 0)
					{
						printf("T1_Close a************\r\n");
						goto CLEANPARA;
					}
				}
				else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) || (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
				{
					if(AGV_Pat_Ptr->Angle <= 0)
					{
						printf("T1_Close a************\r\n");
						goto CLEANPARA;
					}
				}
				
			}
		}
	}

	
	*T1LSpeed = T1LSpeedin;
	*T1RSpeed = T1RSpeedin;
	return;
	
	
	CLEANPARA:
		T1LSpeedin = 0;
		T1RSpeedin = 0;
		*T1LSpeed = T1LSpeedin;
		*T1RSpeed = T1RSpeedin;
		ctrlParasPtr->T1dutyRec = 0;
		rec = max = AgvInits;
		recT1Tim = 0;
		return;

	Center:
		Tnow.T1_update = 0;
		goto CLEANPARA;
	
	BREAK2LOW:
		ctrlParasPtr->FSflag = 0;
		goto Center;
		
		
}



void Get_Damp_Duty(u8 *lmSpeedPull, u8 *rmSpeedPull)
{
	static u8  flag = 0, flag2 = 0, cir = 0, flag3 = 0, index = 0;
	static u32 startCount = 0, countTime = 0;
	static Agv_MS_Location locRec1 = AgvInits, locRec4 = AgvInits, maxRec = AgvInits;
	static u32 time = 3000;
	u8 lmSpeedPullin = 0, rmSpeedPullin = 0;

	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			maxRec = FMSDS_Ptr->AgvMSLocation;
			
			if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_0_5)	// 这里找时间为time的点
			{
				if(0 == flag2)
				{
					if(countTime >= time)		
					{
						locRec1 = FMSDS_Ptr->AgvMSLocation;
						printf("Point: ");
						Show_Analy(FMSDS_Ptr);
						printf(",\tcountTime = %d\r\n", countTime);
						countTime = 0;
						flag2 = 1;
						cir = 0;
					}
					else
					{
						if(locRec4 != FMSDS_Ptr->AgvMSLocation)
						{
							locRec4 = FMSDS_Ptr->AgvMSLocation;
							countTime += FMSDS_Ptr->VelocityXt;
							cir++;
							printf("cir = %d, VelocityXt = %d, countTime = %d\r\n", cir, FMSDS_Ptr->VelocityXt, countTime);
						}
						
					}
				}
				
				
			}
			
		}
		else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)	// 这里回到记录的点, 开始计时并且拉
		{
			if((FMSDS_Ptr->AgvMSLocation >= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}

		
		if(1 == flag)		// 开始拉并且计时
		{
			if(SystemRunningTime - startCount <= time)		// 如果还在时间内, 拉
			{
				if(maxRec <= Agv_MS_Left_2)
				{
					index = Agv_MS_Left_1 - maxRec;
					
					if(1 == dampAdapetInfo[index].lock)
					{
						lmSpeedPullin = dampAdapetInfo[index].goodDuty;
						
					}
					else
					{
						lmSpeedPullin = dampAdapetInfo[index].duty;
						
					}
					
					rmSpeedPullin = 0;
					
					if(0 == flag3)
					{
						flag3 = 1;
						printf("MaxRecoder = %d\r\n", maxRec);
						//printf("l-%d\r\n", lmSpeedPullin);
						printf("l-index = %d, lmSpeedPullin = %d\r\n", index, lmSpeedPullin);
					}
				}
				
			}
			else		// 如果超出时间了, 放
			{
				
				flag3 = 0;
				flag = 2;
				
				lmSpeedPullin = 0;
				rmSpeedPullin = 0;
				
			}
			

			if(2 == flag)		// 打印拉关闭log
			{
				printf("damp close***\r\n");
				flag = 3;
			}


			printf("**ti = %d\r\n", SystemRunningTime - startCount);
			
		}

		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		
		if(AgvCent2Right == FMSDS_Ptr->agvDirection)		// 这里找时间为time的点
		{
			maxRec = FMSDS_Ptr->AgvMSLocation;
			
			if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_0_5)		// 这里找时间为time的点
			{
				if(0 == flag2)
				{
					if(countTime >= time)
					{
						locRec1 = FMSDS_Ptr->AgvMSLocation;
						printf("Point: ");
						Show_Analy(FMSDS_Ptr);
						printf(",\tcountTime = %d\r\n", countTime);
						countTime = 0;
						flag2 = 1;
						cir = 0;
					}
					else
					{
						if(locRec4 != FMSDS_Ptr->AgvMSLocation)
						{
							locRec4 = FMSDS_Ptr->AgvMSLocation;
							countTime += FMSDS_Ptr->VelocityXt;
							cir++;
							printf("cir = %d, VelocityXt = %d, countTime = %d\r\n", cir, FMSDS_Ptr->VelocityXt, countTime);
						}
						
					}
				}
				
			}
			
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 这里回到记录的点, 开始计时并且拉
		{
			if((FMSDS_Ptr->AgvMSLocation <= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}
		
		
		if(1 == flag)	// 开始拉并且计时
		{
			if(SystemRunningTime - startCount <= time)
			{
				if(maxRec >= Agv_MS_Right_2)
				{
					index = maxRec - Agv_MS_Right_1;

					if(1 == dampAdapetInfo[index].lock)
					{
						rmSpeedPullin = dampAdapetInfo[index].goodDuty;
						
					}
					else
					{
						rmSpeedPullin = dampAdapetInfo[index].duty;
						
					}
					
					lmSpeedPullin = 0;
					
					if(0 == flag3)
					{
						flag3 = 1;
						printf("MaxRecoder = %d\r\n", maxRec);
						//printf("r-%d\r\n", rmSpeedPullin);
						printf("r-index = %d, rmSpeedPullin = %d\r\n", index, rmSpeedPullin);
					}
				}
				
				
			}
			else		// 如果超出时间了, 放
			{
				
				flag3 = 0;
				flag = 2;
				
				lmSpeedPullin = 0;
				rmSpeedPullin = 0;
				
			}

			printf("**ti = %d\r\n", SystemRunningTime - startCount);
			
		}

		if(2 == flag)		// 打印拉关闭log
		{
			printf("damp close***\r\n");
			flag = 3;
			
		}


		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		
		cir = 0;
		flag3 = 0;

		flag = 0;
		flag2 = 0;

		countTime = 0;
		startCount = 0;
		

		maxRec = Agv_MS_Center;
	}



	*lmSpeedPull = lmSpeedPullin;
	*rmSpeedPull = rmSpeedPullin;
}

void Get_Damp_Duty_Back(u8 *lmSpeedPull, u8 *rmSpeedPull)
{
	static u8  flag = 0, flag2 = 0, cir = 0, flag3 = 0, index = 0;
	static u32 startCount = 0, countTime = 0;
	static Agv_MS_Location locRec1 = AgvInits, locRec4 = AgvInits, maxRec = AgvInits;
	static u32 time = 3000;
	u8 lmSpeedPullin = 0, rmSpeedPullin = 0;

	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			maxRec = FMSDS_Ptr->AgvMSLocation;
			
			if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_0_5)	// 这里找时间为time的点
			{
				if(0 == flag2)
				{
					if(countTime >= time)		
					{
						locRec1 = FMSDS_Ptr->AgvMSLocation;
						printf("Point: ");
						Show_Analy(FMSDS_Ptr);
						printf(",\tcountTime = %d\r\n", countTime);
						countTime = 0;
						flag2 = 1;
						cir = 0;
					}
					else
					{
						if(locRec4 != FMSDS_Ptr->AgvMSLocation)
						{
							locRec4 = FMSDS_Ptr->AgvMSLocation;
							countTime += FMSDS_Ptr->VelocityXt;
							cir++;
							printf("cir = %d, VelocityXt = %d, countTime = %d\r\n", cir, FMSDS_Ptr->VelocityXt, countTime);
						}
						
					}
				}
				
				
			}
			
		}
		else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)	// 这里回到记录的点, 开始计时并且拉
		{
			if((FMSDS_Ptr->AgvMSLocation >= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}

		
		if(1 == flag)		// 开始拉并且计时
		{
			if(SystemRunningTime - startCount <= time)		// 如果还在时间内, 拉
			{
				if(maxRec <= Agv_MS_Left_2)
				{
					index = Agv_MS_Left_1 - maxRec;
					
					if(1 == dampAdapetInfoB[index].lock)
					{
						lmSpeedPullin = dampAdapetInfoB[index].goodDuty;
						
					}
					else
					{
						lmSpeedPullin = dampAdapetInfoB[index].duty;
						
					}
					
					rmSpeedPullin = 0;
					
					if(0 == flag3)
					{
						flag3 = 1;
						printf("MaxRecoder = %d\r\n", maxRec);
						//printf("l-%d\r\n", lmSpeedPullin);
						printf("l-index = %d, lmSpeedPullin = %d\r\n", index, lmSpeedPullin);
					}
				}
				
			}
			else		// 如果超出时间了, 放
			{
				
				flag3 = 0;
				flag = 2;
				
				lmSpeedPullin = 0;
				rmSpeedPullin = 0;
				
			}
			

			if(2 == flag)		// 打印拉关闭log
			{
				printf("damp close***\r\n");
				flag = 3;
			}


			printf("**ti = %d\r\n", SystemRunningTime - startCount);
			
		}

		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		
		if(AgvCent2Right == FMSDS_Ptr->agvDirection)		// 这里找时间为time的点
		{
			maxRec = FMSDS_Ptr->AgvMSLocation;
			
			if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_0_5)		// 这里找时间为time的点
			{
				if(0 == flag2)
				{
					if(countTime >= time)
					{
						locRec1 = FMSDS_Ptr->AgvMSLocation;
						printf("Point: ");
						Show_Analy(FMSDS_Ptr);
						printf(",\tcountTime = %d\r\n", countTime);
						countTime = 0;
						flag2 = 1;
						cir = 0;
					}
					else
					{
						if(locRec4 != FMSDS_Ptr->AgvMSLocation)
						{
							locRec4 = FMSDS_Ptr->AgvMSLocation;
							countTime += FMSDS_Ptr->VelocityXt;
							cir++;
							printf("cir = %d, VelocityXt = %d, countTime = %d\r\n", cir, FMSDS_Ptr->VelocityXt, countTime);
						}
						
					}
				}
				
			}
			
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 这里回到记录的点, 开始计时并且拉
		{
			if((FMSDS_Ptr->AgvMSLocation <= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}
		
		
		if(1 == flag)	// 开始拉并且计时
		{
			if(SystemRunningTime - startCount <= time)
			{
				if(maxRec >= Agv_MS_Right_2)
				{
					index = maxRec - Agv_MS_Right_1;

					if(1 == dampAdapetInfoB[index].lock)
					{
						rmSpeedPullin = dampAdapetInfoB[index].goodDuty;
						
					}
					else
					{
						rmSpeedPullin = dampAdapetInfoB[index].duty;
						
					}
					
					lmSpeedPullin = 0;
					
					if(0 == flag3)
					{
						flag3 = 1;
						printf("MaxRecoder = %d\r\n", maxRec);
						//printf("r-%d\r\n", rmSpeedPullin);
						printf("r-index = %d, rmSpeedPullin = %d\r\n", index, rmSpeedPullin);
					}
				}
				
				
			}
			else		// 如果超出时间了, 放
			{
				
				flag3 = 0;
				flag = 2;
				
				lmSpeedPullin = 0;
				rmSpeedPullin = 0;
				
			}

			printf("**ti = %d\r\n", SystemRunningTime - startCount);
			
		}

		if(2 == flag)		// 打印拉关闭log
		{
			printf("damp close***\r\n");
			flag = 3;
			
		}


		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		
		cir = 0;
		flag3 = 0;

		flag = 0;
		flag2 = 0;

		countTime = 0;
		startCount = 0;
		

		maxRec = Agv_MS_Center;
	}



	*lmSpeedPull = lmSpeedPullin;
	*rmSpeedPull = rmSpeedPullin;
}


void Get_Damp_Duty2(u8 *lmSpeedPull, u8 *rmSpeedPull)
{
	static Agv_MS_Location maxRec = AgvInits;
	u8 lmSpeedPullin = 0, rmSpeedPullin = 0;
	u8 duty[7] = {1, 2, 4, 5, 6, 7, 8};
	//u8 duty2[9][7] = {{}, {}, {}, {}, {}, {}, {}, {}, {}};

	if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
	{
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_1))
			{
				if((AGV_Pat_Ptr->Angle >= 2) && (AGV_Pat_Ptr->Angle <= 8))
				{
					lmSpeedPullin = duty[AGV_Pat_Ptr->Angle - 2];
				}

			}			
						
		}
		
	}
	else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
	{
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{
			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_5))
			{
				if((AGV_Pat_Ptr->Angle >= -8) && (AGV_Pat_Ptr->Angle <= -2))
				{
					rmSpeedPullin = duty[ABSOLU(AGV_Pat_Ptr->Angle) - 2];
				}
			}
			
		}
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		
		
		maxRec = Agv_MS_Center;
	}



	*lmSpeedPull = lmSpeedPullin;
	*rmSpeedPull = rmSpeedPullin;
}





void scale_1_mode16_dampadapt(u8 gear)
{
	u8 AgvGearS1CDLF[20] = {1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0;
	// 普通模式,偏差在1格之内调整
	
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		ctrlParasPtr->comflag = 641;
		
		
		rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
	
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;
		
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
	
		lmSpeed = 0;
		rmSpeed = 0;
		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	#if 0
	Damp_Adapter_Com(&lmSpeedPull, &rmSpeedPull, dampAdapetInfo);
	#else
	Get_Damp_Duty(&lmSpeedPull, &rmSpeedPull);
	#endif
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - lmSpeedPull;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - rmSpeedPull;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}

void scale_1_mode16_dampadapt_back(u8 gear)
{
	u8 AgvGearS1CDLF[20] = {1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0;
	// 普通模式,偏差在1格之内调整
	
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		ctrlParasPtr->comflag = 641;
		
		
		rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
	
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;
		
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
	
		lmSpeed = 0;
		rmSpeed = 0;
		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}
	#if 0
	Damp_Adapter_Com(&lmSpeedPull, &rmSpeedPull, dampAdapetInfoB);
	#else
	Get_Damp_Duty(&lmSpeedPull, &rmSpeedPull);
	#endif
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - lmSpeed - lmSpeedPull;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - rmSpeed - rmSpeedPull;
	
	damping_func(1000, gearRecod, rmSpeedSet, lmSpeedSet);
	
	
}


void scale_1_mode17(u8 gear)
{
	u8 AgvGearS1CDLF[20] = {1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0,  lmSpeedT1 = 0, rmSpeedT1 = 0;
	// 普通模式,偏差在1格之内调整
	
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		ctrlParasPtr->comflag = 641;
		
		rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
		
		lmSpeed = 0;
		rmSpeed = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	//T1_Adapter3(&lmSpeedT1, &rmSpeedT1, adaptInfo2);
	
	
	//Get_T1_Duty4(&lmSpeedT1, &rmSpeedT1, adaptInfo);
	
	//T1_Adapter4(&lmSpeedT1, &rmSpeedT1, adaptInfo);
	
	Get_Damp_Duty(&lmSpeedPull, &rmSpeedPull);
	
	if((0 == lmSpeedPull) && (0 == rmSpeedPull))
	{
		Get_Damp_Duty(&lmSpeedT1, &rmSpeedT1);
		
	}
	
	
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - lmSpeedPull - lmSpeedT1;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - rmSpeedPull - rmSpeedT1;
	
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}


void scale_1_mode17_back(u8 gear)
{
	u8 AgvGearS1CDLF[20] = {1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0,  lmSpeedT1 = 0, rmSpeedT1 = 0;
	// 普通模式,偏差在1格之内调整
	
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		ctrlParasPtr->comflag = 641;
		
		
		rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
	
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;
		
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
	
		lmSpeed = 0;
		rmSpeed = 0;
		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}
	
	//T1_Adapter3_back(&lmSpeedT1, &rmSpeedT1, adaptInfoB2);

	//Get_T1_Duty_back(&lmSpeedT1, &rmSpeedT1, adaptInfoB);
	//T1_Adapter_back(&lmSpeedT1, &rmSpeedT1);
	
	Get_Damp_Duty_Back(&lmSpeedPull, &rmSpeedPull);
	
	if((0 == lmSpeedPull) && (0 == rmSpeedPull))
	{
		Get_Damp_Duty_Back(&lmSpeedT1, &rmSpeedT1);
		
	}
	
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - lmSpeed - lmSpeedPull - lmSpeedT1;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - rmSpeed - rmSpeedPull - rmSpeedT1;
	
	damping_func(1000, gearRecod, rmSpeedSet, lmSpeedSet);
	
	
}


void Pattern_ctrl(u8 *T1LSpeed, u8 *T1RSpeed)
{
	u8 lmSpeed = 0, rmSpeed = 0;
	static Agv_MS_Location msRec = AgvInits;
	
	ctrlParasPtr->comflag = 64;
	
	if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
	{
		if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)
		{
			if(AGV_Pat_Ptr->Angle > 2)
			{
				lmSpeed = 2;
				
				if(msRec != FMSDS_Ptr->AgvMSLocation)
				{
					msRec = FMSDS_Ptr->AgvMSLocation;
					printf("lmSpeed = 1\r\n");
				}
			}
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_1_5) || (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_2))
		{
			if(AGV_Pat_Ptr->Angle > 4)
			{
				lmSpeed = 3;
				
				if(msRec != FMSDS_Ptr->AgvMSLocation)
				{
					msRec = FMSDS_Ptr->AgvMSLocation;
					printf("lmSpeed = 2\r\n");
				}
			}
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_2_5) || (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_3))
		{
			if(AGV_Pat_Ptr->Angle > 6)
			{
				lmSpeed = 4;
				
				if(msRec != FMSDS_Ptr->AgvMSLocation)
				{
					msRec = FMSDS_Ptr->AgvMSLocation;
					printf("lmSpeed = 3\r\n");
				}
			}
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_3_5) || (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_4))
		{
			if(AGV_Pat_Ptr->Angle > 8)
			{
				lmSpeed = 5;
				
				if(msRec != FMSDS_Ptr->AgvMSLocation)
				{
					msRec = FMSDS_Ptr->AgvMSLocation;
					printf("lmSpeed = 4\r\n");
				}
			}
		}
		
	}
	else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
	{
		if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)
		{
			if(AGV_Pat_Ptr->Angle < -2)
			{
				rmSpeed = 2;

				if(msRec != FMSDS_Ptr->AgvMSLocation)
				{
					msRec = FMSDS_Ptr->AgvMSLocation;
					printf("rmSpeed = 1\r\n");
				}
			}
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_1_5) || (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_2))
		{
			if(AGV_Pat_Ptr->Angle < -4)
			{
				rmSpeed = 3;

				if(msRec != FMSDS_Ptr->AgvMSLocation)
				{
					msRec = FMSDS_Ptr->AgvMSLocation;
					printf("rmSpeed = 2\r\n");
				}
			}
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_2_5) || (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_3))
		{
			if(AGV_Pat_Ptr->Angle < -6)
			{
				rmSpeed = 4;

				if(msRec != FMSDS_Ptr->AgvMSLocation)
				{
					msRec = FMSDS_Ptr->AgvMSLocation;
					printf("rmSpeed = 3\r\n");
				}
			}
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_3_5) || (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_4))
		{
			if(AGV_Pat_Ptr->Angle < -8)
			{
				rmSpeed = 5;

				if(msRec != FMSDS_Ptr->AgvMSLocation)
				{
					msRec = FMSDS_Ptr->AgvMSLocation;
					printf("rmSpeed = 4\r\n");
				}
			}
		}
		
	}

	
	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		lmSpeed = 0;
		rmSpeed = 0;
	}
	
	
	*T1LSpeed = lmSpeed;
	*T1RSpeed = rmSpeed;
	
	return;
		
}

void Pattern_ctrl1(u8 *T1LSpeed, u8 *T1RSpeed)
{
	u8 lmSpeed = 0, rmSpeed = 0;
	static Agv_MS_Location msRec = AgvInits;
	
	ctrlParasPtr->comflag = 64;
	
	if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
	{
		if((AGV_Pat_Ptr->Angle >= 0) && (AGV_Pat_Ptr->Angle <= 2))
		{
			
		}
		else if(AGV_Pat_Ptr->Angle > 2)
		{
			lmSpeed = 4;
		
			if(msRec != FMSDS_Ptr->AgvMSLocation)
			{
				msRec = FMSDS_Ptr->AgvMSLocation;
				printf("lmSpeed = %d\r\n", lmSpeed);
			}
		}
		
	}
	else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
	{
		if((AGV_Pat_Ptr->Angle <= 0) && (AGV_Pat_Ptr->Angle >= -2))
		{

		}
		else if(AGV_Pat_Ptr->Angle < -2)
		{
			
			rmSpeed = 4;

			if(msRec != FMSDS_Ptr->AgvMSLocation)
			{
				msRec = FMSDS_Ptr->AgvMSLocation;
				printf("rmSpeed = %d\r\n", rmSpeed);
			}
		}
		
	}

	
	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		lmSpeed = 0;
		rmSpeed = 0;
	}
	
	
	*T1LSpeed = lmSpeed;
	*T1RSpeed = rmSpeed;
	
	return;
		
}

void Pattern_ctrl2(u8 *T1LSpeed, u8 *T1RSpeed)
{
	u8 lmSpeed = 0, rmSpeed = 0;
	static Agv_MS_Location msRec = AgvInits;
	
	ctrlParasPtr->comflag = 64;

	if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_2) ||\
		(AGV_Pat_Ptr->Angle > 6) || (AGV_Pat_Ptr->Angle < -6))
	{
		
		if(AGV_Pat_Ptr->Angle > 0)
		{
			if((AGV_Pat_Ptr->Angle > 0) && (AGV_Pat_Ptr->Angle <= 2))
			{
				if(AGV_Pat_Ptr->AngleDirection < 0)
				{
					lmSpeed = 0;
					rmSpeed = 0;
				}
				
			}
			else if((AGV_Pat_Ptr->Angle > 2) && (AGV_Pat_Ptr->Angle <= 4))
			{
				if(AGV_Pat_Ptr->AngleDirection < 0)
				{
					lmSpeed = 0;
					rmSpeed = 0;
				}
				else
				{
					lmSpeed = 1;
				}
				
			}
			else if((AGV_Pat_Ptr->Angle > 4) && (AGV_Pat_Ptr->Angle <= 6))
			{
				lmSpeed = 2;
			}
			else
			{
				lmSpeed = 3;
			}
			
		}
		else if(AGV_Pat_Ptr->Angle < 0)
		{
			if((AGV_Pat_Ptr->Angle < 0) && (AGV_Pat_Ptr->Angle >= -2))
			{
				if(AGV_Pat_Ptr->AngleDirection < 0)
				{
					lmSpeed = 0;
					rmSpeed = 0;
				}
				
			}
			else if((AGV_Pat_Ptr->Angle < -2) && (AGV_Pat_Ptr->Angle >= -4))
			{
				if(AGV_Pat_Ptr->AngleDirection < 0)
				{
					lmSpeed = 0;
					rmSpeed = 0;
				}
				else
				{
					rmSpeed = 1;
				}
			}
			else if((AGV_Pat_Ptr->Angle < -4) && (AGV_Pat_Ptr->Angle >= -6))
			{
				rmSpeed = 2;
			}
			else
			{
				rmSpeed = 3;
			}
			
		}
		
		if(msRec != FMSDS_Ptr->AgvMSLocation)
		{
			msRec = FMSDS_Ptr->AgvMSLocation;
			printf("*******lmSpeed = %d, rmSpeed = %d ********\r\n", lmSpeed, rmSpeed);
		}
		
	}
	
	*T1LSpeed = lmSpeed;
	*T1RSpeed = rmSpeed;
	
	return;
		
}

void Software_PWM(u8 *SPWMLSpeed, u8 *SPWMRSpeed)
{
	static u32 timeCount = 0, pwmTime = 0;
	static Agv_MS_Location msRec = AgvInits;
	u8 SPWMLSpeedSet = 0, SPWMRSpeedSet = 0;
	static u8 step = 0, printfFlag = 0;

	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		msRec = AgvInits;
		timeCount = 0;
		step = 0;
		
		if(1 == printfFlag)
		{
			printf("pwmClose1\r\n");
		}
		printfFlag = 0;
	}
	else
	{
		
		if(msRec != FMSDS_Ptr->AgvMSLocation)
		{
			msRec = FMSDS_Ptr->AgvMSLocation;
			timeCount = SystemRunningTime;
			step = 0;
			
			if(1 == printfFlag)
			{
				printf("pwmClose2\r\n");
			}
			printfFlag = 0;
		}
		else
		{
			if(0 == step)
			{
				if(SystemRunningTime - timeCount > 5000)
				{
					step = 1;
					pwmTime = SystemRunningTime;
					
					if(0 == printfFlag)
					{
						printf("pwmStart\r\n");
						printfFlag = 1;
					}
					
				}
			}
			
		}
		
	}

	if(1 == step)
	{
		if(SystemRunningTime - pwmTime <= 1000)
		{
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				SPWMRSpeedSet = 1;
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				SPWMLSpeedSet = 1;
			}
			
		}
		else
		{
			step = 0;
			timeCount = SystemRunningTime;
			
			printf("pwmClose\r\n");
		}
	}
	

	*SPWMLSpeed = SPWMLSpeedSet;
	*SPWMRSpeed = SPWMRSpeedSet;
	return;
	
}

void Pattern_ctrl3(u8 *T1LSpeed, u8 *T1RSpeed)
{
	u8 lmSpeed = 0, rmSpeed = 0;
	static Agv_MS_Location msRec = AgvInits;
	//static u8 angArr[4] = {1, 2, 3, 4};
	
	ctrlParasPtr->comflag = 64;

	//if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_2))
	if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5))
	{
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			
			if(AGV_Pat_Ptr->Angle > 1)
			{
				if(AGV_Pat_Ptr->Angle <= 9)
				{
					if((9 == AGV_Pat_Ptr->Angle) || (8 == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 4;
					}
					else if((7 == AGV_Pat_Ptr->Angle) || (6 == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 3;
					}
					else if((5 == AGV_Pat_Ptr->Angle) || (4 == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 2;
					}
					else if((3 == AGV_Pat_Ptr->Angle) || (2 == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 1;
					}
				}
				else
				{
					rmSpeed = 5;
				}
				
				//lmSpeed = 0;
			}
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{
			if(AGV_Pat_Ptr->Angle < -1)
			{
				if(AGV_Pat_Ptr->Angle >= -9)
				{
					if((-9 == AGV_Pat_Ptr->Angle) || (-8 == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 4;
					}
					else if((-7 == AGV_Pat_Ptr->Angle) || (-6 == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 3;
					}
					else if((-5 == AGV_Pat_Ptr->Angle) || (-4 == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 2;
					}
					else if((-3 == AGV_Pat_Ptr->Angle) || (-2 == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 1;
					}
					
					//rmSpeed = angArr[(abs(AGV_Pat_Ptr->Angle) / 2) - 2];
				}
				else
				{
					rmSpeed = 5;
				}
				
				//lmSpeed = 0;
				
			}
			
		}
		
		if(msRec != FMSDS_Ptr->AgvMSLocation)
		{
			msRec = FMSDS_Ptr->AgvMSLocation;
			printf("*******lmSpeed = %d, rmSpeed = %d ********\r\n", lmSpeed, rmSpeed);
		}
		
	}
	
	*T1LSpeed = lmSpeed;
	*T1RSpeed = rmSpeed;
	
	return;
	
}

void Pattern_ctrl4(u8 *T1LSpeed, u8 *T1RSpeed)
{
	u8 lmSpeed = 0, rmSpeed = 0;
	static Agv_MS_Location msRec = AgvInits;
	static u8 angArr[4] = {1, 2, 3, 4};
	s16 agvLimt = 0;
	
	ctrlParasPtr->comflag = 64;

	//if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_2))
	//if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5))
	if(1)
	{
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_4)
			{
				
				if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_0_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_1))
				{
					agvLimt = 1;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_1_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_2))
				{
					agvLimt = 2;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_2_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_3))
				{
					agvLimt = 3;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_3_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_4))
				{
					agvLimt = 4;
				}
				
			}
			else
			{
				agvLimt = 5;
			}
			
			if(AGV_Pat_Ptr->Angle > agvLimt)
			{
				if(AGV_Pat_Ptr->Angle <= (agvLimt + 8))
				{
					if(((agvLimt + 8) == AGV_Pat_Ptr->Angle) || ((agvLimt + 7) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 4;
					}
					else if(((agvLimt + 6) == AGV_Pat_Ptr->Angle) || ((agvLimt + 5) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 3;
					}
					else if(((agvLimt + 4) == AGV_Pat_Ptr->Angle) || ((agvLimt + 3) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 2;
					}
					else if(((agvLimt + 2) == AGV_Pat_Ptr->Angle) || ((agvLimt + 1) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 1;
					}
				}
				else
				{
					rmSpeed = 5;
				}
				
				//lmSpeed = 0;
			}
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_4)
			{
				
				if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_0_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_1))
				{
					agvLimt = -1;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_1_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_2))
				{
					agvLimt = -2;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_2_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_3))
				{
					agvLimt = -3;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_3_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_4))
				{
					agvLimt = -4;
				}
				
			}
			else
			{
				agvLimt = -5;
			}
			
			if(AGV_Pat_Ptr->Angle < -1)
			{
				if(AGV_Pat_Ptr->Angle >= (agvLimt - 8))
				{
					if(((agvLimt - 8) == AGV_Pat_Ptr->Angle) || ((agvLimt - 7) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 4;
					}
					else if(((agvLimt - 5) == AGV_Pat_Ptr->Angle) || ((agvLimt - 5) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 3;
					}
					else if(((agvLimt - 4) == AGV_Pat_Ptr->Angle) || ((agvLimt - 3) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 2;
					}
					else if(((agvLimt - 2) == AGV_Pat_Ptr->Angle) || ((agvLimt - 1) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 1;
					}
					
					//rmSpeed = angArr[(abs(AGV_Pat_Ptr->Angle) / 2) - 2];
				}
				else
				{
					rmSpeed = 5;
				}
				
				//lmSpeed = 0;
				
			}
			
		}
		
		if(msRec != FMSDS_Ptr->AgvMSLocation)
		{
			msRec = FMSDS_Ptr->AgvMSLocation;
			printf("******* lmSpeed = %d, rmSpeed = %d ********\r\n", lmSpeed, rmSpeed);
		}
		
	}
	
	*T1LSpeed = lmSpeed;
	*T1RSpeed = rmSpeed;
	
	return;
	
}

void Pattern_ctrl5(u8 *T1LSpeed, u8 *T1RSpeed)
{
	u8 lmSpeed = 0, rmSpeed = 0;
	static Agv_MS_Location msRec = AgvInits;
	static u8 angArr[4] = {1, 2, 3, 4};
	s16 agvLimt = 0;
	
	ctrlParasPtr->comflag = 64;

	//if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_2))
	//if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5))
	if(1)
	{
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_4)
			{
				
				if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_0_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_1))
				{
					agvLimt = 1;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_1_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_2))
				{
					agvLimt = 2;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_2_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_3))
				{
					agvLimt = 3;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_3_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Left_4))
				{
					agvLimt = 4;
				}
				
			}
			else
			{
				agvLimt = 5;
			}
			
			if(AGV_Pat_Ptr->Angle > agvLimt)
			{
				if(AGV_Pat_Ptr->Angle <= (agvLimt + 8))
				{
					if(((agvLimt + 8) == AGV_Pat_Ptr->Angle) || ((agvLimt + 7) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 4;
					}
					else if(((agvLimt + 6) == AGV_Pat_Ptr->Angle) || ((agvLimt + 5) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 3;
					}
					else if(((agvLimt + 4) == AGV_Pat_Ptr->Angle) || ((agvLimt + 3) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 2;
					}
					else if(((agvLimt + 2) == AGV_Pat_Ptr->Angle) || ((agvLimt + 1) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 1;
					}
				}
				else
				{
					rmSpeed = 5;
				}
				
				//lmSpeed = 0;
			}
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_4)
			{
				
				if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_0_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_1))
				{
					agvLimt = -1;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_1_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_2))
				{
					agvLimt = -2;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_2_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_3))
				{
					agvLimt = -3;
				}
				else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_3_5) && (FMSDS_Ptr->AgvMSLocation == Agv_MS_Right_4))
				{
					agvLimt = -4;
				}
				
			}
			else
			{
				agvLimt = -5;
			}
			
			if(AGV_Pat_Ptr->Angle < -1)
			{
				if(AGV_Pat_Ptr->Angle >= (agvLimt - 8))
				{
					if(((agvLimt - 8) == AGV_Pat_Ptr->Angle) || ((agvLimt - 7) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 4;
					}
					else if(((agvLimt - 5) == AGV_Pat_Ptr->Angle) || ((agvLimt - 5) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 3;
					}
					else if(((agvLimt - 4) == AGV_Pat_Ptr->Angle) || ((agvLimt - 3) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 2;
					}
					else if(((agvLimt - 2) == AGV_Pat_Ptr->Angle) || ((agvLimt - 1) == AGV_Pat_Ptr->Angle))
					{
						rmSpeed = 1;
					}
					
					//rmSpeed = angArr[(abs(AGV_Pat_Ptr->Angle) / 2) - 2];
				}
				else
				{
					rmSpeed = 5;
				}
				
				//lmSpeed = 0;
				
			}
			
		}
		
		if(msRec != FMSDS_Ptr->AgvMSLocation)
		{
			msRec = FMSDS_Ptr->AgvMSLocation;
			//printf("*******lmSpeed = %d, rmSpeed = %d ********\r\n", lmSpeed, rmSpeed);
		}
		
	}
	
	*T1LSpeed = lmSpeed;
	*T1RSpeed = rmSpeed;
	
	return;
	
}


void get_Pull_Duty(u8 *PullLSpeed, u8 *PullRSpeed, T1_AutoAdapt_Info *arr)
{

	static u8 PullLSpeedin = 0, PullRSpeedin = 0, re = 0, step = 0;

	static u32 recT1Tim = 0;

	u8 div = 100;

	if(0 == step)
	{
		if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
		{
			if(Agv_MS_Left_2 == FMSDS_Ptr->AgvMSLocation)
			{
				//re = (FMSDS_Ptr->VelocityXt + FMSDS_Pre_Ptr->VelocityXt) / div;
				re = FMSDS_Ptr->VelocityXt / div;

				if((re >= 0) && (re < 20))
				{
					
					PullLSpeedin = 0;
					PullRSpeedin = arr[re].duty;
					
					recT1Tim = SystemRunningTime;

					step = 1;
					printf("PULL_Start re = %d, PullLSpeedin = %d, PullRSpeedin = %d\r\n", re, PullLSpeedin, PullRSpeedin);
				}
				
			}
			else if(Agv_MS_Right_2 == FMSDS_Ptr->AgvMSLocation)
			{
				re = (FMSDS_Ptr->VelocityXt + FMSDS_Pre_Ptr->VelocityXt) / div;

				if((re >= 0) && (re < 20))
				{
					
					PullLSpeedin = arr[re].duty;
					PullRSpeedin = 0;
					
					recT1Tim = SystemRunningTime;

					step = 1;
					printf("PULL_Start re = %d, PullLSpeedin = %d, PullRSpeedin = %d\r\n", re, PullLSpeedin, PullRSpeedin);
				}
				
			}
		}
	}
	

	if(1 == step)
	{
		if(SystemRunningTime - recT1Tim > 3000)
		{
			printf("pull close timeout\r\n");
			step = 2;

			PullLSpeedin = 0;
			PullRSpeedin = 0;
		}
		/*
		else if(AGV_Pat_Ptr->AngleDirection < 0)
		{
			printf("pull close AngleDirection\r\n");
			step = 2;

			PullLSpeedin = 0;
			PullRSpeedin = 0;
		}
		*/
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
		{
			printf("pull close AgvRight2Cent\r\n");
			step = 2;

			PullLSpeedin = 0;
			PullRSpeedin = 0;
		}
		else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
		{
			printf("pull close AgvLeft2Cent\r\n");
			step = 2;

			PullLSpeedin = 0;
			PullRSpeedin = 0;
		}
		
	}

	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		step = 0;

		PullLSpeedin = 0;
		PullRSpeedin = 0;
	}

	*PullLSpeed = PullLSpeedin;
	*PullRSpeed = PullRSpeedin;
}

void get_Pull_Duty2(u8 *PullLSpeed, u8 *PullRSpeed)
{

	static u8 PullLSpeedin = 0, PullRSpeedin = 0, re = 0, step = 0;

	static u32 recT1Tim = 0;

	u8 div = 100;

	if(0 == step)
	{
		if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_3) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3))
		{
			
			if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3)
			{
				PullLSpeedin = 0;
				PullRSpeedin = 10;
			}
			else
			{
				PullLSpeedin = 10;
				PullRSpeedin = 0;
			}
			
			recT1Tim = SystemRunningTime;
			step = 1;
			
		}
	}

	if(1 == step)
	{
		if((SystemRunningTime - recT1Tim > 2000))
		{
			PullLSpeedin = 0;
			PullRSpeedin = 0;

			step = 0;
		}
	}

	*PullLSpeed = PullLSpeedin;
	*PullRSpeed = PullRSpeedin;
}


void scale_1_mode18(u8 gear)
{
	//u8 AgvGearS1CDLF[20] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10};
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0,	lmSpeedT1 = 0, rmSpeedT1 = 0, lmSpeedPat = 0, rmSpeedPat = 0;
	u8 softwarePWML = 0, softwarePWMR = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	// 普通模式,偏差在1格之内调整
	u8 AgvGearS1CDLF[20] = {1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10, 10};
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		ctrlParasPtr->comflag = 641;
		
		if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_4)
		{
			//ctrlParasPtr->gear = 8;
		}
		
		rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;
		
		if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_4)
		{
			//ctrlParasPtr->gear = 8;
		}
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
		
		lmSpeed = 0;
		rmSpeed = 0;
		
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 633;
			
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
			}
			
			if(centCount > 7000)
			{
				ctrlParasPtr->comflag = 6331;
				
				if(10 != ctrlParasPtr->gear)
				{
					//ctrlParasPtr->gear = 10;
				}
				
				startCount = 0;
				
			}
			
		}
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}
	
	//Get_T1_Duty3(&lmSpeedT1, &rmSpeedT1, adaptInfo);
	
	Pattern_ctrl3(&lmSpeedPat, &rmSpeedPat);
	
	//Software_PWM(&softwarePWML, &softwarePWMR);

	//get_Pull_Duty(&lmSpeedPull, &rmSpeedPull, adaptInfo);
	
	//T1_Adapter3(&lmSpeedT1, &rmSpeedT1, adaptInfo2);
	
	//T1_Adapter4(&lmSpeedT1, &rmSpeedT1, adaptInfo);
	
	//Get_Damp_Duty(&lmSpeedPull, &rmSpeedPull);
	
	if((0 == lmSpeedPull) && (0 == rmSpeedPull))
	{
		//Get_Damp_Duty(&lmSpeedT1, &rmSpeedT1);
		
	}
	

	if((AgvRight2Cent == FMSDS_Ptr->agvDirection) || (AgvLeft2Cent == FMSDS_Ptr->agvDirection))
	{
		lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedPull - lmSpeedT1 - lmSpeedPat - softwarePWML;
	
		rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedPull - rmSpeedT1 - rmSpeedPat - softwarePWMR;
	}
	else if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - lmSpeedPull - lmSpeedT1;
	
		rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - rmSpeedPull - rmSpeedT1;
	}
	
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}

void scale_1_mode18_back(u8 gear)
{
	//u8 AgvGearS1CDLF[20] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10};
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0,	lmSpeedT1 = 0, rmSpeedT1 = 0, lmSpeedPat = 0, rmSpeedPat = 0;
	u8 softwarePWML = 0, softwarePWMR = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	// 普通模式,偏差在1格之内调整
	u8 AgvGearS1CDLF[20] = {1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10, 10};
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		
		ctrlParasPtr->comflag = 641;
		
		if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_4)
		{
			//ctrlParasPtr->gear = 8;
		}
		
		rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;
		
		if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_4)
		{
			//ctrlParasPtr->gear = 8;
		}
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
		
		lmSpeed = 0;
		rmSpeed = 0;
		
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 633;
			
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
			}
			
			if(centCount > 7000)
			{
				ctrlParasPtr->comflag = 6331;
				
				if(10 != ctrlParasPtr->gear)
				{
					ctrlParasPtr->gear = 10;
				}
				
				startCount = 0;
				
			}
			
		}
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}
	
	//Get_T1_Duty3(&lmSpeedT1, &rmSpeedT1, adaptInfo);
	
	Pattern_ctrl4(&lmSpeedPat, &rmSpeedPat);

	//Software_PWM(&softwarePWML, &softwarePWMR);

	//get_Pull_Duty(&lmSpeedPull, &rmSpeedPull, adaptInfo);
	
	//T1_Adapter3(&lmSpeedT1, &rmSpeedT1, adaptInfo2);
	
	//T1_Adapter4(&lmSpeedT1, &rmSpeedT1, adaptInfo);
	
	//Get_Damp_Duty(&lmSpeedPull, &rmSpeedPull);
	

	if((AgvRight2Cent == FMSDS_Ptr->agvDirection) || (AgvLeft2Cent == FMSDS_Ptr->agvDirection))
	{
		lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - lmSpeedPat;
		
		rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - rmSpeedPat;
	}
	else if((AgvCent2Left == FMSDS_Ptr->agvDirection) || (AgvCent2Right == FMSDS_Ptr->agvDirection))
	{
		lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - lmSpeed;
		
		rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - rmSpeed;
	}
	
	
	//damping_func(1000, gearRecod, rmSpeedSet, lmSpeedSet);
	set_duty2(lmSpeedSet, rmSpeedSet);
	
}


void Get_Pat_P_T1_Duty(void)
{
	
}


void so_This_is_P(u8 *lmSpeedPat_PP, u8 *rmSpeedPat_PP)
{	
	//u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16};
	//u8 AgvPatAngOut[MAX_OUT] = {1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10};
	//u8 AgvPatAngOut[21] = {1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
	u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 3, 3, 5, 5, 7, 7, 9, 9, 11, 11, 13, 13, 15, 15, 15, 15};
	u8 lmSpeedPat_P = 0, rmSpeedPat_P = 0, tempAngle = 0, lmSpeedPat_T1 = 0, rmSpeedPat_T1 = 0;
	u8 maxLimt = MAX_OUT - 1;

	// 1.判别车体的运动方向, 区分控制逻辑(正数:车体往远离磁条方向; 负数:车体靠近磁条方向)
	if(AGV_Pat_Ptr->MidpointDirection >= 0)		// 1.2 车体中点为远离磁条方向
	{
		// 2.判别车体在左边/右边,然后区分控制逻辑
		if(AGV_Pat_Ptr->Midpoint > 0)	// 2.1 车体中点在磁条右边, 远离磁条方向
		{
			
			if(AGV_Pat_Ptr->Angle >= 0)		// 中点在磁条右边并且方向远离磁条, 角度是正数/与磁条平行
			{
				if(AGV_Pat_Ptr->Angle < MAX_OUT)		// 角度偏差极限值
				{
					lmSpeedPat_P = AgvPatAngOut[AGV_Pat_Ptr->Angle];
				}
				else
				{
					lmSpeedPat_P = AgvPatAngOut[maxLimt];
				}
				
			}
			else	// 中点方向继续远离磁条, 但是角度已经为负数, 除了"甩尾"我真的想不出有什么词可以形容此时的情况了
			{
				if((AGV_Pat_Ptr->Angle < 0) && (AGV_Pat_Ptr->Angle > -MAX_OUT))
				{
					tempAngle = -AGV_Pat_Ptr->Angle;

					if((tempAngle > 0) && (tempAngle < MAX_OUT))
					{
						lmSpeedPat_P = AgvPatAngOut[tempAngle];
					}
					else
					{
						printf("error code 5: %d, %d\r\n", tempAngle, AGV_Pat_Ptr->Angle);
					}
				}
				else
				{
					lmSpeedPat_P = AgvPatAngOut[maxLimt];
				}
				
			}
			
		}
		else if(0 == AGV_Pat_Ptr->Midpoint)		// 2.2 车体中点在磁条上, 远离磁条方向
		{
			if(AGV_Pat_Ptr->Angle > 0)			// 车体中点在磁条上, 并且方向为远离磁条方向, 角度为正数
			{
				if(AGV_Pat_Ptr->Angle < MAX_OUT)		// 角度偏差极限值
				{
					lmSpeedPat_P = AgvPatAngOut[AGV_Pat_Ptr->Angle];
				}
				else
				{
					lmSpeedPat_P = AgvPatAngOut[maxLimt];
				}
			}
			else if(0 == AGV_Pat_Ptr->Angle)	// 车体中点在磁条上, 并且方向为远离磁条方向, 与磁条平行
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)		// 车体中点在磁条上, 并且方向为远离磁条方向, 角度为负数
			{
				
				if(AGV_Pat_Ptr->Angle > -MAX_OUT)		// 角度偏差极限值
				{
					tempAngle = -AGV_Pat_Ptr->Angle;
					
					if((tempAngle > 0) && (tempAngle < MAX_OUT))
					{
						rmSpeedPat_P = AgvPatAngOut[tempAngle];
					}
					else
					{
						printf("error code 3: %d, %d\r\n", tempAngle, AGV_Pat_Ptr->Angle);
					}
					
				}
				else
				{
					rmSpeedPat_P = AgvPatAngOut[maxLimt];
				}
			}
			
		}
		else if(AGV_Pat_Ptr->Midpoint < 0)		// 2.3车体中点在磁条右边, 远离磁条方向
		{
			if(AGV_Pat_Ptr->Angle <= 0)
			{
				
				if(AGV_Pat_Ptr->Angle > -MAX_OUT)		// 角度偏差极限值
				{
					tempAngle = -AGV_Pat_Ptr->Angle;
					
					if((tempAngle >= 0) && (tempAngle < MAX_OUT))
					{
						rmSpeedPat_P = AgvPatAngOut[tempAngle];
					}
					else
					{
						printf("error code 4: %d, %d\r\n", tempAngle, AGV_Pat_Ptr->Angle);
					}	
					
				}
				else
				{
					rmSpeedPat_P = AgvPatAngOut[maxLimt];
				}
			}
			else		// 甩尾
			{
				if((AGV_Pat_Ptr->Angle > 0) && (AGV_Pat_Ptr->Angle < MAX_OUT))
				{
					rmSpeedPat_P = AgvPatAngOut[AGV_Pat_Ptr->Angle];
				}
				else
				{
					rmSpeedPat_P = AgvPatAngOut[maxLimt];
				}
				
			}
			
		}
		
	}

	//Get_T1_Duty4(&lmSpeedPat_T1, &rmSpeedPat_T1, adaptInfo);

	ctrlParasPtr->LP_duty = lmSpeedPat_P;
	ctrlParasPtr->RP_duty = rmSpeedPat_P;
	
	*lmSpeedPat_PP = lmSpeedPat_P;
	*rmSpeedPat_PP = rmSpeedPat_P;
}


void so_This_is_D(u8 *lmSpeedPat_DP, u8 *rmSpeedPat_DP)
{
	u8 lmSpeedPat_D = 0, rmSpeedPat_D = 0, tempAngle = 0;
	
	if(AGV_Pat_Ptr->MidpointDirection < 0)	// 1.2 车体中点方向是往磁条靠拢
	{
		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(AGV_Pat_Ptr->Angle < 0)		
			{
				tempAngle = -AGV_Pat_Ptr->Angle;
				
				if(AGV_Pat_Ptr->Angle >= -1)
				{
					
				}
				else if((AGV_Pat_Ptr->Angle < -1) && (AGV_Pat_Ptr->Angle > -10))
				{
					if((-9 == AGV_Pat_Ptr->Angle) || (-8 == AGV_Pat_Ptr->Angle))
					{
						rmSpeedPat_D = 4;
					}
					else if((-7 == AGV_Pat_Ptr->Angle) || (-6 == AGV_Pat_Ptr->Angle))
					{
						rmSpeedPat_D = 3;
					}
					else if((-5 == AGV_Pat_Ptr->Angle) || (-4 == AGV_Pat_Ptr->Angle))
					{
						rmSpeedPat_D = 2;
					}
					else if((-3 == AGV_Pat_Ptr->Angle) || (-2 == AGV_Pat_Ptr->Angle))
					{
						rmSpeedPat_D = 1;
					}
					
				}
				else
				{
					rmSpeedPat_D = 5;
				}
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				lmSpeedPat_D = 1;
			}
			else if(AGV_Pat_Ptr->Angle > 0)		// 甩尾
			{
				lmSpeedPat_D = 2;
				printf("MpDir < 0 && Mp > 0 && Angle < 0\r\n");
			}
			
		}
		else if(0 == AGV_Pat_Ptr->Midpoint)
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				if(AGV_Pat_Ptr->Angle <= 1)
				{
					
				}
				else if((AGV_Pat_Ptr->Angle > 1) && (AGV_Pat_Ptr->Angle < 10))
				{
					if((9 == AGV_Pat_Ptr->Angle) || (8 == AGV_Pat_Ptr->Angle))
					{
						lmSpeedPat_D = 4;
					}
					else if((7 == AGV_Pat_Ptr->Angle) || (6 == AGV_Pat_Ptr->Angle))
					{
						lmSpeedPat_D = 3;
					}
					else if((5 == AGV_Pat_Ptr->Angle) || (4 == AGV_Pat_Ptr->Angle))
					{
						lmSpeedPat_D = 2;
					}
					else if((3 == AGV_Pat_Ptr->Angle) || (2 == AGV_Pat_Ptr->Angle))
					{
						lmSpeedPat_D = 1;
					}
					
				}
				else
				{
					lmSpeedPat_D = 5;
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				if(AGV_Pat_Ptr->Angle >= -1)
				{
					
				}
				else if((AGV_Pat_Ptr->Angle < -1) && (AGV_Pat_Ptr->Angle > -10))
				{
					if((-9 == AGV_Pat_Ptr->Angle) || (-8 == AGV_Pat_Ptr->Angle))
					{
						rmSpeedPat_D = 4;
					}
					else if((-7 == AGV_Pat_Ptr->Angle) || (-6 == AGV_Pat_Ptr->Angle))
					{
						rmSpeedPat_D = 3;
					}
					else if((-5 == AGV_Pat_Ptr->Angle) || (-4 == AGV_Pat_Ptr->Angle))
					{
						rmSpeedPat_D = 2;
					}
					else if((-3 == AGV_Pat_Ptr->Angle) || (-2 == AGV_Pat_Ptr->Angle))
					{
						rmSpeedPat_D = 1;
					}
					
				}
				else
				{
					rmSpeedPat_D = 5;
				}
				
			}
		}
		else if(AGV_Pat_Ptr->Midpoint < 0)
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				if(AGV_Pat_Ptr->Angle <= 1)
				{
					
				}
				else if((AGV_Pat_Ptr->Angle > 1) && (AGV_Pat_Ptr->Angle < 10))
				{
					if((9 == AGV_Pat_Ptr->Angle) || (8 == AGV_Pat_Ptr->Angle))
					{
						lmSpeedPat_D = 4;
					}
					else if((7 == AGV_Pat_Ptr->Angle) || (6 == AGV_Pat_Ptr->Angle))
					{
						lmSpeedPat_D = 3;
					}
					else if((5 == AGV_Pat_Ptr->Angle) || (4 == AGV_Pat_Ptr->Angle))
					{
						lmSpeedPat_D = 2;
					}
					else if((3 == AGV_Pat_Ptr->Angle) || (2 == AGV_Pat_Ptr->Angle))
					{
						lmSpeedPat_D = 1;
					}
					
				}
				else
				{
					lmSpeedPat_D = 5;
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeedPat_D = 1;
			}
			else if(AGV_Pat_Ptr->Angle < 0)		// 甩尾
			{
				rmSpeedPat_D = 2;
				printf("MpDir < 0 && Mp < 0 && Angle < 0\r\n");
			}
		}
		
	}

	*lmSpeedPat_DP = lmSpeedPat_D;
	*rmSpeedPat_DP = rmSpeedPat_D;
}


void so_This_is_D2(u8 *lmSpeedPat_DP, u8 *rmSpeedPat_DP)
{
	u8 lmSpeedPat_D = 0, rmSpeedPat_D = 0, tempAngle = 0;
	//u8 AgvPatAngIn[MAX_IN] = {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16};
	//u8 AgvPatAngIn[MAX_IN] = {2, 3, 3, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 18, 18, 18, 18};
	u8 AgvPatAngIn[MAX_IN] = {1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 12, 12, 14, 14, 16, 16, 16, 16};
	u8 limt1 = 0;

	if(0 == AGV_Pat_Ptr->Midpoint)
	{
		limt1 = 1;
	}
	else
	{
		limt1 = 2;
	}
	
	if(AGV_Pat_Ptr->MidpointDirection < 0)	// 1.2 车体中点方向是往磁条靠拢
	{
		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(AGV_Pat_Ptr->Angle < 0)		
			{
				
				if((AGV_Pat_Ptr->Angle < -limt1) && (AGV_Pat_Ptr->Angle > -MAX_IN))
				{
					tempAngle = -AGV_Pat_Ptr->Angle;
					
					if((tempAngle > limt1) && (tempAngle < MAX_IN))
					{
						rmSpeedPat_D = AgvPatAngIn[tempAngle];
					}
					else
					{
						printf("error code 1: %d\r\n", tempAngle);
					}
					
				}
				else if(AGV_Pat_Ptr->Angle <= -MAX_IN)
				{
					rmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeedPat_D = 1;
			}
			else if(AGV_Pat_Ptr->Angle > 0)		// 甩尾
			{
				if(AGV_Pat_Ptr->Angle < MAX_IN)
				{
					lmSpeedPat_D = AgvPatAngIn[AGV_Pat_Ptr->Angle];
				}
				else
				{
					lmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
				
				//printf("MpDir < 0 && Mp > 0 && Angle < 0\r\n");
			}
			
		}

		
		else if(0 == AGV_Pat_Ptr->Midpoint)
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				if((AGV_Pat_Ptr->Angle > limt1) && (AGV_Pat_Ptr->Angle < MAX_IN))
				{
					lmSpeedPat_D = AgvPatAngIn[AGV_Pat_Ptr->Angle];
				}
				else if(AGV_Pat_Ptr->Angle >= MAX_IN)
				{
					lmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				
				if((AGV_Pat_Ptr->Angle < -limt1) && (AGV_Pat_Ptr->Angle > -MAX_IN))
				{
					tempAngle = -AGV_Pat_Ptr->Angle;
					
					if((tempAngle > limt1) && (tempAngle < MAX_IN))
					{
						rmSpeedPat_D = AgvPatAngIn[tempAngle];
					}
					else
					{
						printf("error code 2: %d\r\n", tempAngle);
					}
					
				}
				else if(AGV_Pat_Ptr->Angle <= -MAX_IN)
				{
					rmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
			}
		}

		
		else if(AGV_Pat_Ptr->Midpoint < 0)
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				if((AGV_Pat_Ptr->Angle > limt1) && (AGV_Pat_Ptr->Angle < MAX_IN))
				{
					lmSpeedPat_D = AgvPatAngIn[AGV_Pat_Ptr->Angle];
				}
				else if(AGV_Pat_Ptr->Angle >= MAX_IN)
				{
					lmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				lmSpeedPat_D = 1;
			}
			else if(AGV_Pat_Ptr->Angle < 0)		// 甩尾
			{
				tempAngle = -AGV_Pat_Ptr->Angle;
				
				if((tempAngle > 0) && (tempAngle < MAX_IN))
				{
					rmSpeedPat_D = AgvPatAngIn[tempAngle];
				}
				else
				{
					rmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
					printf("tempAngle = %d\r\n", tempAngle);
				}
				
				//printf("MpDir < 0 && Mp < 0 && Angle < 0\r\n");
			}
		}
		
	}

	
	ctrlParasPtr->LD_duty = lmSpeedPat_D;
	ctrlParasPtr->RD_duty = rmSpeedPat_D;

	*lmSpeedPat_DP = lmSpeedPat_D;
	*rmSpeedPat_DP = rmSpeedPat_D;
}

void so_This_is_D3(u8 *lmSpeedPat_DP, u8 *rmSpeedPat_DP)
{
	u8 lmSpeedPat_D = 0, rmSpeedPat_D = 0, tempAngle = 0;
	u8 AgvPatAngIn[MAX_IN] = {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16};
	//u8 AgvPatAngIn[MAX_IN] = {2, 3, 3, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 18, 18, 18, 18};
	/*
	u8 AgvPatAngIn[5][MAX_IN] = {{1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16},\
								 {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16},\
								 {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16},\
								 {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16},\
								 {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16}};*/
	u8 limt1 = 0;

	if(0 == AGV_Pat_Ptr->Midpoint)
	{
		limt1 = 1;
	}
	else
	{
		limt1 = 2;
	}
	
	if(AGV_Pat_Ptr->MidpointDirection < 0)	// 1.2 车体中点方向是往磁条靠拢
	{
		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(AGV_Pat_Ptr->Angle < 0)		
			{
				
				if((AGV_Pat_Ptr->Angle < -limt1) && (AGV_Pat_Ptr->Angle > -MAX_IN))
				{
					tempAngle = -AGV_Pat_Ptr->Angle;
					
					if((tempAngle > limt1) && (tempAngle < MAX_IN))
					{
						rmSpeedPat_D = AgvPatAngIn[tempAngle];
					}
					else
					{
						printf("error code 1: %d\r\n", tempAngle);
					}
					
				}
				else if(AGV_Pat_Ptr->Angle <= -MAX_IN)
				{
					rmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				lmSpeedPat_D = 1;
			}
			else if(AGV_Pat_Ptr->Angle > 0)		// 甩尾
			{
				if(AGV_Pat_Ptr->Angle < MAX_IN)
				{
					lmSpeedPat_D = AgvPatAngIn[AGV_Pat_Ptr->Angle];
				}
				else
				{
					lmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
				
				//printf("MpDir < 0 && Mp > 0 && Angle < 0\r\n");
			}
			
		}

		
		else if(0 == AGV_Pat_Ptr->Midpoint)
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				if((AGV_Pat_Ptr->Angle > limt1) && (AGV_Pat_Ptr->Angle < MAX_IN))
				{
					lmSpeedPat_D = AgvPatAngIn[AGV_Pat_Ptr->Angle];
				}
				else if(AGV_Pat_Ptr->Angle >= MAX_IN)
				{
					lmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				
				if((AGV_Pat_Ptr->Angle < -limt1) && (AGV_Pat_Ptr->Angle > -MAX_IN))
				{
					tempAngle = -AGV_Pat_Ptr->Angle;
					
					if((tempAngle > limt1) && (tempAngle < MAX_IN))
					{
						rmSpeedPat_D = AgvPatAngIn[tempAngle];
					}
					else
					{
						printf("error code 2: %d\r\n", tempAngle);
					}
					
				}
				else if(AGV_Pat_Ptr->Angle <= -MAX_IN)
				{
					rmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
			}
		}

		
		else if(AGV_Pat_Ptr->Midpoint < 0)
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				if((AGV_Pat_Ptr->Angle > limt1) && (AGV_Pat_Ptr->Angle < MAX_IN))
				{
					lmSpeedPat_D = AgvPatAngIn[AGV_Pat_Ptr->Angle];
				}
				else if(AGV_Pat_Ptr->Angle >= MAX_IN)
				{
					lmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeedPat_D = 1;
			}
			else if(AGV_Pat_Ptr->Angle < 0)		// 甩尾
			{
				tempAngle = -AGV_Pat_Ptr->Angle;
				
				if((tempAngle > 0) && (tempAngle < MAX_IN))
				{
					rmSpeedPat_D = AgvPatAngIn[tempAngle];
				}
				else
				{
					rmSpeedPat_D = AgvPatAngIn[MAX_IN - 1];
					printf("tempAngle = %d\r\n", tempAngle);
				}
				
				//printf("MpDir < 0 && Mp < 0 && Angle < 0\r\n");
			}
		}
		
	}


	

	*lmSpeedPat_DP = lmSpeedPat_D;
	*rmSpeedPat_DP = rmSpeedPat_D;
}



void Pat_ShowAs_Symble(void)
{
	if(AGV_Pat_Ptr->Midpoint > 0)
	{
		if(AGV_Pat_Ptr->MidpointDirection > 0)
		{
			printf("*** |-->\t");
		}
		else if(AGV_Pat_Ptr->MidpointDirection < 0)
		{
			printf("*** |<--\t");
		}
		
	}
	else if(AGV_Pat_Ptr->Midpoint < 0)
	{
		if(AGV_Pat_Ptr->MidpointDirection > 0)
		{
			printf("*** <--|\t");
		}
		else if(AGV_Pat_Ptr->MidpointDirection < 0)
		{
			printf("*** -->|\t");
		}
	}
	else if(0 == AGV_Pat_Ptr->Midpoint)
	{
		if(AGV_Pat_Pre_Ptr->Midpoint > 0)
		{
			printf("*** |<--\t|");
		}
		else if(AGV_Pat_Pre_Ptr->Midpoint < 0)
		{
			printf("*** -->|\t|");
		}
	}

	if(AGV_Pat_Ptr->Angle > 0)
	{
		printf("/ ***\r\n");
	}
	else if(0 == AGV_Pat_Ptr->Angle)
	{
		printf("| ***\r\n");
	}
	else if(AGV_Pat_Ptr->Angle < 0)
	{
		printf("\\ ***\r\n");
	}
}

void scale_1_mode19(u8 gear)
{
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeedPat_P = 0, rmSpeedPat_P = 0, lmSpeedPat_D = 0, rmSpeedPat_D = 0;
	static Agv_MS_Location msfRec = AgvInits, msrRec = AgvInits;
		
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;

	so_This_is_P(&lmSpeedPat_P, &rmSpeedPat_P);
	
	so_This_is_D2(&lmSpeedPat_D, &rmSpeedPat_D);

	if((msfRec != FMSDS_Ptr->AgvMSLocation) || (msrRec != RMSDS_Ptr->AgvMSLocation))
	{
		
		msfRec = FMSDS_Ptr->AgvMSLocation;
		
		msrRec = RMSDS_Ptr->AgvMSLocation;
		
		printf("*** P: lm = %d, rm = %d ***\r\n", lmSpeedPat_P, rmSpeedPat_P);
		printf("*** D: lm = %d, rm = %d ***\r\n", lmSpeedPat_D, rmSpeedPat_D);
		
		Pat_ShowAs_Symble();
		
	}

	if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedPat_P - lmSpeedPat_D;
		
		rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedPat_P - rmSpeedPat_D;

	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - lmSpeedPat_P - lmSpeedPat_D;
		
		rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - rmSpeedPat_P - rmSpeedPat_D;

	}

	set_duty_Com(lmSpeedSet, rmSpeedSet);
}


void scale_1_mode20(u8 gear)
{
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeedPat_P = 0, rmSpeedPat_P = 0, lmSpeedPat_D = 0, rmSpeedPat_D = 0;
	static Agv_MS_Location msfRec = AgvInits, msrRec = AgvInits;
		
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;

	so_This_is_P(&lmSpeedPat_P, &rmSpeedPat_P);
	
	so_This_is_D2(&lmSpeedPat_D, &rmSpeedPat_D);

	if((msfRec != FMSDS_Ptr->AgvMSLocation) || (msrRec != RMSDS_Ptr->AgvMSLocation))
	{
		
		msfRec = FMSDS_Ptr->AgvMSLocation;
		
		msrRec = RMSDS_Ptr->AgvMSLocation;
		
		//printf("*** P: lm = %d, rm = %d ***\r\n", lmSpeedPat_P, rmSpeedPat_P);
		//printf("*** D: lm = %d, rm = %d ***\r\n", lmSpeedPat_D, rmSpeedPat_D);
		
		//Pat_ShowAs_Symble();
		
	}

	if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		lmSpeedSet = AgvGear[gearRecod] - lmSpeedPat_P - lmSpeedPat_D;
		
		rmSpeedSet = AgvGear[gearRecod] - rmSpeedPat_P - rmSpeedPat_D;

	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		lmSpeedSet = AgvGear[gearRecod] - lmSpeedPat_P - lmSpeedPat_D;
		
		rmSpeedSet = AgvGear[gearRecod] - rmSpeedPat_P - rmSpeedPat_D;

	}

	set_duty_Com(lmSpeedSet, rmSpeedSet);
}



void AGV_Correct_gS_8ug(u8 gear)		// 3 mode
{
	static u8 gearRecod = 0;
	
	ctrlParasPtr->comflag = 6;
		
	gearRecod = gear;
	
	//if(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3)
	if(1)
	{
		
		if(0 == ctrlParasPtr->FSflag)		
		{
			// 启动模式
			gS_startup_mode2(3);
			
		}
		else if(1 == ctrlParasPtr->FSflag)
		{
			// 偏差达到1格模式
			//scale_1_mode17(gearRecod);
			scale_1_mode20(gearRecod);
			
		}
		else if(2 == ctrlParasPtr->FSflag)
		{
			// 紧急模式
			gS_urgency_mode();
			
		}
		
	}
	ctrlParasPtr->BSflag = 0;
}

void AGV_Correct_back_ug(u8 gear)		// 3 mode
{
	static u8 gearRecod = 0;
	
	ctrlParasPtr->comflag = 6;

	
	gearRecod = gear;
	
	//printf("************\r\n");

	
	//if(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3)
	if(1)
	{

		if(0 == ctrlParasPtr->BSflag)		
		{
			// 启动模式
			bS_startup_mode2(3);
			
		}
		else if(1 == ctrlParasPtr->BSflag)
		{
			// 偏差达到1格模式
			//scale_1_mode18_back(gearRecod);
			scale_1_mode20(gearRecod);
		}
		else if(2 == ctrlParasPtr->BSflag)
		{
			// 紧急模式
			gS_urgency_mode();
			
		}
		else if(3 == ctrlParasPtr->BSflag)
		{
			back_slow(5);
			
		}
		
		ctrlParasPtr->FSflag = 0;
	}
	
}


void gS_step_gB(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, lreco = 0, rreco = 0;
	static u32 startCount = 0;
	u32 centCount = 0;
	u8 gearRecod = 0;
	
	
	if(0 == ctrlParasPtr->FSflag)
	{
		gearRecod = gear;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			
			ctrlParasPtr->comflag = 61;

			if(FMSDS_Ptr->AgvMSLocation < FMSDS_Pre_Ptr->AgvMSLocation)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
				printf("MaxRecoder = %d, calu = %d\r\n", FMSDS_Ptr->MaxRecoder, LocValu(FMSDS_Ptr->MaxRecoder));
			}
			

			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				/////// 偏差小的情况下
				
				ctrlParasPtr->comflag = 611;
				
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					//if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Center)
					if(FMSDS_Pre_Ptr->AgvMSLocation - FMSDS_Ptr->AgvMSLocation > 0)
					{
						lreco++;
					}
					
				}
				
				//printf("lreco = %d\r\n", lreco);

				//lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][gearRecod];
				//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
			}
	#if 1
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3))
			{
				///// 偏差大的情况下
				
				ctrlParasPtr->BSflag = 1;
				ctrlParasPtr->comflag = 612;
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				//lmSpeed = AgvGear[2] + FLG[0][2] + FLG[2][gearRecod];
				lmSpeed = AgvGear[3] + AgvGearCompDutyLB[3] + FLG[3][3];
				rmSpeed = AgvGear[3] + AgvGearCompDutyRB[3];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
				
				lreco = rreco + 3;
			}
	#endif
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 62;

			if(FMSDS_Ptr->AgvMSLocation > FMSDS_Pre_Ptr->AgvMSLocation)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
				printf("MaxRecoder = %d, calu = %d\r\n", FMSDS_Ptr->MaxRecoder, LocValu(FMSDS_Ptr->MaxRecoder));
			}

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_3))
			{

				ctrlParasPtr->comflag = 621;
				
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					ctrlParasPtr->comflag = 6211;
					//if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
					if(FMSDS_Ptr->AgvMSLocation - FMSDS_Pre_Ptr->AgvMSLocation > 0)
					{
						ctrlParasPtr->comflag = 62111;
						rreco++;
					}
					
				}
				
				
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
				
			}
	#if 1
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->FSflag = 1;

				ctrlParasPtr->comflag = 622;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				lmSpeed = AgvGear[2] + AgvGearCompDutyLB[2];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[2] + AgvGearCompDutyRB[2] + FRG[2][2];
				
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);


				
				rreco = lreco + 3;
			}
	#endif
			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 63;

			// 加入阻尼模块
			// 阻尼begin
	#if 1
			if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 631;
				ctrlParasPtr->dampingFlag = DampingLeft;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;
				
				//LocValu(FMSDS_Ptr->MaxRecoder);
				
				//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - DutyTableLow[LocValu(FMSDS_Ptr->MaxRecoder)];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] + DutyTableLow[LocValu(FMSDS_Ptr->MaxRecoder)];
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				MOTOR_LEFT_DUTY_SET_F(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
				//printf("lmSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
			}
			else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 632;
				ctrlParasPtr->dampingFlag = DampingRight;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;

				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] + DutyTableLow[LocValu(FMSDS_Ptr->MaxRecoder)];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - DutyTableLow[LocValu(FMSDS_Ptr->MaxRecoder)];
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				MOTOR_LEFT_DUTY_SET_F(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				//printf("rmSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
			}
			
			// 阻尼end
			//else if(AgvNone == FMSDS_Ptr->agvDirection)
			else
			{
				ctrlParasPtr->comflag = 633;
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
			}
	#endif

			
			
			FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
			
			
		}
		
	}
	else
	{
		gearRecod = 2;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 64;

			//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 65;

			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
			//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);

			startCount = 0;
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 66;
			
			if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				if(0 == startCount)
				{
					ctrlParasPtr->comflag = 661;
					startCount = SystemRunningTime;
				}
				else
				{
					ctrlParasPtr->comflag = 662;
					centCount = SystemRunningTime - startCount;
				}
				
				if(centCount > 5000)
				{
					ctrlParasPtr->FSflag = 0;
					ctrlParasPtr->comflag = 663;
					FMSDS_Ptr->agvDirection = AgvNone;
					gearRecod = gear;

					//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
					//rmSpeed = AgvGear[gearRecod] + FRG[0][gearRecod];
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];
		
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);

					startCount = 0;
				}
			}
			else
			{
				startCount = 0;
			#if 0
				ctrlParasPtr->comflag = 662;
				if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
				{
					ctrlParasPtr->comflag = 6621;
					ctrlParasPtr->dampingFlag = DampingLeft;
					ctrlParasPtr->dampingTimeRec = SystemRunningTime;

					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];

					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					
					MOTOR_LEFT_DUTY_SET(lmSpeed);
				}
				else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
				{
					ctrlParasPtr->comflag = 6622;
					ctrlParasPtr->dampingFlag = DampingRight;
					ctrlParasPtr->dampingTimeRec = SystemRunningTime;
					
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
					
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
					
					MOTOR_RIGHT_DUTY_SET(rmSpeed);
				}
				// 阻尼end
			#endif
			}

		}
	}
	
#if 0
	if(lreco > rreco)
	{
		
		if(lreco - rreco >= 3)
		{
			
			if(FRightCompDuty[AgvGear[gearRecod]] > 0)
			{
				
				FRightCompDuty[AgvGear[gearRecod]]--;
			}
			else
			{
				
				FLeftCompDuty[AgvGear[gearRecod]]++;
			}
			
			printf("loffset = %d, roffset = %d\r\n", FLeftCompDuty[AgvGear[gearRecod]], FRightCompDuty[AgvGear[gearRecod]]);
			
			
			lreco = 0;
			rreco = 0;
		}
	}
	else if(lreco < rreco)
	{
		
		if(rreco - lreco >= 3)
		{
			
			if(FLeftCompDuty[AgvGear[gearRecod]] > 0)
			{
				
				FLeftCompDuty[AgvGear[gearRecod]]--;
			}
			else
			{
				
				FRightCompDuty[AgvGear[gearRecod]]++;
			}
			
			printf("loffset = %d, roffset = %d\r\n", FLeftCompDuty[AgvGear[gearRecod]], FRightCompDuty[AgvGear[gearRecod]]);
			
			lreco = 0;
			rreco = 0;
		}
	}
#endif
	
}

void gS_step_exit(u8 gearRecod)
{
	static u8 lmSpeed = 0, rmSpeed = 0;
	//u32 centCount = 0;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 64;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];
		
		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

		MOTOR_LEFT_DUTY_SET(rmSpeed);
		MOTOR_RIGHT_DUTY_SET(lmSpeed);
		
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 65;

		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

		MOTOR_LEFT_DUTY_SET(rmSpeed);
		MOTOR_RIGHT_DUTY_SET(lmSpeed);

	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 66;

		
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
		}
		else
		{
			#if 0
			ctrlParasPtr->comflag = 662;
			if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 6621;
				ctrlParasPtr->dampingFlag = DampingLeft;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;

				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
			}
			else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 6622;
				ctrlParasPtr->dampingFlag = DampingRight;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;
				
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
			// 阻尼end
			#endif
		}

	}
	
	#if 0
	if(lreco > rreco)
	{
		
		if(lreco - rreco >= 3)
		{
			
			if(FRightCompDuty[AgvGear[gearRecod]] > 0)
			{
				
				FRightCompDuty[AgvGear[gearRecod]]--;
			}
			else
			{
				
				FLeftCompDuty[AgvGear[gearRecod]]++;
			}
			
			printf("loffset = %d, roffset = %d\r\n", FLeftCompDuty[AgvGear[gearRecod]], FRightCompDuty[AgvGear[gearRecod]]);
			
			
			lreco = 0;
			rreco = 0;
		}
	}
	else if(lreco < rreco)
	{
		
		if(rreco - lreco >= 3)
		{
			
			if(FLeftCompDuty[AgvGear[gearRecod]] > 0)
			{
				
				FLeftCompDuty[AgvGear[gearRecod]]--;
			}
			else
			{
				
				FRightCompDuty[AgvGear[gearRecod]]++;
			}
			
			printf("loffset = %d, roffset = %d\r\n", FLeftCompDuty[AgvGear[gearRecod]], FRightCompDuty[AgvGear[gearRecod]]);
			
			lreco = 0;
			rreco = 0;
		}
	}
	#endif
	
}


void AVG_Calu_Program(void)
{
	static u32 HLavg = 0, HRavg = 0, TLnow = 0, TRnow = 0;
	
	//TLpre = TLnow;
	TLnow = ctrlParasPtr->leftHallIntervalTime;
	HLavg = (ctrlParasPtr->HLavg + TLnow) / 2;
	if(HLavg > 100)
	{
		ctrlParasPtr->HLavg = HLavg;
	}
	else
	{
		//printf("HLavg = %d, TLnow = %d\r\n", ctrlParasPtr->HLavg, TLnow);
	}

	//TRpre = TRnow;
	TRnow = ctrlParasPtr->rightHallIntervalTime;
	HRavg = (ctrlParasPtr->HRavg + TRnow) / 2;
	if(HRavg > 100)
	{
		ctrlParasPtr->HRavg = HRavg;
	}
	else
	{
		//printf("HRavg = %d, TRnow = %d\r\n", ctrlParasPtr->HRavg, TRnow);
	}
	
	//printf("LHT = %d, HLavg = %d\r\n", ctrlParasPtr->leftHallIntervalTime, ctrlParasPtr->HLavg);
	//printf("RHT = %d, HRavg = %d\r\n\r\n", ctrlParasPtr->rightHallIntervalTime, ctrlParasPtr->HRavg);
	
	
}


void AGV_Correct_1(void)
{
	u32 result = 0;
	u8 range = 0, lmSpeed = 0, rmSpeed = 0;
	static u8 duty = 0;
	static u8 lcompduty = 0, rcompduty = 0;
	static u8 gear = 10;

	duty = AgvGear[gear];
	// 延时一秒, 重新检测速度
	if(ctrlParasPtr->avgFlag)
	{
	#if 0
		
		result = ctrlParasPtr->leftHallIntervalTime - ctrlParasPtr->rightHallIntervalTime;

		// 目前设定的范围值为 ±5, 如果左右两边时间在这个范围内, 则认定两边速度是相等的
		if(result > range)			// 左边速度比右变速度大的时候
		{
			if(lcompduty > 0)
			{
				lcompduty--;
			}
			else
			{
				rcompduty++;
			}
			
		}
		else if(result < -range)	// 左边的速度比右边速度小的时候
		{
			if(rcompduty > 0)
			{
				rcompduty--;
			}
			else
			{
				lcompduty++;
			}
			
		}
		else					//	在误差范围内, 则打印数据, 然后换挡		
		{
			
			
			printf("d = %d, lcd = %d, rcd = %d\r\n", duty, lcompduty, rcompduty);
			
			lcompduty = 0;
			rcompduty = 0;
			
			duty++;
			
		}

		lmSpeed = duty + lcompduty;
		rmSpeed = duty + rcompduty;
		
		if((lmSpeed > 100) || (rmSpeed > 100))
		{
			CHANGE_TO_STOP_MODE();
			MOTOR_LEFT_DUTY_SET(0);
			MOTOR_RIGHT_DUTY_SET(0);
			printf("ok!!!!!!!!\r\n");
		}
		else
		{
			MOTOR_LEFT_DUTY_SET(duty + lcompduty);
			MOTOR_RIGHT_DUTY_SET(duty + rcompduty);
		}
		
	#else
		
		
	
		if(duty <= 25)
		{
			range = 10;
		}
		else if((duty > 25) && (range <= 35))
		{
			range = 5;
		}
		else
		{
			range = 3;
		}
		//result = ctrlParasPtr->leftHallIntervalTime - ctrlParasPtr->rightHallIntervalTime;
		result = SubAbsV(ctrlParasPtr->HLavg, ctrlParasPtr->HRavg);
		
		printf("res = %d, HLavg = %d, HRavg = %d\r\n", result, ctrlParasPtr->HLavg, ctrlParasPtr->HRavg);

		// 目前设定的范围值为 ±5, 如果左右两边时间在这个范围内, 则认定两边速度是相等的
		if(result > range)			// 如果两个时间大于偏差范围
		{
			if(ctrlParasPtr->HLavg > ctrlParasPtr->HRavg)	// 如果是左边时间大于右边时间, 则左边转速比右边慢, 则需要左边加速/右边减速
			{
				if(rcompduty > 0)
				{
					rcompduty--;
				}
				else
				{
					lcompduty++;
				}
			}
			else											// 如果是左边时间小于右边时间, 否则是左边转速比右边快, 则需要左边减速/右边加速
			{
				if(lcompduty > 0)
				{
					lcompduty--;
				}
				else
				{
					rcompduty++;
				}
			}
			
		}
		else	// 如果速度在偏差范围内
		{
			//printf("d = %d, lcd = %d, rcd = %d\r\n", duty, lcompduty, rcompduty);
			printf("%d,%d,%d\r\n", duty, lcompduty, rcompduty);
			//lcompduty = 0;
			//rcompduty = 0;
			
			//duty--;
			//gear++;
			
		}

		duty = AgvGear[gear];
		
		lmSpeed = duty + lcompduty;
		rmSpeed = duty + rcompduty;
		
		printf("gear = %d\r\n", gear);
		printf("ls = %d, duty = %d, lcd = %d\r\n", lmSpeed, duty, lcompduty);
		printf("rs = %d, duty = %d, rcd = %d\r\n\r\n", rmSpeed, duty, rcompduty);
		

		
		if(gear >= 19)
		{
			//CHANGE_TO_STOP_MODE();
			//MOTOR_LEFT_DUTY_SET(0);
			//MOTOR_RIGHT_DUTY_SET(0);
			printf("ok!!!!!!!!\r\n");
		}
		else
		{
			MOTOR_LEFT_DUTY_SET(duty + lcompduty);
			MOTOR_RIGHT_DUTY_SET(duty + rcompduty);
		}

	#endif
		
		
		ctrlParasPtr->avgFlag = 0;
	}
	
	
	
}


void Get_Zigbee_Info_From_Buf(void)
{
	if((0x0000 == ctrlParasPtr->goalRFIDnode) && (zigbeeQueueCtrl.Total > 0))
	{
		get_zigbeeData(&(Zigbee_Ptr->recvId));

		if(0 == Zigbee_Ptr->recvId)
		{
			printf("recvId error!\r\n");
		}
		else
		{
			Zigbee_Ptr->recvValidDataFlag = 1;
			
		}
		
		zigbeeRecvDataBuf_Delete();
	}
}

void RFID_Goal_Node_Analy(void)
{
	if(1 == Zigbee_Ptr->recvValidDataFlag)
	{
		Zigbee_Ptr->recvValidDataFlag = 0;

		if((ZBandRFIDmapping[SpinStation_1] == Zigbee_Ptr->recvId) || (ZBandRFIDmapping[SpinStation_2] == Zigbee_Ptr->recvId))
		{
			ctrlParasPtr->goalRFIDnode = STATION_1AND2_RFID;
			//printf("%04x, %04x\r\n", ZBandRFIDmapping[SpinStation_1], Zigbee_Ptr->recvId);
			if(ZBandRFIDmapping[SpinStation_1] == Zigbee_Ptr->recvId)
			{
				ctrlParasPtr->goalStation = SpinStation_1;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_2;
			}
			ctrlParasPtr->walkingstep = step_gS;
		}
		else if((ZBandRFIDmapping[SpinStation_3] == Zigbee_Ptr->recvId) || (ZBandRFIDmapping[SpinStation_4] == Zigbee_Ptr->recvId))
		{
			ctrlParasPtr->goalRFIDnode = STATION_3AND4_RFID;
			if(ZBandRFIDmapping[SpinStation_3] == Zigbee_Ptr->recvId)
			{
				ctrlParasPtr->goalStation = SpinStation_3;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_4;
			}
			ctrlParasPtr->walkingstep = step_gS;
		}
		else if((ZBandRFIDmapping[SpinStation_5] == Zigbee_Ptr->recvId) || (ZBandRFIDmapping[SpinStation_6] == Zigbee_Ptr->recvId))
		{
			ctrlParasPtr->goalRFIDnode = STATION_5AND6_RFID;
			if(ZBandRFIDmapping[SpinStation_5] == Zigbee_Ptr->recvId)
			{
				ctrlParasPtr->goalStation = SpinStation_5;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_6;
			}
			ctrlParasPtr->walkingstep = step_gS;
		}
		else if((ZBandRFIDmapping[SpinStation_7] == Zigbee_Ptr->recvId) || (ZBandRFIDmapping[SpinStation_8] == Zigbee_Ptr->recvId))
		{
			ctrlParasPtr->goalRFIDnode = STATION_7AND8_RFID;
			if(ZBandRFIDmapping[SpinStation_7] == Zigbee_Ptr->recvId)
			{
				ctrlParasPtr->goalStation = SpinStation_7;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_8;
			}
			ctrlParasPtr->walkingstep = step_gS;
		}
		else if((ZBandRFIDmapping[SpinStation_9] == Zigbee_Ptr->recvId) || (ZBandRFIDmapping[SpinStation_10] == Zigbee_Ptr->recvId))
		{
			ctrlParasPtr->goalRFIDnode = STATION_9AND10_RFID;
			if(ZBandRFIDmapping[SpinStation_9] == Zigbee_Ptr->recvId)
			{
				ctrlParasPtr->goalStation = SpinStation_9;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_10;
			}
			ctrlParasPtr->walkingstep = step_gS;
		}
		else
		{
			ctrlParasPtr->goalRFIDnode = ZBandRFIDmapping[ControlCenter];
			ctrlParasPtr->goalStation = ControlCenter;
			ctrlParasPtr->walkingstep = step_stop;
		}


		printf("goalRFIDnode = %d, goalStation = %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->goalStation);
	}
	
}


void AGV_Walking(void)
{
	
	if(AutomaticMode == ctrlParasPtr->agvWalkingMode)
	{
		
		agv_walking_func[ctrlParasPtr->agvStatus](ctrlParasPtr->gear);
		
	}
	
}

void LeftOrRight_Counter(void)
{
	static u8 leftCounter = 0, rightCounter = 0;
	static Agv_MS_Location recoder = AgvInits;
	
	if(recoder != FMSDS_Ptr->AgvMSLocation)
	{
		recoder = FMSDS_Ptr->AgvMSLocation;

		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			leftCounter++;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{
			rightCounter++;
		}

		printf("LC = %d,\tRC = %d\r\n", leftCounter, rightCounter);
		
	}
	
}

void AGV_Change_Mode(void)
{
	//static u8 flag = 1;
	
	if((goStraightStatus == ctrlParasPtr->agvStatus) || (backStatus == ctrlParasPtr->agvStatus))
	{
#if 0
		if(1 == flag)
		{
			if(0xFFFF == FMSDS_Ptr->MSD_Hex)
			{
				flag = 0;
				FMSDS_Ptr->zflag = 1;
			}
		}
		else
		{
			if(FMSDS_Ptr->zeropointfive >= 500)
			{
				if(0x0000 == FMSDS_Ptr->MSD_Hex)
				{
					
					if(goStraightStatus == ctrlParasPtr->agvStatus)
					{
						backStatus_change();
						printf("backStatus_change\r\n");
					}
					else if(backStatus == ctrlParasPtr->agvStatus)
					{
						goStraight_change();
						printf("goStraight_change\r\n");
					}
				}

				flag = 1;
				
				FMSDS_Ptr->zflag = 0;
			}
			
		}
#else

		#if 0
		// 在单独直线磁条跑前后退
		if(0x0000 == FMSDS_Ptr->MSD_Hex)
		{
			
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
				backStatus_change();
				printf("backStatus_change\r\n");
				
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
				goStraight_change();
				printf("goStraight_change\r\n");
			}
		}
		#endif
		
		#if 1
		// 有十字路口的直线前后跑
		if(0xFFFF == FMSDS_Ptr->MSD_Hex)
		{
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
				backStatus_change();
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				CHANGE_TO_STOP_MODE();
			}
			
		}
		#endif

		
		if((0xFFFF == FMSDS_Ptr->MSD_Hex) && (0xFFFF == RMSDS_Ptr->MSD_Hex))
		{
			CHANGE_TO_STOP_MODE();
		}
		
		
#endif
	}
	else if((cirLeft == ctrlParasPtr->agvStatus) || (cirRight == ctrlParasPtr->agvStatus))
	{
		if(0x0000 == FMSDS_Ptr->MSD_Hex)
		{
			if(cirLeft == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 1;
				ctrlParasPtr->settedSpeed = 0;
				ctrlParasPtr->leftMotorSettedSpeed = 0;
				ctrlParasPtr->rightMotorSettedSpeed = 0;
				ctrlParasPtr->leftMotorSpeedOffset = 0;
				ctrlParasPtr->rightMotorSpeedOffset = 0;
				MOTOR_RIGHT_DUTY_SET(0);
				MOTOR_LEFT_DUTY_SET(0);
				CHANGE_TO_STOP_MODE();
				printf("cirLeft_change\r\n");
			}
			else if(cirRight == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 1;
				ctrlParasPtr->settedSpeed = 0;
				ctrlParasPtr->leftMotorSettedSpeed = 0;
				ctrlParasPtr->rightMotorSettedSpeed = 0;
				ctrlParasPtr->leftMotorSpeedOffset = 0;
				ctrlParasPtr->rightMotorSpeedOffset = 0;
				MOTOR_RIGHT_DUTY_SET(0);
				MOTOR_LEFT_DUTY_SET(0);
				CHANGE_TO_STOP_MODE();
				printf("cirRight_change\r\n");
			}
		}
		
	}

}


void AGV_Proc(void)
{
	u8 flag = 4;

	// 跑出去的紧急模式
	if(1 == flag)
	{
		if((goStraightStatus == ctrlParasPtr->agvStatus) || (backStatus == ctrlParasPtr->agvStatus) ||\
			(gSslow == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
		{
			if((0xFFFF == FMSDS_Ptr->MSD_Hex) && (0xFFFF == RMSDS_Ptr->MSD_Hex))
			{
				CHANGE_TO_STOP_MODE();
			}
		}
	}
	

	if(2 == flag)
	{
		if(0x0000 == FMSDS_Ptr->MSD_Hex)
		{
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
				backStatus_change();
				printf("backStatus_change\r\n");
				Delay_ns(2);
				
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
				goStraight_change();
				
				printf("goStraight_change\r\n");
				Delay_ns(2);
			}
		}

		if(0xFFFF == FMSDS_Ptr->MSD_Hex)
		{
			MOTOR_RIGHT_DUTY_SET(0);
			MOTOR_LEFT_DUTY_SET(0);
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
			}
		}
		
		/*
		if((0xFFFF == FMSDS_Ptr->MSD_Hex) && (0xFFFF == RMSDS_Ptr->MSD_Hex))
		{
			//CHANGE_TO_STOP_MODE();
			MOTOR_RIGHT_DUTY_SET(0);
			MOTOR_LEFT_DUTY_SET(0);
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
			}
		}
		*/
		
		if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_3))
		{
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
			}
		}
	}


	
	if(3 == flag)
	{
		if(0x0000 == FMSDS_Ptr->MSD_Hex)
		{
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
				backStatus_change();
				printf("backStatus_change\r\n");
				Delay_ns(2);
				
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
				goStraight_change();
				
				printf("goStraight_change\r\n");
				Delay_ns(2);
			}
		}

		if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_8) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_8))
		{
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
			}
		}
		
		if((0xFFFF == FMSDS_Ptr->MSD_Hex) && (0xFFFF == RMSDS_Ptr->MSD_Hex))
		{
			MOTOR_RIGHT_DUTY_SET(0);
			MOTOR_LEFT_DUTY_SET(0);
			MOTOR_POWER_OFF();
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 0;
			}
			//MOTOR_POWER_OFF();
		}
		else
		{
			if(1 == PDin(15))
			{
				MOTOR_POWER_ON();
			}
			
		}
	}

	if(4 == flag)
	{
		if(0x0000 == FMSDS_Ptr->MSD_Hex)
		{
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->BSflag = 0;
				backStatus_change();
				printf("backStatus_change\r\n");
				Delay_ns(2);
				
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->BSflag = 0;
				goStraight_change();
				
				printf("goStraight_change\r\n");
				Delay_ns(2);
			}
		}
	}
	
}


void AGV_Walking_Test(void)
{
	//MOTOR_POWER = 0;
	
	#if 1
	//MOTOR_RIGHT_CR_PIN_SET();
	//MOTOR_LEFT_CR_PIN_SET();
	CHANGE_TO_GO_STRAIGHT_MODE();
	//CHANGE_TO_TEST_MODE();
	//CHANGE_TO_CIR_LEFT_MODE();
	//CHANGE_TO_CIR_LEFT_MODE();
	#else
	CHANGE_TO_BACK_MODE();
	#endif
	//MOTOR_LEFT_STOP_PIN_SET();
	#if 1

	TRIGGER_PIN_O = 0;
	
	ctrlParasPtr->settedSpeed = AgvGear[10];
	ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed + 2;
	ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	
	MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
	MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
	#endif
	//CHANGE_TO_BACK_MODE();
}



void STATION_1AND2_WalkControl(void)
{
	
	if(step_gS == ctrlParasPtr->walkingstep)
	{
		//CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
		CHANGE_TO_GO_STRAIGHT_MODE();
		
		if(1 == ctrlParasPtr->crossRoadCountF)
		{
			ctrlParasPtr->agvStatus = gSslow;
			ctrlParasPtr->gear = 3;
			//printf("ctrlParasPtr->gear = 3\r\n");
		}
		else
		{
			ctrlParasPtr->gear = 10;
		}

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			
			RFID_Info_Ptr->updateFlag = 0;
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			//printf("1LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}
		/*
		else if(1 == ctrlParasPtr->crossRoadCountF)
		{
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}
		*/
	
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 10;
		
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;

			RFID_Info_Ptr->rfidData = 0;
		}
		
	}
}


void STATION_3AND4_WalkControl(void)
{	
	if(step_gS == ctrlParasPtr->walkingstep)
	{
		CHANGE_TO_GO_STRAIGHT_MODE();
		
		if(2 == ctrlParasPtr->crossRoadCountF)
		{
			ctrlParasPtr->agvStatus = gSslow;
			ctrlParasPtr->gear = 3;
			//printf("ctrlParasPtr->gear = 3\r\n");
		}
		else
		{
			ctrlParasPtr->gear = 10;
		}

		
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			//printf("2LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
			
			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				
				//printf("info: %d, %d, %d, %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->crossRoadCountF, ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
				ctrlParasPtr->walkingstep = step_gVeer;
			}
		}
		/*
		else if(2 == ctrlParasPtr->crossRoadCountF)
		{
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				
				//printf("info: %d, %d, %d, %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->crossRoadCount, ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
				ctrlParasPtr->walkingstep = step_gVeer;
			}
		}
		*/
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 10;

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;

			RFID_Info_Ptr->rfidData = 0;
		}
	}
}

void STATION_5AND6_WalkControl(void)
{
	
	if(step_gS == ctrlParasPtr->walkingstep)
	{
		CHANGE_TO_GO_STRAIGHT_MODE();
		
		if(3 == ctrlParasPtr->crossRoadCountF)
		{
			ctrlParasPtr->agvStatus = gSslow;
			ctrlParasPtr->gear = 3;
			//printf("ctrlParasPtr->gear = 3\r\n");
		}
		else
		{
			ctrlParasPtr->gear = 10;
		}

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			//printf("updateFlag = %d, cr = %d\r\n", RFID_Info_Ptr->updateFlag, ctrlParasPtr->crossRoadCountF);
			RFID_Info_Ptr->updateFlag = 0;

			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			//printf("3LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
			
		}
		/*
		else if(3 == ctrlParasPtr->crossRoadCountF)
		{
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}
		*/
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 10;

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;

			RFID_Info_Ptr->rfidData = 0;
		}
	}
}


void STATION_7AND8_WalkControl(void)
{
	
	if(step_gS == ctrlParasPtr->walkingstep)
	{
		CHANGE_TO_GO_STRAIGHT_MODE();

		if(4 == ctrlParasPtr->crossRoadCountF)
		{
			ctrlParasPtr->agvStatus = gSslow;
			ctrlParasPtr->gear = 3;
			//printf("ctrlParasPtr->gear = 3\r\n");
		}
		else
		{
			ctrlParasPtr->gear = 10;
		}
		
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			//printf("4LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
			
			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
			
		}
		/*
		else if(4 == ctrlParasPtr->crossRoadCountF)
		{
			ctrlParasPtr->gear = 3;
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}
		*/
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 10;

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;

			RFID_Info_Ptr->rfidData = 0;
		}
	}
}


void STATION_9AND10_WalkControl(void)
{
	
	if(step_gS == ctrlParasPtr->walkingstep)
	{
		CHANGE_TO_GO_STRAIGHT_MODE();

		ctrlParasPtr->gear = 10;
			
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			//printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			printf("5LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
			
		}
		/*
		else if(5 == ctrlParasPtr->crossRoadCountF)
		{
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}
		*/
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 10;

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;

			RFID_Info_Ptr->rfidData = 0;
		}

	}
}

void SHOW_ARR(s16 *arr, int head)
{
	int cir = 0;
	for(cir = 0; cir <= head; cir++)
	{
		printf("arr[%d] = %d\r\n", cir, arr[cir]);
	}
}

void PUSH_INTO_ARR(s16 *arr, int head, s16 data)
{
	int cir = 0, cir2 = 0;

	arr[head] = data;
	//printf("arr[%d] = %d\r\n", head, data);
	
	for(cir = 0; cir < head; cir++)
	{
		if(arr[cir] > data)
		{
			//printf("arr[%d] = %d > data = %d\r\n", cir, arr[cir], data);
			// 往后移以一位
			for(cir2 = head; cir2 > cir; cir2--)
			{
				arr[cir2] = arr[cir2 - 1];
				//printf("arr[%d] = %d --> arr[%d] = %d\r\n", cir2 - 1, arr[cir2 - 1], cir2, arr[cir2]);
			}
			// 插入空位
			arr[cir] = data;
			//printf("arr[%d] = %d\r\n\r\n", cir, arr[cir]);
			//SHOW_ARR(arr, head);
			//printf("\r\n");
			break;
		}
	}
	
}

void MPU6050_Data_init(void)
{
	int cir = 0;
	s16 tempArr[1000];

	for(cir = 0; cir < 1000; cir++)
	{
		tempArr[cir] = 0;
	}

	cir = 0;

	while(cir < 1000)
	{
		MPU6050_getADC(&mpu6050DS_ptr->ax, &mpu6050DS_ptr->ay, &mpu6050DS_ptr->az, &mpu6050DS_ptr->gx, &mpu6050DS_ptr->gy, &mpu6050DS_ptr->gz, &mpu6050DS_ptr->tempture);

		if(1 == MPU6050_UPDATE)
		{
			s32 temp = 0;
			MPU6050_UPDATE = 0;
			
			//printf("ay_%04d = %d\r\n", cir, mpu6050DS_ptr->ay);
			mpu6050DS_ptr->sum += mpu6050DS_ptr->ay;
			//printf("sum = %d\r\n", mpu6050DS_ptr->sum);
			//tempArr[cir] = mpu6050DS_ptr->ay;
			//printf("tempArr[%d] = %d\r\n", cir, tempArr[cir]);
			PUSH_INTO_ARR(tempArr, cir, mpu6050DS_ptr->ay);
			cir++;
			
			if(((cir / 100) == 1) && ((cir % 100) == 0))
			{
				temp = mpu6050DS_ptr->sum / 100;
				//printf("sum = %d, temp = %d\r\n", mpu6050DS_ptr->sum, temp);
				mpu6050DS_ptr->ayAverage = temp;
				//printf("%d, 1, ayAverage = %d\r\n", cir, mpu6050DS_ptr->ayAverage);
				mpu6050DS_ptr->sum = 0;
			}
			else if(((cir / 100) > 1) && ((cir % 100) == 0))
			{
				temp = mpu6050DS_ptr->sum / 100;
				//printf("sum = %d, temp = %d\r\n", mpu6050DS_ptr->sum, temp);
				mpu6050DS_ptr->ayAverage = (mpu6050DS_ptr->ayAverage + temp) / 2;
				mpu6050DS_ptr->sum = 0;
				//printf("%d, %d, ayAverage = %d\r\n", cir, (cir / 100), mpu6050DS_ptr->ayAverage);
			}
			
		}
		
	}
	
	mpu6050DS_ptr->ayMax = tempArr[899];
	mpu6050DS_ptr->ayMin = tempArr[199];
	mpu6050DS_ptr->ayMid = tempArr[499];

	printf("ayMax = %d, ayAverage = %d, ayMin = %d\r\n", mpu6050DS_ptr->ayMax, mpu6050DS_ptr->ayAverage, mpu6050DS_ptr->ayMin);
}

void MPU6050_Data_init2(void)
{
	int cir = 0;
	s16 tempArr[100];

	for(cir = 0; cir < 100; cir++)
	{
		tempArr[cir] = 0;
	}

	cir = 0;

	while(cir < 100)
	{
		MPU6050_getADC(&mpu6050DS_ptr->ax, &mpu6050DS_ptr->ay, &mpu6050DS_ptr->az, &mpu6050DS_ptr->gx, &mpu6050DS_ptr->gy, &mpu6050DS_ptr->gz, &mpu6050DS_ptr->tempture);

		if(1 == MPU6050_UPDATE)
		{
			s32 temp = 0;
			MPU6050_UPDATE = 0;
			
			printf("ay_%04d = %d\r\n", cir, mpu6050DS_ptr->ay);
			mpu6050DS_ptr->sum += mpu6050DS_ptr->ay;
			printf("sum = %d\r\n", mpu6050DS_ptr->sum);
			
			PUSH_INTO_ARR(tempArr, cir, mpu6050DS_ptr->ay);
			cir++;

			if(((cir / 100) == 1) && ((cir % 100) == 0))
			{
				temp = mpu6050DS_ptr->sum / 100;
				printf("sum = %d, temp = %d\r\n", mpu6050DS_ptr->sum, temp);
				mpu6050DS_ptr->ayAverage = temp;
				printf("%d, 1, ayAverage = %d\r\n", cir, mpu6050DS_ptr->ayAverage);
				mpu6050DS_ptr->sum = 0;
			}
			else if(((cir / 100) > 1) && ((cir % 100) == 0))
			{
				temp = mpu6050DS_ptr->sum / 100;
				printf("sum = %d, temp = %d\r\n", mpu6050DS_ptr->sum, temp);
				mpu6050DS_ptr->ayAverage = (mpu6050DS_ptr->ayAverage + temp) / 2;
				mpu6050DS_ptr->sum = 0;
				printf("%d, %d, ayAverage = %d\r\n", cir, (cir / 100), mpu6050DS_ptr->ayAverage);
			}

			
		}
		
	}
	
	mpu6050DS_ptr->ayMax = tempArr[89];
	mpu6050DS_ptr->ayMin = tempArr[19];

	printf("ayMax = %d, ayAverage = %d, ayMin = %d\r\n", mpu6050DS_ptr->ayMax, mpu6050DS_ptr->ayAverage, mpu6050DS_ptr->ayMin);
}


void MPU6050_Data_init3(void)
{
	int cir = 0, cir2 = 0;
	s16 tempArr[300], tempArr2[30];

	for(cir = 0; cir < 300; cir++)
	{
		tempArr[cir] = 0;
	}

	for(cir = 0; cir < 30; cir++)
	{
		tempArr2[cir] = 0;
	}

	cir = 0;

	while(cir2 < 100)
	{
		MPU6050_getADC(&mpu6050DS_ptr->ax, &mpu6050DS_ptr->ay, &mpu6050DS_ptr->az, &mpu6050DS_ptr->gx, &mpu6050DS_ptr->gy, &mpu6050DS_ptr->gz, &mpu6050DS_ptr->tempture);

		if(1 == MPU6050_UPDATE)
		{

			MPU6050_UPDATE = 0;
						
			cir2++;
			
		}
		
	}
	cir2 = 0;
	
	for(cir = 0; cir < 300; cir++)
	{
		while(cir2 < 10)
		{
			MPU6050_getADC(&mpu6050DS_ptr->ax, &mpu6050DS_ptr->ay, &mpu6050DS_ptr->az, &mpu6050DS_ptr->gx, &mpu6050DS_ptr->gy, &mpu6050DS_ptr->gz, &mpu6050DS_ptr->tempture);

			if(1 == MPU6050_UPDATE)
			{

				MPU6050_UPDATE = 0;
				
				mpu6050DS_ptr->sum += mpu6050DS_ptr->ay;
				
				//printf("%d\r\n", mpu6050DS_ptr->ay);
				
				cir2++;
				
			}
			
		}
		
		tempArr[cir] = mpu6050DS_ptr->sum / 10;
		
		mpu6050DS_ptr->sum = 0;
		cir2 = 0;
		
		mpu6050DS_ptr->ayMax = MaxValu(mpu6050DS_ptr->ayMax, tempArr[cir]);
		mpu6050DS_ptr->ayMin = MinValu(mpu6050DS_ptr->ayMin, tempArr[cir]);
	}
	
	//printf("ayMax = %d, ayMin = %d\r\n", mpu6050DS_ptr->ayMax, mpu6050DS_ptr->ayMin);
	//printf("******************\r\n");

	/*
	for(cir = 0; cir < 300; cir++)
	{
		printf("%d\r\n", tempArr[cir]);
	}
	*/
	
	mpu6050DS_ptr->sum = 0;
	
	for(cir = 0; cir < 30; cir++)
	{
		for(cir2 = 0; cir2 < 10; cir2++)
		{
			mpu6050DS_ptr->sum += tempArr[cir * 10 + cir2];
		}
		
		tempArr2[cir] = mpu6050DS_ptr->sum / 10;
		mpu6050DS_ptr->sum = 0;
	}

	for(cir = 0; cir < 30; cir++)
	{
		mpu6050DS_ptr->sum += tempArr2[cir];
	}

	mpu6050DS_ptr->ayMid = mpu6050DS_ptr->sum / 30;
	mpu6050DS_ptr->sum = 0;

	//printf("ayMid = %d\r\n", mpu6050DS_ptr->ayMid);
	printf("ayMax = %d, ayMid = %d, ayMin = %d\r\n", mpu6050DS_ptr->ayMax, mpu6050DS_ptr->ayMid, mpu6050DS_ptr->ayMin);
	mpu6050DS_ptr->ayMax = mpu6050DS_ptr->ayMid + (mpu6050DS_ptr->ayMax - mpu6050DS_ptr->ayMid) * 2.5;
	mpu6050DS_ptr->ayMin = mpu6050DS_ptr->ayMid - (mpu6050DS_ptr->ayMid - mpu6050DS_ptr->ayMin) * 2.5;
	printf("ayMax = %d, ayMid = %d, ayMin = %d\r\n\r\n", mpu6050DS_ptr->ayMax, mpu6050DS_ptr->ayMid, mpu6050DS_ptr->ayMin);
}


void MPU6050_Get_Average_10(void)
{
	static u8 counter = 0;
	static s32 sum = 0;
	
	MPU6050_getADC(&(mpu6050DS_ptr->ax), &(mpu6050DS_ptr->ay), &(mpu6050DS_ptr->az), &(mpu6050DS_ptr->gx), &(mpu6050DS_ptr->gy), &(mpu6050DS_ptr->gz), &(mpu6050DS_ptr->tempture));

	
	if(1 == MPU6050_UPDATE)
	{
		MPU6050_UPDATE = 0;

		if(counter < 10)
		{
			sum = sum + mpu6050DS_ptr->ay;
			counter++;
			//printf("%d\r\n", mpu6050DS_ptr->ay);
		}
		else
		{
			//printf("sum = %d\r\n", sum);
			mpu6050DS_ptr->ayAverage = sum / 10;
			mpu6050DS_ptr->ayAverageUpdate = 1;
			counter = 0;
			sum = 0;
			//printf("*************\r\nayAverage = %d\r\n", mpu6050DS_ptr->ayAverage);
		}
		
	}
}

void MPU6050_Get_Average_50(void)
{
	static u8 counter = 0;
	static s32 sum = 0;
	
	MPU6050_getADC(&(mpu6050DS_ptr->ax), &(mpu6050DS_ptr->ay), &(mpu6050DS_ptr->az), &(mpu6050DS_ptr->gx), &(mpu6050DS_ptr->gy), &(mpu6050DS_ptr->gz), &(mpu6050DS_ptr->tempture));

	
	if(1 == MPU6050_UPDATE)
	{
		MPU6050_UPDATE = 0;

		if(counter < 2)
		{
			sum = sum + mpu6050DS_ptr->ay;
			counter++;
			//printf("%d\r\n", mpu6050DS_ptr->ay);
		}
		else
		{
			//printf("sum = %d\r\n", sum);
			mpu6050DS_ptr->ayAverage = sum / 2;
			mpu6050DS_ptr->ayAverageUpdate = 1;
			counter = 0;
			sum = 0;
			//printf("*************\r\nayAverage = %d\r\n", mpu6050DS_ptr->ayAverage);
		}
		
	}
}


void MS_Location_Translate(void)
{
	float locationNum = 0;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
	{
		if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			locationNum = (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center) / 2.0;
		}
		else
		{
			locationNum = FMSDS_Ptr->AgvMSLocation - Agv_MS_Left_2_5;
		}
		
		printf("%.1f, ", locationNum);
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_5))
		{
			locationNum = (float)(FMSDS_Ptr->AgvMSLocation - Agv_MS_Center) / 2.0;
		}
		else
		{
			locationNum = FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_2_5;
		}
		
		printf("%.1f, ", locationNum);
	}
	else if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		locationNum = 0;
		
		printf("%.1f, ", locationNum);
	}
}

void MPU6050_Data(void)
{
	MPU6050_Get_Average_50();
	
#if 1
	
	if(1 == mpu6050DS_ptr->ayAverageUpdate)
	{
		if((mpu6050DS_ptr->ayAverage >= mpu6050DS_ptr->ayMin) && (mpu6050DS_ptr->ayAverage <= mpu6050DS_ptr->ayMax))
		{
			mpu6050DS_ptr->ayOffset = 0;
		}
		else
		{
			if(mpu6050DS_ptr->ayAverage > mpu6050DS_ptr->ayMax)
			{
				mpu6050DS_ptr->ayOffset = (mpu6050DS_ptr->ayAverage - mpu6050DS_ptr->ayMax) * 2;
			}
			else if(mpu6050DS_ptr->ayAverage < mpu6050DS_ptr->ayMin)
			{
				mpu6050DS_ptr->ayOffset = (mpu6050DS_ptr->ayAverage - mpu6050DS_ptr->ayMin) * 2;
			}
			
		}
		
		//MS_Location_Translate();
		//printf("%d, %d\r\n", mpu6050DS_ptr->ayOffset, mpu6050DS_ptr->az);
		mpu6050DS_ptr->ayAverageUpdate = 0;
		
	}
	
#else

	if(1 == mpu6050DS_ptr->ayAverageUpdate)
	{
		if((mpu6050DS_ptr->ayAverage >= mpu6050DS_ptr->ayMin) && (mpu6050DS_ptr->ayAverage <= mpu6050DS_ptr->ayMax))
		{
			mpu6050DS_ptr->ayOffset = 0;
		}
		else
		{
			mpu6050DS_ptr->ayOffset = (mpu6050DS_ptr->ayAverage - mpu6050DS_ptr->ayMid);
		}
		
		MS_Location_Translate();
		printf("%d\r\n", mpu6050DS_ptr->ayOffset);
		mpu6050DS_ptr->ayAverageUpdate = 0;
	}

#endif
	
}



void MPU6050_Data1(void)
{
	
	
	MPU6050_getADC1(&(mpu6050DS_ptr->ax), &(mpu6050DS_ptr->ay), &(mpu6050DS_ptr->az), &(mpu6050DS_ptr->gx), &(mpu6050DS_ptr->gy), &(mpu6050DS_ptr->gz), &(mpu6050DS_ptr->tempture));

	
	if(1 == MPU6050_UPDATE)
	{
		MPU6050_UPDATE = 0;

		MS_Location_Translate();
		printf("%d\r\n", mpu6050DS_ptr->ay);
	}
	
}



void Hall_Count(void)
{
	static u8 crcRec = 0;
	static u8 hallCountFlag = 0;
	
	if(crcRec != ctrlParasPtr->crossRoadCountF)
	{
		
	}
	
}


void CrossRoad_Count(void)
{
	static Agv_MS_Location mslRec = AgvInits;

	//if((goStraightStatus == ctrlParasPtr->agvStatus) || (backStatus == ctrlParasPtr->agvStatus) ||\
		//(gSslow == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
	if((goStraightStatus == ctrlParasPtr->agvStatus) || (backStatus == ctrlParasPtr->agvStatus))
	{
		if(mslRec != FMSDS_Ptr->AgvMSLocation)
		{
			mslRec = FMSDS_Ptr->AgvMSLocation;

			
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountF < 5)
					{
						ctrlParasPtr->rightHallCounter = 0;
						ctrlParasPtr->leftHallCounter = 0;
						ctrlParasPtr->crossRoadCountF++;
						ctrlParasPtr->crossRoadUpdateF = 1;
						//printf("1Hclean\r\n");
						//printf("GcrossRoadCountF = %d\r\n", ctrlParasPtr->crossRoadCountF);
					}
					else
					{
						//printf("crossRoadCountF error\r\n");
						ctrlParasPtr->crossRoadCountF = 5;
					}
				}
				else if(Agv_MS_CrossRoad == RMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountR < 5)
					{
						ctrlParasPtr->crossRoadCountR++;
						ctrlParasPtr->crossRoadUpdateR = 1;
						//printf("1Hclean\r\n");
						//printf("GcrossRoadUpdateR = %d\r\n", ctrlParasPtr->crossRoadUpdateR);
					}
					else
					{
						//printf("crossRoadCountR error\r\n");
					}
				}
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountF > 0)
					{
						ctrlParasPtr->rightHallCounter = 0;
						ctrlParasPtr->leftHallCounter = 0;
						ctrlParasPtr->crossRoadCountF--;
						ctrlParasPtr->crossRoadUpdateF = 1;
						//printf("2Hclean\r\n");
						//printf("BcrossRoadCountF = %d\r\n", ctrlParasPtr->crossRoadCountF);
					}
					else
					{
						//printf("crossRoadCountF error\r\n");
					}
				}
				else if(Agv_MS_CrossRoad == RMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountR > 0)
					{
						ctrlParasPtr->crossRoadCountR--;
						ctrlParasPtr->crossRoadUpdateR = 1;
						//printf("2Hclean\r\n");
						//printf("BcrossRoadUpdateR = %d\r\n", ctrlParasPtr->crossRoadUpdateR);
					}
					else
					{
						//printf("crossRoadCountR error\r\n");
					}
				}
				
			}

			
		}
	}
	
	
}


void CrossRoad_Count2(void)
{
	static Agv_MS_Location mslRec = AgvInits;

	if((goStraightStatus == ctrlParasPtr->agvStatus) || (backStatus == ctrlParasPtr->agvStatus) ||\
		(gSslow == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
	{
		if(mslRec != FMSDS_Ptr->AgvMSLocation)
		{
			mslRec = FMSDS_Ptr->AgvMSLocation;

			
			if((goStraightStatus == ctrlParasPtr->agvStatus) || (gSslow == ctrlParasPtr->agvStatus))
			{
				if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountF < 5)
					{
						ctrlParasPtr->rightHallCounter = 0;
						ctrlParasPtr->leftHallCounter = 0;
						ctrlParasPtr->crossRoadCountF++;
						ctrlParasPtr->crossRoadUpdateR = 1;
						printf("1Hclean\r\n");
						printf("crossRoadCount = %d\r\n", ctrlParasPtr->crossRoadCountF);
					}
					else
					{
						printf("crossRoadCount error\r\n");
					}
				}
				
			}
			else if((backStatus == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
			{
				if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountF > 0)
					{
						ctrlParasPtr->rightHallCounter = 0;
						ctrlParasPtr->leftHallCounter = 0;
						ctrlParasPtr->crossRoadCountF--;
						ctrlParasPtr->crossRoadUpdateR = 1;
						printf("2Hclean\r\n");
						printf("crossRoadCount = %d\r\n", ctrlParasPtr->crossRoadCountF);
					}
					else
					{
						printf("crossRoadCountF error\r\n");
					}
				}
				
			}

			
		}
	}
	
	
}


void step_gVeer_Func(void)
{
	static u8 stepFlag = 0;
	
	if((SpinStation_1 == ctrlParasPtr->goalStation) || \
		(SpinStation_3 == ctrlParasPtr->goalStation) || \
		(SpinStation_5 == ctrlParasPtr->goalStation) || \
		(SpinStation_7 == ctrlParasPtr->goalStation) || \
		(SpinStation_9 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_LEFT_MODE();
		//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
		printf("info1: %d, %d, %d, %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->crossRoadCountF, ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
	}
	else if((SpinStation_2 == ctrlParasPtr->goalStation) || \
			(SpinStation_4 == ctrlParasPtr->goalStation) || \
			(SpinStation_6 == ctrlParasPtr->goalStation) || \
			(SpinStation_8 == ctrlParasPtr->goalStation) || \
			(SpinStation_10 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_RIGHT_MODE();
		//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
		//printf("info2: %d, %d, %d, %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->crossRoadCountF, ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
	}

	if(0 == stepFlag)
	{
		ctrlParasPtr->gear = 3;
		stepFlag = 1;
	}
	else if(1 == stepFlag)
	{
		if((0xFFFF == FMSDS_Ptr->MSD_Hex) && (0xFFFF == RMSDS_Ptr->MSD_Hex))
		{
			ctrlParasPtr->gear = 2;
			stepFlag = 2;
		}
	}
	else if(2 == stepFlag)
	{	
		if((0xFFFF != FMSDS_Ptr->MSD_Hex) || (0xFFFF != RMSDS_Ptr->MSD_Hex))
		{
			if(cirLeft == ctrlParasPtr->agvStatus)
			{
				if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
				{
					CHANGE_TO_CIR_RIGHT_MODE();
					
					Delay_ms(100);
					CleanAllSpeed();
					
					CHANGE_TO_STOP_MODE();
					//Delay_ms(500);
					stepFlag = 0;
					ctrlParasPtr->walkingstep = step_entry;
				}
				
			}
			else if(cirRight == ctrlParasPtr->agvStatus)
			{
				if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Center))
				{
					CHANGE_TO_CIR_LEFT_MODE();
					
					Delay_ms(100);
					CleanAllSpeed();
					
					CHANGE_TO_STOP_MODE();
					//Delay_ms(500);
					stepFlag = 0;
					ctrlParasPtr->walkingstep = step_entry;
				}
				
			}
		
			
		}
	}
	
}


void step_entry_Func(void)
{
	static u8 flag = 0;

	
	CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
	

#if 0
	
	if(0x0000 == FMSDS_Ptr->MSD_Hex)
	{
		CHANGE_TO_STOP_MODE();
		Delay_ms(500);
		
		ctrlParasPtr->walkingstep = step_catch;
	}
	
#else

	if(0x0000 == FMSDS_Ptr->MSD_Hex)
	{
		flag = 1;
	}
	
	if(0 == flag)
	{
		ctrlParasPtr->gear = 2;
	}
	else
	{
		ctrlParasPtr->gear = 1;
	}
	
	if((0 == LMT_INR) || (0 == LMT_INL))		// 接近开关响应或者是超过磁条了
	//if((0 == LMT_INR) || (0 == LMT_INL) || (0x0000 == FMSDS_Ptr->MSD_Hex))
	{
		CHANGE_TO_STOP_MODE();
		//Delay_ns(1);
		
		RFID_Info_Ptr->rfidData = 0;
		flag = 0;
		ctrlParasPtr->walkingstep = step_catch;
	}

#endif	
}

void step_catch_Func(void)
{
#if 1
	
	if(0 == LMT_SW) 	// 已经抓到货物了
	{
		//Delay_ns(3);
		
		ECV_POWER_OFF();
		
		RFID_Info_Ptr->updateFlag = 0;
		
		Send_GettedGoods(1);

		printf("change to exit\r\n");
		
		ctrlParasPtr->walkingstep = step_exit;
		
		printf("change to back\r\n");
		
		CHANGE_TO_BACK_SLOW_MODE();
		
		RFID_Info_Ptr->lock = 0x00;
	}
	else
	{
		
		BECV_UP();
		FECV_UP();
		
		ECV_POWER_ON();

		//Delay_ns(1);
		
	}
	
#else
	
	FECV_UP();
	BECV_UP();
	ECV_POWER_ON();
	Delay_ns(3);
	ECV_POWER_OFF();
	RFID_Info_Ptr->updateFlag = 0;
	printf("change to back\r\n");
	ctrlParasPtr->walkingstep = step_exit;
	CHANGE_TO_BACK_SLOW_MODE();
	
#endif
	
}

void step_exit_Func(void)
{
	
	ctrlParasPtr->gear = 3;
	
	if(1 == RFID_Info_Ptr->updateFlag)
	{
		RFID_Info_Ptr->updateFlag = 0;
		//printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);

		if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
		{
			CHANGE_TO_STOP_MODE();
			//Delay_ns(1);
			ctrlParasPtr->walkingstep = step_weigh;
		}
	}
	else
	{
		static Agv_MS_Location mslRec = AgvInits;
		static u8 fla = 0;

		if(mslRec != FMSDS_Ptr->AgvMSLocation)
		{
			mslRec = FMSDS_Ptr->AgvMSLocation;
			
			if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
			{
				fla = 1;
				printf("Agv_MS_CrossRoad\r\n");
			}
		}	
		

		if(2 == fla)
		{
			//printf("LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGB[ctrlParasPtr->crossRoadCountF].HallCountRight) ||\
			(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGB[ctrlParasPtr->crossRoadCountF].HallCountLeft))
			{
				fla = 0;
				CHANGE_TO_STOP_MODE();
				//Delay_ns(1);
				ctrlParasPtr->walkingstep = step_weigh;
			}
		}
		
		
	}
}


void step_weigh_Func(void)
{
	
	ECV_POWER_ON();
	//FECV_STOP();
	BECV_DOWN();
	//WECV_UP();
	//Delay_ns(3);
	
	//FECV_UP();
	//BECV_UP();

	//Delay_ns(3);
	//WECV_STOP();

	//WECV_DOWN();
	//Delay_ns(6);

	if(0 == Return_SW)
	{
		ECV_POWER_OFF();
		ctrlParasPtr->walkingstep = step_bVeer;
	}
	
}

void step_bVeer_Func(void)
{
	static u8 stepFlag = 0;
	
	if((SpinStation_1 == ctrlParasPtr->goalStation) || \
		(SpinStation_3 == ctrlParasPtr->goalStation) || \
		(SpinStation_5 == ctrlParasPtr->goalStation) || \
		(SpinStation_7 == ctrlParasPtr->goalStation) || \
		(SpinStation_9 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_RIGHT_MODE();
	}
	else if((SpinStation_2 == ctrlParasPtr->goalStation) || \
			(SpinStation_4 == ctrlParasPtr->goalStation) || \
			(SpinStation_6 == ctrlParasPtr->goalStation) || \
			(SpinStation_8 == ctrlParasPtr->goalStation) || \
			(SpinStation_10 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_LEFT_MODE();
	}

	if(0 == stepFlag)
	{
		ctrlParasPtr->gear = 4;
		stepFlag = 1;
	}
	else if(1 == stepFlag)
	{
		if((0xFFFF == FMSDS_Ptr->MSD_Hex) && (0xFFFF == RMSDS_Ptr->MSD_Hex))
		{
			ctrlParasPtr->gear = 2;
			stepFlag = 2;
		}
	}
	else if(2 == stepFlag)
	{	
		if((0xFFFF != FMSDS_Ptr->MSD_Hex) || (0xFFFF != RMSDS_Ptr->MSD_Hex))
		{
			if(cirLeft == ctrlParasPtr->agvStatus)
			{
				if((RMSDS_Ptr->AgvMSLocation >= Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
				{
					CHANGE_TO_CIR_RIGHT_MODE();
					
					Delay_ms(100);
					CleanAllSpeed();
					
					CHANGE_TO_STOP_MODE();
					//Delay_ms(500);
					Send_WaitForGoods();
					stepFlag = 0;
					ctrlParasPtr->BSflag = 0;
					ctrlParasPtr->walkingstep = step_gB;
				}
			}
			else if(cirRight == ctrlParasPtr->agvStatus)
			{
				if((RMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (RMSDS_Ptr->AgvMSLocation <= Agv_MS_Center))
				{
					CHANGE_TO_CIR_LEFT_MODE();
					
					Delay_ms(100);
					CleanAllSpeed();
					
					CHANGE_TO_STOP_MODE();
					//Delay_ms(500);
					Send_WaitForGoods();
					stepFlag = 0;
					ctrlParasPtr->BSflag = 0;
					ctrlParasPtr->walkingstep = step_gB;
				}
			}
			
			
		}
	}
	
}


void step_gB_Func(void)
{
	CHANGE_TO_BACK_MODE();
	FECV_DOWN();
	BECV_DOWN();
	ECV_POWER_ON();
	//ctrlParasPtr->BSflag = 1;

	if((0 == ctrlParasPtr->crossRoadCountF) && (0 == ctrlParasPtr->crossRoadCountR))
	{
		//ctrlParasPtr->agvStatus = bSslow;
		//ctrlParasPtr->gear = 3;
	}
	
	
	if((0x0000 == FMSDS_Ptr->MSD_Hex) && (0 == ctrlParasPtr->crossRoadCountF))
	{
		CHANGE_TO_STOP_MODE();
		ECV_POWER_OFF();
		ctrlParasPtr->gear = 10;
		RFID_Info_Ptr->updateFlag = 0;
		ctrlParasPtr->walkingstep = step_wFTans;
				
	}
}

void step_wFTans_Func(void)
{
	static u32 timRec = 0;
	
	if(0 == timRec)
	{
		Send_Arrive();
		ctrlParasPtr->crossRoadCountF = 0;
		ctrlParasPtr->crossRoadCountR = 0;
		BECV_UP();
		timRec = SystemRunningTime;
		//printf("timRec = %d\r\n", timRec);
	}
	else
	{
		//printf("%d\r\n", SystemRunningTime);
		if(SystemRunningTime - timRec >= 10000)
		{
			BECV_STOP();
			CMD_Flag_Ptr->cmdFlag = GoodLeav;
		}

		if(GoodLeav == CMD_Flag_Ptr->cmdFlag)
		{
			timRec = 0;
			RFID_Info_Ptr->lock = 0x00;
			ctrlParasPtr->walkingstep = step_origin;
		}
	}
	
	
}

void step_origin_Func(void)
{
	ctrlParasPtr->FSflag = 0;
	
	CHANGE_TO_GO_STRAIGHT_MODE();
	
	if(1 == ctrlParasPtr->crossRoadCountF)
	{
		ctrlParasPtr->agvStatus = gSslow;
		ctrlParasPtr->gear = 3;
		//printf("ctrlParasPtr->gear = 3\r\n");
	}
	else
	{
		ctrlParasPtr->gear = 10;
	}

	if(1 == RFID_Info_Ptr->updateFlag)
	{
		
		RFID_Info_Ptr->updateFlag = 0;
		printf("step_origin_Func\r\n");
		printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
		//printf("1LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
		if(0x01 == RFID_Info_Ptr->rfidData)
		{
			CHANGE_TO_STOP_MODE();
			//Delay_ms(500);
			ctrlParasPtr->walkingstep = step_stop;
			
		}
		else if(RFID_Info_Ptr->rfidData > 0x01)
		{
			ctrlParasPtr->BSflag = 0;
			CHANGE_TO_BACK_MODE();
		}
	}
	/*
	else if(1 == ctrlParasPtr->crossRoadCountF)
	{
		if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountRight) ||\
			(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCountF].HallCountLeft))
		{
			CHANGE_TO_STOP_MODE();
			//Delay_ms(500);
			ctrlParasPtr->walkingstep = step_stop;
			
		}
	}
	*/
}

void step_stop_Func(void)
{
	ctrlParasPtr->FSflag = 0;
	ctrlParasPtr->goalStation = ControlCenter;
	ctrlParasPtr->goalRFIDnode = 0x0000;
	ctrlParasPtr->walkingstep = step_stop;
}

void Walking_Step_Controler(void)
{
	if(STATION_1AND2_RFID == ctrlParasPtr->goalRFIDnode)
	{
		STATION_1AND2_WalkControl();
	}
	else if(STATION_3AND4_RFID == ctrlParasPtr->goalRFIDnode)
	{
		STATION_3AND4_WalkControl();
	}
	else if(STATION_5AND6_RFID == ctrlParasPtr->goalRFIDnode)
	{
		STATION_5AND6_WalkControl();
	}
	else if(STATION_7AND8_RFID == ctrlParasPtr->goalRFIDnode)
	{
		STATION_7AND8_WalkControl();
	}
	else if(STATION_9AND10_RFID == ctrlParasPtr->goalRFIDnode)
	{
		STATION_9AND10_WalkControl();
	}
	else
	{
		
	}


	if(step_gVeer == ctrlParasPtr->walkingstep)
	{
		step_gVeer_Func();
	}
	else if(step_entry == ctrlParasPtr->walkingstep)
	{
		step_entry_Func();
	}
	else if(step_catch == ctrlParasPtr->walkingstep)
	{
		step_catch_Func();
	}
	else if(step_exit == ctrlParasPtr->walkingstep)
	{
		step_exit_Func();
	}
	else if(step_weigh == ctrlParasPtr->walkingstep)
	{
		step_weigh_Func();
	}
	else if(step_bVeer == ctrlParasPtr->walkingstep)
	{
		step_bVeer_Func();
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		step_gB_Func();
	}
	else if(step_wFTans == ctrlParasPtr->walkingstep)
	{
		step_wFTans_Func();
	}
	else if(step_origin == ctrlParasPtr->walkingstep)
	{
		step_origin_Func();
	}
	else if(step_stop == ctrlParasPtr->walkingstep)
	{
		step_stop_Func();
	}
}

void ProtectFunc(void)
{
	if((0 == ProtectSW_F) || (0 == ProtectSW_R))
	{
		MOTOR_POWER_OFF();
	}
}

void WarningLedTwinkleCtrl(void)
{
	static u32 timRec = 0;
	static u8 twinkleCounter = 0, step = 0;
	
	if(1 == WarningLedCtrlPtr->twinkleFlag)
	{
		if(0 == timRec)
		{
			timRec = SystemRunningTime;
			Warning_LED = 1;
			step = 0;
		}
		else
		{
			
			if(0 == step)
			{
				if(SystemRunningTime - timRec >= WarningLedCtrlPtr->intervalTime_ms * 10)
				{
					Warning_LED = 0;
					step = 1;
					timRec = SystemRunningTime;
				}
			}
			else if(1 == step)
			{
				if(SystemRunningTime - timRec >= WarningLedCtrlPtr->intervalTime_ms * 10)
				{
					timRec = 0;
					twinkleCounter++;
				}
			}
			
		}

		if(twinkleCounter >= WarningLedCtrlPtr->twinkleNum)
		{
			WarningLedCtrlPtr->twinkleFlag = 0;
			twinkleCounter = 0;
			step = 0;
		}
	}
	
}


void BuzzerCtrlFunc(void)
{
	static u32 timRec = 0;
	static u8 buzzerCounter = 0, step = 0;
	
	if(1 == BuzzerCtrlPtr->buzzerFlag)
	{
		if(0 == timRec)
		{
			timRec = SystemRunningTime;
			BUZZER_1 = 1;
			step = 0;
		}
		else
		{
			
			if(0 == step)
			{
				if(SystemRunningTime - timRec >= BuzzerCtrlPtr->buzzerTime_ms * 10)
				{
					BUZZER_1 = 0;
					step = 1;
					timRec = SystemRunningTime;
				}
			}
			else if(1 == step)
			{
				if(SystemRunningTime - timRec >= BuzzerCtrlPtr->buzzerTime_ms * 10)
				{
					timRec = 0;
					buzzerCounter++;
				}
			}
			
		}

		if(buzzerCounter >= BuzzerCtrlPtr->buzzerNum)
		{
			BuzzerCtrlPtr->buzzerFlag = 0;
			buzzerCounter = 0;
			step = 0;
		}
	}
	
}


void Motion_Ctrl_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
#if 0
	/***********MOTOR RIGHT: START***************/
	/*设置为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*设置为浮空输入，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/***********MOTOR RIGHT: END***************/

	/***********MOTOR LEFT: START***************/
	/*设置为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*设置为浮空输入，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/***********MOTOR LEFT: END***************/

#else
	/*Motor Right Out*/
	/*设置为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*Motor Left Out*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*Motor Left In*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/*Motor Right In*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_8);
	MOTOR_LEFT_BK = 0;
	MOTOR_RIGHT_BK = 0;
#endif
}

void PG_EXTI_CFG(void)
{
	/* 定义EXIT初始化结构体 EXTI_InitStructure */
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource12);
	
	
	/* 设置外部中断0通道（EXIT Line2）在下降沿时触发中断 */  
  	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;		//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);								//初始化
	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource14);

	/* 设置外部中断0通道（EXIT Line2）在下降沿时触发中断 */  
  	EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;		//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);								//初始化
}


void E_pushrod_Gpio_Init(void)
{
	#if 1
	
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	/*打开APB2总线上的GPIOA时钟*/
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	#else

	
	

	#endif
}


void SW_Gpio_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	/*打开APB2总线上的GPIOA时钟*/
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
}

void Trigger_Gpio_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	GPIO_SetBits(GPIOE, GPIO_Pin_11);
}

void Motion_Ctrl_Init(void)
{
	u8 cir = 0, cir2 = 0;
	
	//Trigger_Gpio_Init();
	//Delay_ns(1);
	Motion_Ctrl_GPIO_Init();
	
	PG_EXTI_CFG();
	
	CHANGE_TO_STOP_MODE();
	
	E_pushrod_Gpio_Init();

	SW_Gpio_Init();

	//Trigger_Gpio_Init();
	
	ctrlParasPtr->agvStatus = stopStatus;
	ctrlParasPtr->settedSpeed = 0;
	ctrlParasPtr->rightMotorRealSpeed = 0;
	ctrlParasPtr->rightMotorSettedSpeed = 0;
	ctrlParasPtr->leftMotorRealSpeed = 0;
	ctrlParasPtr->leftMotorSettedSpeed = 0;
	ctrlParasPtr->leftInc = 0;
	ctrlParasPtr->rightInc = 0;
	ctrlParasPtr->speedMode = PWM_MODE;
	ctrlParasPtr->speedModeValue_Right = 0x00;
	ctrlParasPtr->speedModeValue_Left = 0x00;
	ctrlParasPtr->agvWalkingMode = AutomaticMode;
	ctrlParasPtr->leftMotorSpeedOffset = 0;
	ctrlParasPtr->rightMotorSpeedOffset = 0;
	ctrlParasPtr->leftHallIntervalTime = 0x00;
	ctrlParasPtr->rightHallIntervalTime = 0x00;
	ctrlParasPtr->HLavg = 0x00;
	ctrlParasPtr->HRavg = 0x00;
	ctrlParasPtr->avgFlag = 0;
	ctrlParasPtr->avgFlagCount = 0;
	ctrlParasPtr->gear = 0;
	ctrlParasPtr->FSflag = 0;
	ctrlParasPtr->BSflag = 0;
	ctrlParasPtr->dampingFlag = DampingNone;
	ctrlParasPtr->goalRFIDnode = 0;
	ctrlParasPtr->goalStation = ControlCenter;
	ctrlParasPtr->walkingstep = step_stop;
	ctrlParasPtr->crossRoadCountF = 0;
	ctrlParasPtr->T1DF = 0;
	ctrlParasPtr->T1dutyRec = 0;
	ctrlParasPtr->LP_duty = 0;
	ctrlParasPtr->RP_duty = 0;
	ctrlParasPtr->LD_duty = 0;
	ctrlParasPtr->RD_duty = 0;
	
	agv_walking_func[StatusStart] = NullFunc;
	agv_walking_func[stopStatus] = walking_stopStatus;
	agv_walking_func[goStraightStatus] = AGV_Correct_gS_8ug;
	agv_walking_func[backStatus] = AGV_Correct_back_ug;
	agv_walking_func[cirLeft] = walking_cirLeft;
	agv_walking_func[cirRight] = walking_cirRight;
	agv_walking_func[gSslow] = gS_slow;
	agv_walking_func[bSslow] = back_slow;
	

	ZBandRFIDmapping[ControlCenter] = 0x0000;
	ZBandRFIDmapping[SpinStation_1] = 0x0001;
	ZBandRFIDmapping[SpinStation_2] = 0x0002;
	ZBandRFIDmapping[SpinStation_3] = 0x0003;
	ZBandRFIDmapping[SpinStation_4] = 0x0004;
	ZBandRFIDmapping[SpinStation_5] = 0x0005;
	ZBandRFIDmapping[SpinStation_6] = 0x0006;
	ZBandRFIDmapping[SpinStation_7] = 0x0007;
	ZBandRFIDmapping[SpinStation_8] = 0x0008;
	ZBandRFIDmapping[SpinStation_9] = 0x0009;
	ZBandRFIDmapping[SpinStation_10] = 0x000A;

	CrossRoadHallCountArrGS[0].HallCountLeft = 0;
	CrossRoadHallCountArrGS[1].HallCountLeft = 29;
	CrossRoadHallCountArrGS[2].HallCountLeft = 29;
	CrossRoadHallCountArrGS[3].HallCountLeft = 97;
	CrossRoadHallCountArrGS[4].HallCountLeft = 120;
	CrossRoadHallCountArrGS[5].HallCountLeft = 120;

	CrossRoadHallCountArrGS[0].HallCountRight = 0;
	CrossRoadHallCountArrGS[1].HallCountRight = 29;
	CrossRoadHallCountArrGS[2].HallCountRight = 29;
	CrossRoadHallCountArrGS[3].HallCountRight = 97;
	CrossRoadHallCountArrGS[4].HallCountRight = 120;
	CrossRoadHallCountArrGS[5].HallCountRight = 120;

	CrossRoadHallCountArrGB[0].HallCountLeft = 0;
	CrossRoadHallCountArrGB[1].HallCountLeft = 29;
	CrossRoadHallCountArrGB[2].HallCountLeft = 29;
	CrossRoadHallCountArrGB[3].HallCountLeft = 29;
	CrossRoadHallCountArrGB[4].HallCountLeft = 29;
	CrossRoadHallCountArrGB[5].HallCountLeft = 29;

	CrossRoadHallCountArrGB[0].HallCountRight = 0;
	CrossRoadHallCountArrGB[1].HallCountRight = 29;
	CrossRoadHallCountArrGB[2].HallCountRight = 29;
	CrossRoadHallCountArrGB[3].HallCountRight = 29;
	CrossRoadHallCountArrGB[4].HallCountRight = 29;
	CrossRoadHallCountArrGB[5].HallCountRight = 29;

	for(cir = 0; cir < 20; cir++)
	{
		adaptInfo[cir].result = Good;
		adaptInfo[cir].timRec = 0;
		adaptInfo[cir].duty = 0;
		adaptInfo[cir].goodDuty = 0;
		adaptInfo[cir].upLock = 0;
		adaptInfo[cir].lowLock = 0;
		adaptInfo[cir].goodLock = 0;
		adaptInfo[cir].upLimDuty = 0;
		adaptInfo[cir].lowLimDuty = 0;

		adaptInfoB[cir].result = Good;
		adaptInfoB[cir].timRec = 0;
		adaptInfoB[cir].upLock = 0;
		adaptInfoB[cir].lowLock = 0;
		adaptInfoB[cir].goodLock = 0;
		adaptInfoB[cir].goodDuty = 0;
		adaptInfoB[cir].upLock = 0;
		adaptInfoB[cir].upLimDuty = 0;
		adaptInfoB[cir].lowLimDuty = 0;
	}
	/*
	adaptInfo[0].timRec = 0;
	adaptInfo[0].duty = 20;

	adaptInfo[1].timRec = 0;
	adaptInfo[1].duty = 20;

	adaptInfo[2].timRec = 0;
	adaptInfo[2].duty = 20;

	adaptInfo[3].timRec = 0;
	adaptInfo[3].duty = 20;

	adaptInfo[4].timRec = 0;
	adaptInfo[4].duty = 20;

	adaptInfo[5].timRec = 0;
	adaptInfo[5].duty = 14;

	adaptInfo[6].timRec = 0;
	adaptInfo[6].duty = 14;

	adaptInfo[7].timRec = 0;
	adaptInfo[7].duty = 12;

	adaptInfo[8].timRec = 0;
	adaptInfo[8].duty = 12;

	adaptInfo[9].timRec = 0;
	adaptInfo[9].duty = 11;

	adaptInfo[10].timRec = 0;
	adaptInfo[10].duty = 11;

	adaptInfo[11].timRec = 0;
	adaptInfo[11].duty = 10;

	adaptInfo[12].timRec = 0;
	adaptInfo[12].duty = 10;

	adaptInfo[13].timRec = 0;
	adaptInfo[13].duty = 10;

	adaptInfo[14].timRec = 0;
	adaptInfo[14].duty = 9;

	adaptInfo[15].timRec = 0;
	adaptInfo[15].duty = 9;

	adaptInfo[16].timRec = 0;
	adaptInfo[16].duty = 9;

	adaptInfo[17].timRec = 0;
	adaptInfo[17].duty = 8;

	adaptInfo[18].timRec = 0;
	adaptInfo[18].duty = 5;

	adaptInfo[19].timRec = 0;
	adaptInfo[19].duty = 4;

	
	adaptInfoB[0].timRec = 0;
	adaptInfoB[0].duty = 20;

	adaptInfoB[1].timRec = 0;
	adaptInfoB[1].duty = 20;

	adaptInfoB[2].timRec = 0;
	adaptInfoB[2].duty = 20;

	adaptInfoB[3].timRec = 0;
	adaptInfoB[3].duty = 20;

	adaptInfoB[4].timRec = 0;
	adaptInfoB[4].duty = 20;

	adaptInfoB[5].timRec = 0;
	adaptInfoB[5].duty = 16;

	adaptInfoB[6].timRec = 0;
	adaptInfoB[6].duty = 14;

	adaptInfoB[7].timRec = 0;
	adaptInfoB[7].duty = 14;

	adaptInfoB[8].timRec = 0;
	adaptInfoB[8].duty = 12;

	adaptInfoB[9].timRec = 0;
	adaptInfoB[9].duty = 12;

	adaptInfoB[10].timRec = 0;
	adaptInfoB[10].duty = 12;

	adaptInfoB[11].timRec = 0;
	adaptInfoB[11].duty = 10;

	adaptInfoB[12].timRec = 0;
	adaptInfoB[12].duty = 10;

	adaptInfoB[13].timRec = 0;
	adaptInfoB[13].duty = 10;

	adaptInfoB[14].timRec = 0;
	adaptInfoB[14].duty = 9;

	adaptInfoB[15].timRec = 0;
	adaptInfoB[15].duty = 9;

	adaptInfoB[16].timRec = 0;
	adaptInfoB[16].duty = 9;

	adaptInfoB[17].timRec = 0;
	adaptInfoB[17].duty = 8;

	adaptInfoB[18].timRec = 0;
	adaptInfoB[18].duty = 4;

	adaptInfoB[19].timRec = 0;
	adaptInfoB[19].duty = 2;
	*/
	
	adaptInfo[0].duty = 10;
	adaptInfo[1].duty = 10;
	adaptInfo[2].duty = 9;
	adaptInfo[3].duty = 9;
	adaptInfo[4].duty = 8;
	adaptInfo[5].duty = 8;
	adaptInfo[6].duty = 7;
	adaptInfo[7].duty = 7;
	adaptInfo[8].duty = 6;
	adaptInfo[9].duty = 6;
	adaptInfo[10].duty = 5;
	adaptInfo[11].duty = 5;
	adaptInfo[12].duty = 4;
	adaptInfo[13].duty = 4;
	adaptInfo[14].duty = 3;
	adaptInfo[15].duty = 3;
	adaptInfo[16].duty = 2;
	adaptInfo[17].duty = 2;
	adaptInfo[18].duty = 1;
	adaptInfo[19].duty = 1;
	
	
	adaptInfoB[0].duty = 10;
	adaptInfoB[1].duty = 10;
	adaptInfoB[2].duty = 10;
	adaptInfoB[3].duty = 10;
	adaptInfoB[4].duty = 10;
	adaptInfoB[5].duty = 10;
	adaptInfoB[6].duty = 10;
	adaptInfoB[7].duty = 10;
	adaptInfoB[8].duty = 10;
	adaptInfoB[9].duty = 5;
	adaptInfoB[10].duty = 5;
	adaptInfoB[11].duty = 5;
	adaptInfoB[12].duty = 5;
	adaptInfoB[13].duty = 5;
	adaptInfoB[14].duty = 1;
	adaptInfoB[15].duty = 1;
	adaptInfoB[16].duty = 1;
	adaptInfoB[17].duty = 1;
	adaptInfoB[18].duty = 1;
	adaptInfoB[19].duty = 1;

	for(cir = 0; cir < MAX_DAMP_ADAPT_NUM; cir++)
	{
		dampAdapetInfo[cir].duty = 0;
		dampAdapetInfo[cir].goodDuty = 0;
		dampAdapetInfo[cir].lock = 0;
		dampAdapetInfo[cir].result = Good;
		
		dampAdapetInfoB[cir].duty = 0;
		dampAdapetInfoB[cir].goodDuty = 0;
		dampAdapetInfoB[cir].lock = 0;
		dampAdapetInfoB[cir].result = Good;
	}

	dampAdapetInfo[Agv_MS_Right_1 - Agv_MS_Right_1].duty = 2;
	dampAdapetInfo[Agv_MS_Right_1_5 - Agv_MS_Right_1].duty = 3;
	dampAdapetInfo[Agv_MS_Right_2 - Agv_MS_Right_1].duty = 4;
	dampAdapetInfo[Agv_MS_Right_2_5 - Agv_MS_Right_1].duty = 7;
	dampAdapetInfo[Agv_MS_Right_3 - Agv_MS_Right_1].duty = 10;
	dampAdapetInfo[Agv_MS_Right_3_5 - Agv_MS_Right_1].duty = 12;
	dampAdapetInfo[Agv_MS_Right_4 - Agv_MS_Right_1].duty = 15;
	dampAdapetInfo[Agv_MS_Right_4_5 - Agv_MS_Right_1].duty = 18;
	dampAdapetInfo[Agv_MS_Right_5 - Agv_MS_Right_1].duty = 20;
	dampAdapetInfo[Agv_MS_Right_6 - Agv_MS_Right_1].duty = 23;
	dampAdapetInfo[Agv_MS_Right_7 - Agv_MS_Right_1].duty = 25;
	dampAdapetInfo[Agv_MS_Right_8 - Agv_MS_Right_1].duty = 25;
	dampAdapetInfo[Agv_MS_Right_9 - Agv_MS_Right_1].duty = 26;
	dampAdapetInfo[Agv_MS_Right_10 - Agv_MS_Right_1].duty = 26;

	
	dampAdapetInfoB[Agv_MS_Right_1 - Agv_MS_Right_1].duty = 2;
	dampAdapetInfoB[Agv_MS_Right_1_5 - Agv_MS_Right_1].duty = 3;
	dampAdapetInfoB[Agv_MS_Right_2 - Agv_MS_Right_1].duty = 4;
	dampAdapetInfoB[Agv_MS_Right_2_5 - Agv_MS_Right_1].duty = 7;
	dampAdapetInfoB[Agv_MS_Right_3 - Agv_MS_Right_1].duty = 10;
	dampAdapetInfoB[Agv_MS_Right_3_5 - Agv_MS_Right_1].duty = 12;
	dampAdapetInfoB[Agv_MS_Right_4 - Agv_MS_Right_1].duty = 15;
	dampAdapetInfoB[Agv_MS_Right_4_5 - Agv_MS_Right_1].duty = 18;
	dampAdapetInfoB[Agv_MS_Right_5 - Agv_MS_Right_1].duty = 20;
	dampAdapetInfoB[Agv_MS_Right_6 - Agv_MS_Right_1].duty = 23;
	dampAdapetInfoB[Agv_MS_Right_7 - Agv_MS_Right_1].duty = 25;
	dampAdapetInfoB[Agv_MS_Right_8 - Agv_MS_Right_1].duty = 25;
	dampAdapetInfoB[Agv_MS_Right_9 - Agv_MS_Right_1].duty = 26;
	dampAdapetInfoB[Agv_MS_Right_10 - Agv_MS_Right_1].duty = 26;


	
	for(cir = 0; cir < MAX_DAMP_ADAPT_NUM; cir++)
	{
		adaptInfo2[cir].dt_head = 0;
		adaptInfo2[cir].dutyr = 0;
		adaptInfo2[cir].lock = 0;
		adaptInfo2[cir].result = Good;
		
		
		adaptInfoB2[cir].dt_head = 0;
		adaptInfoB2[cir].dutyr = 0;
		adaptInfoB2[cir].lock = 0;
		adaptInfoB2[cir].result = Good;

		for(cir2 = 0; cir2 < MAX_DAMP_ADAPT_NUM; cir2++)
		{
			adaptInfo2[cir].dt[cir2].duty = 0;
			adaptInfo2[cir].dt[cir2].ms1 = AgvInits;
			adaptInfo2[cir].dt[cir2].ms2 = AgvInits;
			adaptInfo2[cir].dt[cir2].ms3 = AgvInits;
			adaptInfo2[cir].dt[cir2].ms4 = AgvInits;
			adaptInfo2[cir].dt[cir2].ms5 = AgvInits;
			adaptInfo2[cir].dt[cir2].msabs = 0;

			adaptInfoB2[cir].dt[cir2].duty = 0;
			adaptInfoB2[cir].dt[cir2].ms1 = AgvInits;
			adaptInfoB2[cir].dt[cir2].ms2 = AgvInits;
			adaptInfoB2[cir].dt[cir2].ms3 = AgvInits;
			adaptInfoB2[cir].dt[cir2].ms4 = AgvInits;
			adaptInfoB2[cir].dt[cir2].ms5 = AgvInits;
			adaptInfoB2[cir].dt[cir2].msabs = 0;
		}



		mpu6050DS_ptr->ax = 0;
		mpu6050DS_ptr->ay = 0;
		mpu6050DS_ptr->ayAverage = 0;
		mpu6050DS_ptr->ayMax = 0;
		mpu6050DS_ptr->ayMid = 0;
		mpu6050DS_ptr->ayMin = S16_MAX;
		mpu6050DS_ptr->ayOffset = 0;
		mpu6050DS_ptr->az = 0;
		mpu6050DS_ptr->gx = 0;
		mpu6050DS_ptr->gy = 0;
		mpu6050DS_ptr->gz = 0;
		mpu6050DS_ptr->minusCount = 0;
		mpu6050DS_ptr->plusCount = 0;
		mpu6050DS_ptr->sum = 0;
		mpu6050DS_ptr->sumCount = 0;
		mpu6050DS_ptr->sumMinus = 0;
		mpu6050DS_ptr->sumPlus = 0;
		mpu6050DS_ptr->tempture = 0;
		mpu6050DS_ptr->ayAverageUpdate = 0;

		WarningLedCtrlPtr->twinkleFlag = 0;
		WarningLedCtrlPtr->intervalTime_ms = 100;
		WarningLedCtrlPtr->twinkleNum = 2;
		WarningLedCtrlPtr->twinkleCtrlFunc = WarningLedTwinkleCtrl;

		BuzzerCtrlPtr->buzzerFlag = 0;
		BuzzerCtrlPtr->buzzerTime_ms = 100;
		BuzzerCtrlPtr->buzzerNum = 2;
		BuzzerCtrlPtr->buzzerCtrlFunc = BuzzerCtrlFunc;
	}
	
}



