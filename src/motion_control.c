#include "motion_control.h"
#include "cfg_gpio.h"
#include "timer_opts.h"
#include "pwm_opts.h"
#include "magn_sensor.h"
#include "zigbee.h"

#define ABSOLU(value)	(value >= 0 ? value : (-value))

TimRec rec[30];
u8 recH = 0;

HallCount HallCountArr[6];
HallCount CrossRoadHallCountArrGS[6];
HallCount CrossRoadHallCountArrGB[6];


u16 ZBandRFIDmapping[11];

u8 AgvGear[MAX_GEAR_NUM] = {0, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
u8 AgvGearCompDutyLF[MAX_GEAR_NUM] = {0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
u8 AgvGearCompDutyRF[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 AgvGearCompDutyLB[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 AgvGearCompDutyRB[MAX_GEAR_NUM] = {0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
u8 AgvGearK[MAX_GEAR_NUM] = {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 16, 17, 18, 19, 20, 21, 22};
u8 AgvGear7CDLF[MAX_GEAR_OFFSET] = {0, 2, 7, 8, 10, 12, 14, 16, 18, 20, 20};
T1_AutoAdapt_Info adaptInfo[20];
T1_AutoAdapt_Info adaptInfoB[20];


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


ControlerParaStruct ctrlParas;
ControlerParaStruct_P ctrlParasPtr = &ctrlParas;
MotionOperaterStruct motionOpts;
MotionOperaterStruct_P motionOptsPtr = &motionOpts;

u32 responseTime = 0;

void (*agv_walking_func[5]) (u8);

#define MOTOR_RIGHT_DUTY_SET(speed)			(pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, speed))
#define MOTOR_LEFT_DUTY_SET(speed)			(pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, speed))

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
void AGV_Brake(void)
{
	MOTOR_RIGHT_BK = 0;
	MOTOR_LEFT_BK = 0;
}

void AGV_Stop(void)
{	
	Motor_Right_CR(0);
	Motor_Left_CCR(0);

	MOTOR_RIGHT_FR = 1;
	MOTOR_LEFT_FR = 1;
	MOTOR_RIGHT_EN = 1;
	MOTOR_LEFT_EN = 1;
}

void AGV_Go_Straight_Start(u8 speed)
{
	ctrlParasPtr->rightMotorSettedSpeed = speed;
	ctrlParasPtr->leftMotorSettedSpeed = speed;
	
	Motor_Right_CR(speed);
	Motor_Left_CCR(speed);
	
	ctrlParasPtr->agvStatus = goStraightStatus;
}

void AGV_Go_Straight_Stop(void)
{
	Motor_Right_CR(0);
	Motor_Left_CCR(0);
}

void AGV_Go_Straight_Distance(void)
{
	
}

void AGV_Back_Start(u8 speed)
{
	Motor_Right_CCR(0);
	Motor_Left_CR(0);
}

void AGV_Back_Stop(void)
{
	Motor_Right_CCR(0);
	Motor_Left_CR(0);
}

void AGV_Back_Distance(void)
{
	
}

void AGV_Turn_Left_Start(u8 speed)
{
	ctrlParasPtr->rightMotorSettedSpeed += speed;

	Motor_Right_CR(ctrlParasPtr->rightMotorSettedSpeed);
	Motor_Left_CCR(ctrlParasPtr->leftMotorSettedSpeed);
}

void AGV_Turn_Left_Stop(void)
{
	
}

void AGV_Turn_Left_Angle(void)
{
	
}

void AGV_Turn_Right_Start(void)
{
	
}

void AGV_Turn_Right_Stop(void)
{
	
}

void AGV_Turn_Right_Angle(void)
{
	
}


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


void motion_up_pwm(void)
{
	printf("ctrlParasPtr->agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
	if(stopStatus == ctrlParasPtr->agvStatus)
	{
		printf("up_goStraightStatus\r\n");
		
		ctrlParasPtr->agvStatus = goStraightStatus;
		
		if(ctrlParasPtr->settedSpeed <= MAX_SPEED_LIMIT)
		{
			ctrlParasPtr->settedSpeed += MAX_STEP_SPEED_INC;
		}
		
		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CCR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		printf("up_goStraightStatus\r\n");
		if(ctrlParasPtr->settedSpeed <= MAX_SPEED_LIMIT)
		{
			ctrlParasPtr->settedSpeed += MAX_STEP_SPEED_INC;
		}
		
		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CCR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		printf("up_stopStatus\r\n");
		if(ctrlParasPtr->settedSpeed <= MAX_STEP_SPEED_INC)
		{
			ctrlParasPtr->settedSpeed = 0;
			ctrlParasPtr->agvStatus = stopStatus;
		}
		else
		{
			ctrlParasPtr->settedSpeed -= MAX_STEP_SPEED_INC;
		}

		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CCR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	
	
}

void motion_down_pwm(void)
{
	printf("ctrlParasPtr->agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
	if(stopStatus == ctrlParasPtr->agvStatus)
	{
		printf("down_backStatus\r\n");
		ctrlParasPtr->agvStatus = backStatus;

		if(ctrlParasPtr->settedSpeed <= MAX_SPEED_LIMIT)
		{
			ctrlParasPtr->settedSpeed += MAX_STEP_SPEED_INC;
		}

		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CCR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		printf("down_backStatus\r\n");
		if(ctrlParasPtr->settedSpeed <= MAX_SPEED_LIMIT)
		{
			ctrlParasPtr->settedSpeed += MAX_STEP_SPEED_INC;
		}
		
		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CCR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		printf("down_stopStatus\r\n");
		if(ctrlParasPtr->settedSpeed <= MAX_STEP_SPEED_INC)
		{
			ctrlParasPtr->settedSpeed = 0;
			ctrlParasPtr->agvStatus = stopStatus;
		}
		else
		{
			ctrlParasPtr->settedSpeed -= MAX_STEP_SPEED_INC;
		}

		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CCR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	
}

void motion_left_pwm(void)
{
	printf("ctrlParasPtr->agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
	if((stopStatus == ctrlParasPtr->agvStatus) || (cirLeft == ctrlParasPtr->agvStatus))
	{
		printf("left_cirleft\r\n");
		ctrlParasPtr->agvStatus = cirLeft;
		if(ctrlParasPtr->settedSpeed <= MAX_SPEED_LIMIT)
		{
			ctrlParasPtr->settedSpeed += MAX_STEP_SPEED_INC;
		}

		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		printf("left_turnleft_go\r\n");
		if(ctrlParasPtr->leftMotorSettedSpeed <= ctrlParasPtr->rightMotorSettedSpeed)
		{
			ctrlParasPtr->leftMotorSettedSpeed--;
		}
		else if(ctrlParasPtr->leftMotorSettedSpeed > ctrlParasPtr->rightMotorSettedSpeed)
		{
			ctrlParasPtr->rightMotorSettedSpeed++;
		}
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CCR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		printf("left_turnleft_back\r\n");
		if(ctrlParasPtr->leftMotorSettedSpeed <= ctrlParasPtr->rightMotorSettedSpeed)
		{
			ctrlParasPtr->leftMotorSettedSpeed--;
		}
		else if(ctrlParasPtr->leftMotorSettedSpeed > ctrlParasPtr->rightMotorSettedSpeed)
		{
			ctrlParasPtr->rightMotorSettedSpeed++;
		}
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CCR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(cirRight == ctrlParasPtr->agvStatus)
	{
		if(ctrlParasPtr->settedSpeed >= MAX_STEP_SPEED_INC)
		{
			ctrlParasPtr->settedSpeed -= MAX_STEP_SPEED_INC;
		}

		if(0 == ctrlParasPtr->settedSpeed)
		{
			ctrlParasPtr->agvStatus = stopStatus;
		}
		
		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CCR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CCR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
}

void motion_right_pwm(void)
{
	printf("ctrlParasPtr->agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
	if((stopStatus == ctrlParasPtr->agvStatus) || (cirRight == ctrlParasPtr->agvStatus))
	{
		printf("left_cirright\r\n");
		ctrlParasPtr->agvStatus = cirRight;
		if(ctrlParasPtr->settedSpeed <= MAX_SPEED_LIMIT)
		{
			ctrlParasPtr->settedSpeed += MAX_STEP_SPEED_INC;
		}

		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CCR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CCR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		printf("left_turnright_go\r\n");
		if(ctrlParasPtr->rightMotorSettedSpeed <= ctrlParasPtr->leftMotorSettedSpeed)
		{
			ctrlParasPtr->rightMotorSettedSpeed--;
		}
		else if(ctrlParasPtr->rightMotorSettedSpeed > ctrlParasPtr->leftMotorSettedSpeed)
		{
			ctrlParasPtr->leftMotorSettedSpeed++;
		}
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CCR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		printf("left_turnright_back\r\n");
		if(ctrlParasPtr->rightMotorSettedSpeed <= ctrlParasPtr->leftMotorSettedSpeed)
		{
			ctrlParasPtr->rightMotorSettedSpeed--;
		}
		else if(ctrlParasPtr->rightMotorSettedSpeed > ctrlParasPtr->leftMotorSettedSpeed)
		{
			ctrlParasPtr->leftMotorSettedSpeed++;
		}
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CCR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
	else if(cirLeft == ctrlParasPtr->agvStatus)
	{
		if(ctrlParasPtr->settedSpeed >= MAX_STEP_SPEED_INC)
		{
			ctrlParasPtr->settedSpeed -= MAX_STEP_SPEED_INC;
		}

		if(0 == ctrlParasPtr->settedSpeed)
		{
			ctrlParasPtr->agvStatus = stopStatus;
		}
		
		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
		
		ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

		MOTOR_RIGHT_CR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
		MOTOR_LEFT_CR_DEF(ctrlParasPtr->leftMotorSettedSpeed);
		//Mode_Pin_Ctrl(ctrlParasPtr);
	}
}

void motion_stop_pwm(void)
{
	//printf("ctrlParasPtr->agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
	static u8 flag = 0x01;
	
	ctrlParasPtr->settedSpeed = 0;

	#if 1
	if(flag)
	{
		flag = 0x00;
		
		printf("leftMotorSettedSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
		printf("rightMotorSettedSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);

		
	}
	#endif
	
	ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	
	ctrlParasPtr->leftMotorRealSpeed = ctrlParasPtr->leftMotorSettedSpeed;
	ctrlParasPtr->rightMotorRealSpeed = ctrlParasPtr->rightMotorSettedSpeed;

	MOTOR_RIGHT_CCR_DEF(ctrlParasPtr->rightMotorSettedSpeed);
	MOTOR_LEFT_CCR_DEF(ctrlParasPtr->leftMotorSettedSpeed);

	CHANGE_TO_STOP_MODE();
}


void x3x2x1_right_setting(void)
{
	if(ctrlParasPtr->speedModeValue_Right <= 0x07)
	{
		//printf("speedModeValue_Right = %x\r\n", ctrlParasPtr->speedModeValue_Right);
		MOTOR_RIGHT_X1 = ctrlParasPtr->speedModeValue_Right & 0x01;
		//printf("MOTOR_RIGHT_X1 = %x\r\n", ctrlParasPtr->speedModeValue_Right & 0x01);
		MOTOR_RIGHT_X2 = (ctrlParasPtr->speedModeValue_Right >> 1) & 0x01;
		//printf("MOTOR_RIGHT_X2 = %x\r\n", (ctrlParasPtr->speedModeValue_Right >> 1) & 0x01);
		MOTOR_RIGHT_X3 = (ctrlParasPtr->speedModeValue_Right >> 2) & 0x01;
		//printf("MOTOR_RIGHT_X3 = %x\r\n", (ctrlParasPtr->speedModeValue_Right >> 2) & 0x01);
	}
}

void x3x2x1_left_setting(void)
{
	if(ctrlParasPtr->speedModeValue_Left <= 0x07)
	{
		//printf("speedModeValue_Left = %x\r\n", ctrlParasPtr->speedModeValue_Left);
		MOTOR_LEFT_X1 = ctrlParasPtr->speedModeValue_Left & 0x01;	
		//printf("MOTOR_LEFT_X1 = %x\r\n", ctrlParasPtr->speedModeValue_Left & 0x01);
		MOTOR_LEFT_X2 = (ctrlParasPtr->speedModeValue_Left >> 1) & 0x01;
		//printf("MOTOR_LEFT_X2 = %x\r\n", (ctrlParasPtr->speedModeValue_Left >> 1) & 0x01);
		MOTOR_LEFT_X3 = (ctrlParasPtr->speedModeValue_Left >> 2) & 0x01;
		//printf("MOTOR_LEFT_X3 = %x\r\n", (ctrlParasPtr->speedModeValue_Left >> 2) & 0x01);
	}
}

void motion_up_x1x2x3_I(void)
{	
	printf("ctrlParasPtr->agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
	if((stopStatus == ctrlParasPtr->agvStatus) || (goStraightStatus == ctrlParasPtr->agvStatus))
	{
		printf("down_backStatus\r\n");
		if(stopStatus == ctrlParasPtr->agvStatus)
		{
			ctrlParasPtr->agvStatus = goStraightStatus;
		}
		else
		{
			if(ctrlParasPtr->speedModeValue_Right <= X3X2X1_MAX_SPEED_LIMIT)
			{
				ctrlParasPtr->speedModeValue_Right += 1;
			}

			if(ctrlParasPtr->speedModeValue_Left <= X3X2X1_MAX_SPEED_LIMIT)
			{
				ctrlParasPtr->speedModeValue_Left += 1;
			}
		}

		MOTOR_RIGHT_CR_PIN_SET();
		MOTOR_LEFT_CR_PIN_SET();
		
		x3x2x1_right_setting();
		x3x2x1_left_setting();
	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		printf("up_stopStatus\r\n");
		if(ctrlParasPtr->speedModeValue_Right <= 0x01)
		{
			ctrlParasPtr->speedModeValue_Right = 0;
			ctrlParasPtr->agvStatus = stopStatus;
			MOTOR_RIGHT_STOP_PIN_SET();
		}
		else
		{
			ctrlParasPtr->speedModeValue_Right -= 1;
			MOTOR_RIGHT_CCR_PIN_SET();
		}

		if(ctrlParasPtr->speedModeValue_Left <= 0x01)
		{
			ctrlParasPtr->speedModeValue_Left = 0;
			ctrlParasPtr->agvStatus = stopStatus;
			MOTOR_LEFT_STOP_PIN_SET();
		}
		else
		{
			ctrlParasPtr->speedModeValue_Left -= 1;
			MOTOR_LEFT_CCR_PIN_SET();
		}
		
		x3x2x1_right_setting();
		x3x2x1_left_setting();
	}
	
}

void motion_down_x1x2x3_I(void)
{
	printf("ctrlParasPtr->agvStatus = %d\r\n", ctrlParasPtr->agvStatus);
	if((stopStatus == ctrlParasPtr->agvStatus) || (backStatus == ctrlParasPtr->agvStatus))
	{
		printf("down_backStatus\r\n");
		if(stopStatus == ctrlParasPtr->agvStatus)
		{
			ctrlParasPtr->agvStatus = backStatus;
		}
		else
		{
			if(ctrlParasPtr->speedModeValue_Right <= X3X2X1_MAX_SPEED_LIMIT)
			{
				ctrlParasPtr->speedModeValue_Right += 1;
			}

			if(ctrlParasPtr->speedModeValue_Left <= X3X2X1_MAX_SPEED_LIMIT)
			{
				ctrlParasPtr->speedModeValue_Left += 1;
			}
		}
		
		MOTOR_RIGHT_CCR_PIN_SET();
		MOTOR_LEFT_CCR_PIN_SET();
		
		x3x2x1_right_setting();
		x3x2x1_left_setting();
	}
	else if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		printf("down_stopStatus\r\n");
		if(ctrlParasPtr->speedModeValue_Right <= 0x00)
		{
			ctrlParasPtr->speedModeValue_Right = 0;
			ctrlParasPtr->agvStatus = stopStatus;
			MOTOR_RIGHT_STOP_PIN_SET();
		}
		else
		{
			ctrlParasPtr->speedModeValue_Right -= 1;
			MOTOR_RIGHT_CCR_PIN_SET();
		}

		if(ctrlParasPtr->speedModeValue_Left <= 0x00)
		{
			ctrlParasPtr->speedModeValue_Left = 0;
			ctrlParasPtr->agvStatus = stopStatus;
			MOTOR_LEFT_STOP_PIN_SET();
		}
		else
		{
			ctrlParasPtr->speedModeValue_Left -= 1;
			MOTOR_LEFT_CCR_PIN_SET();
		}
		
		x3x2x1_right_setting();
		x3x2x1_left_setting();
	}
}

void motion_left_x1x2x3_I(void)
{
	
}

void motion_right_x1x2x3_I(void)
{
	
}

void motion_stop_x1x2x3_I(void)
{
	ctrlParasPtr->agvStatus = stopStatus;
	MOTOR_RIGHT_STOP_PIN_SET();
	MOTOR_LEFT_STOP_PIN_SET();
	ctrlParasPtr->speedModeValue_Right = 0x00;
	ctrlParasPtr->speedModeValue_Left = 0x00;
	x3x2x1_right_setting();
	x3x2x1_left_setting();
}




void motion_up(void)
{
	switch(ctrlParasPtr->speedMode)
	{
		case PWM_MODE:
			motion_up_pwm();
			break;

		case X1X2X3_I_MODE:
			motion_up_x1x2x3_I();
			break;

		case X1X2X3_II_MODE:
			break;

		case INTER_MODE:
			break;

		default:
			break;
	}
}

void motion_down(void)
{
	switch(ctrlParasPtr->speedMode)
	{
		case PWM_MODE:
			motion_down_pwm();
			break;

		case X1X2X3_I_MODE:
			motion_down_x1x2x3_I();
			break;

		case X1X2X3_II_MODE:
			break;

		case INTER_MODE:
			break;

		default:
			break;
	}
}

void motion_left(void)
{
	switch(ctrlParasPtr->speedMode)
	{
		case PWM_MODE:
			motion_left_pwm();
			break;

		case X1X2X3_I_MODE:
			motion_left_x1x2x3_I();
			break;

		case X1X2X3_II_MODE:
			break;

		case INTER_MODE:
			break;

		default:
			break;
	}
}

void motion_right(void)
{
	switch(ctrlParasPtr->speedMode)
	{
		case PWM_MODE:
			motion_right_pwm();
			break;

		case X1X2X3_I_MODE:
			motion_right_x1x2x3_I();
			break;

		case X1X2X3_II_MODE:
			break;

		case INTER_MODE:
			break;

		default:
			break;
	}
}

void motion_stop(void)
{
	switch(ctrlParasPtr->speedMode)
	{
		case PWM_MODE:
			motion_stop_pwm();
			break;

		case X1X2X3_I_MODE:
			motion_stop_x1x2x3_I();
			break;

		case X1X2X3_II_MODE:
			break;

		case INTER_MODE:
			break;

		default:
			break;
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

void motor_left_duty_offset_gS(u8 duty)
{
	u8 tempDuty = ctrlParasPtr->settedSpeed + duty;
	
	if(tempDuty <= 100)
	{
		ctrlParasPtr->leftMotorSpeedOffset = duty;
		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed + ctrlParasPtr->leftMotorSpeedOffset;
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
	}
	else
	{
		/*
		ctrlParasPtr->leftMotorSettedSpeed = 100;
		ctrlParasPtr->rightMotorSettedSpeed -= (tempDuty - 100);
		
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
		*/
	}
	
}

void motor_right_duty_offset_gS(u8 duty)
{
	u8 tempDuty = ctrlParasPtr->settedSpeed + duty;
	
	if(tempDuty <= 100)
	{
		ctrlParasPtr->rightMotorSpeedOffset = duty;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed + ctrlParasPtr->rightMotorSpeedOffset;
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
	}
	else
	{
		/*
		ctrlParasPtr->rightMotorSettedSpeed = 100;
		ctrlParasPtr->leftMotorSettedSpeed -= (tempDuty - 100);
		
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
		*/
	}
	
}

void motor_left_duty_offset_back(u8 duty)
{
	u8 tempDuty = ctrlParasPtr->settedSpeed + duty;
	
	if(tempDuty <= 100)
	{
		ctrlParasPtr->leftMotorSpeedOffset = duty;
		ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed + ctrlParasPtr->leftMotorSpeedOffset;
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
	}
	else
	{
		/*
		ctrlParasPtr->leftMotorSettedSpeed = 100;
		ctrlParasPtr->rightMotorSettedSpeed -= (tempDuty - 100);
		
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
		*/
	}
	
}

void motor_right_duty_offset_back(u8 duty)
{
	u8 tempDuty = ctrlParasPtr->settedSpeed + duty;
	
	if(tempDuty <= 100)
	{
		ctrlParasPtr->rightMotorSpeedOffset = duty;
		ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed + ctrlParasPtr->rightMotorSpeedOffset;
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
	}
	else
	{
		/*
		ctrlParasPtr->rightMotorSettedSpeed = 100;
		ctrlParasPtr->leftMotorSettedSpeed -= (tempDuty - 100);
		
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
		*/
	}
	
}


void motor_left_duty_set(u8 duty)
{
	
	if(duty <= 100)
	{
		ctrlParasPtr->leftMotorSettedSpeed = duty;
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
	}
	else
	{
		ctrlParasPtr->leftMotorSettedSpeed = 100;
		ctrlParasPtr->rightMotorSettedSpeed -= (duty - 100);
		
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
	}
}

void motor_right_duty_set(u8 duty)
{	
	if(duty <= 100)
	{
		ctrlParasPtr->rightMotorSettedSpeed = duty;
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
	}
	else
	{
		ctrlParasPtr->rightMotorSettedSpeed = 100;
		ctrlParasPtr->leftMotorSettedSpeed -= (duty - 100);
		
		MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
		MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
	}
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
	
	

	/*
	ctrlParasPtr->leftMotorSpeedOffset_p = &(ctrlParasPtr->rightMotorSpeedOffset);
	ctrlParasPtr->rightMotorSpeedOffset_p = &(ctrlParasPtr->leftMotorSpeedOffset);
	ctrlParasPtr->leftMotorSettedSpeed_p = &(ctrlParasPtr->rightMotorSettedSpeed);
	ctrlParasPtr->rightMotorSettedSpeed_p = &(ctrlParasPtr->leftMotorSettedSpeed);
	motionOptsPtr->motor_left_duty_offset = motor_right_duty_offset;
	motionOptsPtr->motor_right_duty_offset = motor_left_duty_offset;
	*/
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

void LeftGearSpeed(u8 gearNum, u8 offset)
{
	ctrlParasPtr->leftMotorSpeedOffset = offset;
	ctrlParasPtr->leftMotorSettedSpeed = AgvGear[gearNum] + FLeftCompDuty[gearNum] + ctrlParasPtr->leftMotorSpeedOffset;

	 if(ctrlParasPtr->leftMotorSettedSpeed > 100)
	 {
		 ctrlParasPtr->leftMotorSettedSpeed = 100;
	 }

	 MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
}

void RightGearSpeed(u8 gearNum, u8 offset)
{
	ctrlParasPtr->rightMotorSpeedOffset = offset;
	ctrlParasPtr->rightMotorSettedSpeed = AgvGear[gearNum] + FRightCompDuty[gearNum] + ctrlParasPtr->rightMotorSpeedOffset;

	 if(ctrlParasPtr->rightMotorSettedSpeed > 100)
	 {
		 ctrlParasPtr->rightMotorSettedSpeed = 100;
	 }

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



void debugInfoShow(char *flag,u32 debugCode)
{
	#if 0
	static u32 codeRecod = 0;
	
	if(codeRecod != debugCode)
	{
		MSD_Show_Bin(FMSDS_Ptr->MSD_Hex);
		printf("\t");
		codeRecod = debugCode;
		printf("%s_%d\r\n", flag, debugCode);
	}
	#endif
}

#if 0
void walking_goStraight(void)
{
	static u32 counter = 0;
	
	ctrlParasPtr->comflag = 6;
	//printf("AgvMSLocation = %d\r\n", FMSDS_Ptr->AgvMSLocation);
	
	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		ctrlParasPtr->comflag = 11;
		
		//if(Agv_MS_Center == RMSDS_Ptr->AgvMSLocation)
		if(((RMSDS_Ptr->AgvMSLocation > Agv_MS_Left_Begin) && (RMSDS_Ptr->AgvMSLocation < Agv_MS_Left_2)) || ((RMSDS_Ptr->AgvMSLocation > Agv_MS_Right_Begin) && (RMSDS_Ptr->AgvMSLocation < Agv_MS_Right_2)))
		{
			counter++;
			if(counter >= 500)
			{
				counter = 0;
				// 加速操作
				if(ctrlParasPtr->settedSpeed < 20)
				{
					ctrlParasPtr->settedSpeed += 2;
					motor_left_duty_offset_gS(0);
					motor_right_duty_offset_gS(0);
					printf("speed add\r\n");
				}
			}
		}
		
	}
	else
	{
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_Begin) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_End))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 12;
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_End))
			{
				ctrlParasPtr->settedSpeed = 10;
			}
			
			//printf("AgvMSLocation = %d\r\n", FMSDS_Ptr->AgvMSLocation);
			//ctrlParasPtr->leftMotorSpeedOffset =  DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Left_1];
			//ctrlParasPtr->rightMotorSpeedOffset = 0;
			
			motor_left_duty_offset_gS(DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Left_1]);
			motor_right_duty_offset_gS(0);
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_Begin) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 13;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->settedSpeed = 10;
			}
			
			//ctrlParasPtr->rightMotorSpeedOffset = DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
			//ctrlParasPtr->leftMotorSpeedOffset = 0;
			
			motor_left_duty_offset_gS(0);
			motor_right_duty_offset_gS(DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1]);
			
		}
		else if(Agv_MS_Right_Outside == FMSDS_Ptr->AgvMSLocation)
		{

			ctrlParasPtr->comflag = 14;
			ctrlParasPtr->settedSpeed = 10;
			//ctrlParasPtr->rightMotorSpeedOffset = DutyTable[Agv_MS_Right_8 - Agv_MS_Right_1];
			//ctrlParasPtr->rightMotorSpeedOffset = 0;
			//ctrlParasPtr->leftMotorSpeedOffset = 0;
		
			motor_left_duty_offset_gS(0);
			motor_right_duty_offset_gS(0);

		}
		else if(Agv_MS_Left_Outside == FMSDS_Ptr->AgvMSLocation)
		{	
			ctrlParasPtr->comflag = 15;
			ctrlParasPtr->settedSpeed = 10;
			//ctrlParasPtr->leftMotorSpeedOffset =  DutyTable[Agv_MS_Left_8 - Agv_MS_Left_1];
			//ctrlParasPtr->rightMotorSpeedOffset = 0;
			ctrlParasPtr->settedSpeed = 10;
			motor_left_duty_offset_gS(0);
			motor_right_duty_offset_gS(0);
			
		}
		else
		{
			ctrlParasPtr->comflag = 16;
			//ctrlParasPtr->leftMotorSpeedOffset =  0;
			//ctrlParasPtr->rightMotorSpeedOffset = 0;
			motor_left_duty_offset_gS(0);
			motor_right_duty_offset_gS(0);
			
		}
	}
	
}
#else

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

#endif

#if 0
void walking_backStatus(void)
{
	static u32 counter = 0;
	
	ctrlParasPtr->comflag = 6;
	//printf("AgvMSLocation = %d\r\n", FMSDS_Ptr->AgvMSLocation);
	
	if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		ctrlParasPtr->comflag = 21;
		
		//if(Agv_MS_Center == RMSDS_Ptr->AgvMSLocation)
		if(((RMSDS_Ptr->AgvMSLocation > Agv_MS_Left_2) && (RMSDS_Ptr->AgvMSLocation < Agv_MS_Left_2)) || ((RMSDS_Ptr->AgvMSLocation > Agv_MS_Right_Begin) && (RMSDS_Ptr->AgvMSLocation < Agv_MS_Right_2)))
		{
			counter++;
			if(counter >= 500)
			{
				counter = 0;
				// 加速操作
				if(ctrlParasPtr->settedSpeed < 25)
				{
					ctrlParasPtr->settedSpeed += 2;
					motor_left_duty_offset_gS(0);
					motor_right_duty_offset_gS(0);
					printf("speed add b\r\n");
				}
			}
		}
		
	}
	else
	{
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_Begin) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_End))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 22;
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_End))
			{
				ctrlParasPtr->settedSpeed = 10;
			}
			
			//printf("AgvMSLocation = %d\r\n", FMSDS_Ptr->AgvMSLocation);
			//ctrlParasPtr->leftMotorSpeedOffset =	DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Left_1];
			//ctrlParasPtr->rightMotorSpeedOffset = 0;
			
			motor_left_duty_offset_back(DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Left_1]);
			motor_right_duty_offset_back(0);
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_Begin) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 23;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->settedSpeed = 10;
			}
			
			//ctrlParasPtr->rightMotorSpeedOffset = DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
			//ctrlParasPtr->leftMotorSpeedOffset = 0;
			
			motor_left_duty_offset_back(0);
			motor_right_duty_offset_back(DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1]);
			
		}
		else if(Agv_MS_Right_Outside == FMSDS_Ptr->AgvMSLocation)
		{

			ctrlParasPtr->comflag = 24;
			ctrlParasPtr->settedSpeed = 10;
			//ctrlParasPtr->rightMotorSpeedOffset = DutyTable[Agv_MS_Right_8 - Agv_MS_Right_1];
			//ctrlParasPtr->rightMotorSpeedOffset = 0;
			//ctrlParasPtr->leftMotorSpeedOffset = 0;
		
			motor_left_duty_offset_back(0);
			motor_right_duty_offset_back(0);

		}
		else if(Agv_MS_Left_Outside == FMSDS_Ptr->AgvMSLocation)
		{	
			ctrlParasPtr->comflag = 25;
			ctrlParasPtr->settedSpeed = 10;
			//ctrlParasPtr->leftMotorSpeedOffset =	DutyTable[Agv_MS_Left_8 - Agv_MS_Left_1];
			//ctrlParasPtr->leftMotorSpeedOffset =	0;
			//ctrlParasPtr->rightMotorSpeedOffset = 0;

			motor_left_duty_offset_back(0);
			motor_right_duty_offset_back(0);
			
		}
		else
		{
			ctrlParasPtr->comflag = 26;
			
			//ctrlParasPtr->leftMotorSpeedOffset =	0;
			//ctrlParasPtr->rightMotorSpeedOffset = 0;

			motor_left_duty_offset_back(0);
			motor_right_duty_offset_back(0);
		}
	}
	
	
}

#else

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




#endif

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


/**********Motor Basic Control Mode: End****************/

void AGV_Correct_gS(u8 gear)
{
	static u32 counter = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 2, lreco = 0, rreco = 0;
	//static u8 loffset = 0, roffset = 5;
	
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
					
					lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
					rmSpeed = AgvGear[gearRecod] + FRG[0][gearRecod];
					
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
		
		
		if(lreco > rreco)
		{
			if(lreco - rreco >= 3)
			{
				if(FRG[0][gearRecod] > 0)
				{
					FRG[0][gearRecod]--;
				}
				else
				{
					FLG[0][gearRecod]++;
				}
				
				printf("loffset = %d, roffset = %d\r\n", FLG[0][gearRecod], FRG[0][gearRecod]);
				
				
				lreco = 0;
				rreco = 0;
			}
		}
		else if(lreco < rreco)
		{
			if(rreco - lreco >= 3)
			{
				if(FLG[0][gearRecod] > 0)
				{
					FLG[0][gearRecod]--;
				}
				else
				{
					FRG[0][gearRecod]++;
				}
				
				printf("loffset = %d, roffset = %d\r\n", FLG[0][gearRecod], FRG[0][gearRecod]);
				lreco = 0;
				rreco = 0;
			}
		}
		
		
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 12;

			if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
			{
				if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Center)
				{
					lreco++;
				}
				
			}
			
			/*
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3))
			{
				ctrlParasPtr->comflag = 121;

				lmSpeed = AgvGear[gearRecod];
				rmSpeed = AgvGear[gearRecod];
			}
			else
			{
				ctrlParasPtr->comflag = 122;
				lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + loffset;
				rmSpeed = AgvGear[gearRecod];
			}
			*/
			//printf("lreco = %d\r\n", lreco);

			lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][gearRecod];
			rmSpeed = AgvGear[gearRecod];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 13;

			if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
			{
				if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
				{
					rreco++;
				}
				
			}

			/*
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->comflag = 131;
				
				//rreco++;
				//printf("rreco = %d\r\n", rreco);
				lmSpeed = AgvGear[gearRecod];
				rmSpeed = AgvGear[gearRecod];
			}
			else
			{
				//printf("gearRecod = %d\r\n", gearRecod);
				ctrlParasPtr->comflag = 132;
				lmSpeed = AgvGear[gearRecod];
				rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + roffset;
			}
			*/
			
			lmSpeed = AgvGear[gearRecod];
			rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
		else if(Agv_MS_Right_Outside == FMSDS_Ptr->AgvMSLocation)
		{

			ctrlParasPtr->comflag = 14;
			
			lmSpeed = 10 + FLG[0][gearRecod];
			rmSpeed = 10 + FRG[0][gearRecod];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);

		}
		else if(Agv_MS_Left_Outside == FMSDS_Ptr->AgvMSLocation)
		{	
			ctrlParasPtr->comflag = 15;
			
			
			lmSpeed = 10 + FLG[0][gearRecod];
			rmSpeed = 10 + FRG[0][gearRecod];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
		else
		{
			ctrlParasPtr->comflag = 16;

			lmSpeed = 10 + FLG[0][gearRecod];
			rmSpeed = 10 + FRG[0][gearRecod];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
	}
	
}

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


void AGV_Correct_gS_1(u8 gear)
{
	static u32 startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0;
	//static u8 loffset = 0, roffset = 5;
	//static Agv_MS_Location MSLRecode = AgvInits;
	//u8 LTM_flag = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)


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

	
	if(0 == ctrlParasPtr->FSflag)
	{
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			
			ctrlParasPtr->comflag = 61;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
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
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_3))
			{
				ctrlParasPtr->FSflag = 1;
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
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]] + FLG[2][gearRecod];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				
				
				
				lreco = rreco + 3;
			}

			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 62;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_3))
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
				
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->FSflag = 1;

				ctrlParasPtr->comflag = 622;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]] + FRG[2][2];
				
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);


				
				rreco = lreco + 3;
			}

			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 63;
			//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
			
		}
		
		
		
	}











	
	else
	{


		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 64;

			//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
			lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 65;

			lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]];
			//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
			rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);

			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation == Agv_MS_Center))
		{
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
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->comflag = 66;
				
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				//rmSpeed = AgvGear[gearRecod] + FRG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				startCount = 0;
			}
			
		}
	}
	
	
	
}


void AGV_Correct_gS_2(u8 gear)
{
	static u32 startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0;
	//static u8 loffset = 0, roffset = 5;
	//static Agv_MS_Location MSLRecode = AgvInits;
	//u8 LTM_flag = 0;
	u32 centCount = 0;
	
	
	ctrlParasPtr->comflag = 2;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	
	if(0 == ctrlParasPtr->FSflag)
	{
		gearRecod = gear;
		
		ctrlParasPtr->comflag = 21;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			
			ctrlParasPtr->comflag = 211;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_4) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				
				ctrlParasPtr->comflag = 2111;
				
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					//if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Center)
					if(FMSDS_Pre_Ptr->AgvMSLocation - FMSDS_Ptr->AgvMSLocation > 0)
					{
						lreco++;
					}
					
				}
				
				if(AGV_Pat_Ptr->Angle < 0)			// 如果是车体角度偏左
				{
					ctrlParasPtr->comflag = 21111;
					//printf("lreco = %d\r\n", lreco);

					//lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][gearRecod];
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);
				}
				else if(AGV_Pat_Ptr->Angle >= 0)		// 如果是车体角度偏右
				{
					ctrlParasPtr->comflag = 21112;
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[0];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);
				}
				else								// 如果是车体角度跟磁条平行
				{
					ctrlParasPtr->comflag = 21113;
					printf("error\r\n");
				}
				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_4))
			{
				ctrlParasPtr->FSflag = 1;

				ctrlParasPtr->comflag = 2112;
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				
				lreco = rreco + 3;
			}

			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			
			ctrlParasPtr->comflag = 2113;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_4))
			{

				ctrlParasPtr->comflag = 21131;
				
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{

					ctrlParasPtr->comflag = 211311;
					
					//if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
					if(FMSDS_Ptr->AgvMSLocation - FMSDS_Pre_Ptr->AgvMSLocation > 0)
					{
						ctrlParasPtr->comflag = 2113111;
						
						rreco++;
					}
					
				}
				
				
				if(AGV_Pat_Ptr->Angle <= 0)			// 如果是车体角度偏左
				{
					ctrlParasPtr->comflag = 211312;
					
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[0];
		
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
					
					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);	
					
					
				}
				else if(AGV_Pat_Ptr->Angle > 0)		// 如果是车体角度偏右
				{
					ctrlParasPtr->comflag = 211313;
					
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
		
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
					
					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);	
				}
				else								// 如果是车体角度跟磁条平行
				{
					printf("error2\r\n");
				}

				
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_4) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->FSflag = 1;

				ctrlParasPtr->comflag = 21132;
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);


				
				lreco = rreco + 3;
			}

			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			
			ctrlParasPtr->comflag = 2114;
			
			//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
			if(AGV_Pat_Ptr->Angle > 0)			// 如果是车体角度偏左
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)		// 如果是车体角度偏右
			{

			}
			else								// 如果是车体角度跟磁条平行
			{
				
			}
	
			
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
		}
		
		
		
	}











	
	else
	{

		gearRecod = 2;

		ctrlParasPtr->comflag = 22;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{

			ctrlParasPtr->comflag = 221;

			//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		

			ctrlParasPtr->comflag = 222;

			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);

			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation == Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 223;
			
			if(0 == startCount)
			{
				ctrlParasPtr->comflag = 2231;
				startCount = SystemRunningTime;
			}
			else
			{
				ctrlParasPtr->comflag = 2232;
				centCount = SystemRunningTime - startCount;
			}
			
			if(centCount > 5000)
			{
				ctrlParasPtr->FSflag = 0;

				ctrlParasPtr->comflag = 2233;

				gearRecod = gear;
				
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				//rmSpeed = AgvGear[gearRecod] + FRG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				startCount = 0;
			}
			
		}
	}
	
	

	// 自适应学习模块
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

	
}





void AGV_Correct_gS_3(u8 gear)
{
	static u32 startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, lreco = 0, rreco = 0;
	//static u8 loffset = 0, roffset = 5;
	//static Agv_MS_Location MSLRecode = AgvInits;
	u8 gearRecod = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 9;

	
	
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



	
	if(0 == ctrlParasPtr->FSflag)
	{
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往磁条左边偏移
		{
			
			
			ctrlParasPtr->comflag = 91;

			if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
			{
				//if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Center)
				if(FMSDS_Pre_Ptr->AgvMSLocation - FMSDS_Ptr->AgvMSLocation > 0)
				{
					lreco++;
				}
				
			}

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_4) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))		// 偏差在三格以内
			{
				ctrlParasPtr->comflag = 911;

				if(AGV_Pat_Ptr->Angle < 0)			// 如果是车体角度偏左
				{
					ctrlParasPtr->comflag = 9111;
					//printf("lreco = %d\r\n", lreco);

					//lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][gearRecod];
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);
				}
				else if(AGV_Pat_Ptr->Angle >= 0)		// 如果是车体角度偏右
				{
					ctrlParasPtr->comflag = 9112;
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[0];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);
				}
				else								// 如果是车体角度跟磁条平行
				{
					printf("error\r\n");
				}
				
				
				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_4))
			{
				
				ctrlParasPtr->comflag = 912;

				#if 0
				ctrlParasPtr->FSflag = 1;
				
				if(AGV_Pat_Ptr->Angle < 0)			// 如果是车体角度偏左
				{
					
				}
				else if(AGV_Pat_Ptr->Angle > 0)		// 如果是车体角度偏右
				{

				}
				else								// 如果是车体角度跟磁条平行
				{
					
				}
				
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
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]] + FLG[2][gearRecod];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				
				

				#else


				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);


				#endif

				
				lreco = rreco + 3;
			}

						
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 913;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_4))
			{
				ctrlParasPtr->comflag = 9131;

				if(AGV_Pat_Ptr->Angle <= 0)			// 如果是车体角度偏左
				{
					ctrlParasPtr->comflag = 91311;
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[0];
		
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
					
					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);	
					
					
				}
				else if(AGV_Pat_Ptr->Angle > 0)		// 如果是车体角度偏右
				{
					ctrlParasPtr->comflag = 91312;
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
		
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
					
					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);	
				}
				else								// 如果是车体角度跟磁条平行
				{
					printf("error2\r\n");
				}
				
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					ctrlParasPtr->comflag = 9132;
					
					//if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
					if(FMSDS_Ptr->AgvMSLocation - FMSDS_Pre_Ptr->AgvMSLocation > 0)
					{
						ctrlParasPtr->comflag = 91321;
						rreco++;
					}
					
				}
				

				
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_4) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->comflag = 914;

				#if 0
				ctrlParasPtr->FSflag = 1;
				
				if(AGV_Pat_Ptr->Angle > 0)			// 如果是车体角度偏左
				{
					
				}
				else if(AGV_Pat_Ptr->Angle < 0)		// 如果是车体角度偏右
				{

				}
				else								// 如果是车体角度跟磁条平行
				{
					
				}
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]] + FRG[2][2];
				
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);


				#else


				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);	
				

				#endif

				
				rreco = lreco + 3;
			}
			
			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 915;
			//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];

			if(AGV_Pat_Ptr->Angle > 0)			// 如果是车体角度偏左
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)		// 如果是车体角度偏右
			{

			}
			else								// 如果是车体角度跟磁条平行
			{
				
			}
	
			
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);

			
			
		}
		
		
		
	}











	
	else
	{

		ctrlParasPtr->comflag = 92;
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 921;

			if(AGV_Pat_Ptr->Angle > 0)			// 如果是车体角度偏左
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)		// 如果是车体角度偏右
			{

			}
			else								// 如果是车体角度跟磁条平行
			{
				
			}

			//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
			lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 922;

			if(AGV_Pat_Ptr->Angle > 0)			// 如果是车体角度偏左
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)		// 如果是车体角度偏右
			{

			}
			else								// 如果是车体角度跟磁条平行
			{
				
			}

			lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]];
			rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);

			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation == Agv_MS_Center))
		{
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
			}
			
			if(centCount > 7500)
			{
				ctrlParasPtr->FSflag = 0;
				
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				//rmSpeed = AgvGear[gearRecod] + FRG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				startCount = 0;
			}
			
		}
	}
	
	
}


void AGV_Correct_gS_4(u8 gear)
{
	//static u32 startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0;
	//static u8 loffset = 0, roffset = 5;
	//static Agv_MS_Location MSLRecode = AgvInits;
	//u8 LTM_flag = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	
	
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
			
			printf("loffset = %d, roffset = %d\r\n", BLeftCompDuty[AgvGear[gearRecod]], BRightCompDuty[AgvGear[gearRecod]]);
			
			lreco = 0;
			rreco = 0;
		}
	}



	
	if(0 == ctrlParasPtr->FSflag)
	{
			
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			
			ctrlParasPtr->comflag = 12;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				
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
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_3))
			{
				ctrlParasPtr->BSflag = 1;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_BACK_MODE();
				
				//lmSpeed = AgvGear[2] + FLG[0][2] + FLG[2][gearRecod];
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]] + FLG[2][gearRecod];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				
				
				lreco = rreco + 3;
			}

			
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 13;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_3))
			{
				
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					
					//if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
					if(FMSDS_Ptr->AgvMSLocation - FMSDS_Pre_Ptr->AgvMSLocation > 0)
					{
						rreco++;
					}
					
				}
				
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->BSflag = 1;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]] + FRG[2][2];
				
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				
				rreco = lreco + 3;
			}
			
			
			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);


			
		}
		
		
		
	}











	
	else
	{


		
	}
	
	
	
}


void gS_step_gS(u8 gear)
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
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
			#if 1
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3))
			{
				ctrlParasPtr->FSflag = 1;
				ctrlParasPtr->comflag = 612;
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				//lmSpeed = AgvGear[2] + FLG[0][2] + FLG[2][gearRecod];
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]] + FLG[2][gearRecod];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				
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
				
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				
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

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				lmSpeed = AgvGear[3] + FLeftCompDuty[AgvGear[3]];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[3] + FRightCompDuty[AgvGear[3]] + FRG[3][3];
				
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);


				
				rreco = lreco + 3;
			}
			#endif
			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 63;

			// 加入阻尼模块
			// 阻尼begin
			#if 0
			if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 631;
				ctrlParasPtr->dampingFlag = DampingLeft;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;
				
				//LocValu(FMSDS_Ptr->MaxRecoder);
				
				//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[LocValu(FMSDS_Ptr->MaxRecoder)];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				
				MOTOR_LEFT_DUTY_SET_F(lmSpeed);
				
				printf("lmSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
			}
			else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 632;
				ctrlParasPtr->dampingFlag = DampingRight;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;

				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[LocValu(FMSDS_Ptr->MaxRecoder)];
				
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_RIGHT_DUTY_SET_F(rmSpeed);

				printf("rmSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
			}
			
			// 阻尼end
			//else if(AgvNone == FMSDS_Ptr->agvDirection)
			else
			{
				ctrlParasPtr->comflag = 633;
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
			#endif

			ctrlParasPtr->comflag = 633;
			
			//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
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
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 65;

			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);

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
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
		
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);

					startCount = 0;
				}
			}
			else
			{
				
				startCount = 0;
				#if 0
				ctrlParasPtr->comflag = 662;
				if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
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

void gS_step_gS2(u8 gear)
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
			

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
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
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				MOTOR_DUTY_SET(lmSpeed, rmSpeed);
				
			}
			#if 0
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_4))
			{
				ctrlParasPtr->FSflag = 1;
				ctrlParasPtr->comflag = 612;
				lmSpeed = 0;
				rmSpeed = 0;
				
				MOTOR_DUTY_SET(lmSpeed, rmSpeed);

				//CHANGE_TO_STOP_MODE();
				//Delay_ms(1000);
				//CHANGE_TO_GO_STRAIGHT_MODE();
				
				//lmSpeed = AgvGear[2] + FLG[0][2] + FLG[2][gearRecod];
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]] + FLG[2][gearRecod];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]];
				
				MOTOR_DUTY_SET(lmSpeed, rmSpeed);
				
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

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_3))
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
				
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				MOTOR_DUTY_SET(lmSpeed, rmSpeed);

				
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->FSflag = 1;

				ctrlParasPtr->comflag = 622;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				MOTOR_DUTY_SET(lmSpeed, rmSpeed);

				//CHANGE_TO_STOP_MODE();
				//Delay_ms(1000);
				//CHANGE_TO_GO_STRAIGHT_MODE();
				
				lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]] + FRG[2][2];
				
				MOTOR_DUTY_SET(lmSpeed, rmSpeed);
				
				rreco = lreco + 3;
			}

			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 63;

			// 加入阻尼模块
			// 阻尼begin
			
			if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 631;
				ctrlParasPtr->dampingFlag = DampingLeft;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;
				
				//LocValu(FMSDS_Ptr->MaxRecoder);
				
				//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[LocValu(FMSDS_Ptr->MaxRecoder)];
								
				MOTOR_LEFT_DUTY_SET_F(lmSpeed);
				
			}
			else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 632;
				ctrlParasPtr->dampingFlag = DampingRight;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;

				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[LocValu(FMSDS_Ptr->MaxRecoder)];
								
				MOTOR_RIGHT_DUTY_SET_F(rmSpeed);
			}
			// 阻尼end
			//else if(AgvNone == FMSDS_Ptr->agvDirection)
			else
			{
				ctrlParasPtr->comflag = 633;
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

				MOTOR_DUTY_SET(lmSpeed, rmSpeed);
			}
			
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
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
			
			MOTOR_DUTY_SET(lmSpeed, rmSpeed);
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 65;

			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

			MOTOR_DUTY_SET(lmSpeed, rmSpeed);
			
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
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
		
					MOTOR_DUTY_SET(lmSpeed, rmSpeed);

					startCount = 0;
				}
			}
			else
			{
				startCount = 0;
				ctrlParasPtr->comflag = 662;
				if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
				{
					ctrlParasPtr->comflag = 6621;
					ctrlParasPtr->dampingFlag = DampingLeft;
					ctrlParasPtr->dampingTimeRec = SystemRunningTime;

					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
										
					MOTOR_LEFT_DUTY_SET_F(lmSpeed);
				}
				else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
				{
					ctrlParasPtr->comflag = 6622;
					ctrlParasPtr->dampingFlag = DampingRight;
					ctrlParasPtr->dampingTimeRec = SystemRunningTime;
					
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
										
					MOTOR_RIGHT_DUTY_SET_F(rmSpeed);
				}
				// 阻尼end
				
			}

		}
	}
	

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

	
}

void gS_step_gS3(u8 gear)
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
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				
			}
		#if 1
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3))
			{
				///// 偏差大的情况下
				
				ctrlParasPtr->FSflag = 1;
				ctrlParasPtr->comflag = 612;
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				//lmSpeed = AgvGear[2] + FLG[0][2] + FLG[2][gearRecod];
				lmSpeed = AgvGear[3] + AgvGearCompDutyLF[3] + FLG[3][3];
				rmSpeed = AgvGear[3] + AgvGearCompDutyRF[3];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				
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
				
				
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				
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

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_GO_STRAIGHT_MODE();
				
				lmSpeed = AgvGear[3] + AgvGearCompDutyLF[3];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[3] + AgvGearCompDutyRF[3] + FRG[2][2];
				
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				
				rreco = lreco + 3;
			}
		#endif
			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 63;
		#if 1
		
			// 加入阻尼模块
			// 阻尼begin	
			if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 631;
				ctrlParasPtr->dampingFlag = DampingLeft;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;
				
				//LocValu(FMSDS_Ptr->MaxRecoder);
				
				//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - DutyTableLow[LocValu(FMSDS_Ptr->MaxRecoder)];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] + DutyTableLow[LocValu(FMSDS_Ptr->MaxRecoder)];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET_F(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				//printf("lmSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
			}
			else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 632;
				ctrlParasPtr->dampingFlag = DampingRight;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;

				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] + DutyTableLow[LocValu(FMSDS_Ptr->MaxRecoder)];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - DutyTableLow[LocValu(FMSDS_Ptr->MaxRecoder)];
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				MOTOR_LEFT_DUTY_SET_F(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				//printf("rmSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
			}
			// 阻尼end
			
			//else if(AgvNone == FMSDS_Ptr->agvDirection)
			else
			{
				ctrlParasPtr->comflag = 633;
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
		#endif

			
			
			FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
			
			
		}
		
	}






	
	else
	{
		gearRecod = 3;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 64;

			//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 65;
			
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);

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
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
		
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);

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




void gS_step_entry(u8 gearRecod)
{
	static u8 lmSpeed = 0, rmSpeed = 0, lreco = 0, rreco = 0;
	static u32 startCount = 0;
	//u32 centCount = 0;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 64;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
		rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
		
		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

		MOTOR_LEFT_DUTY_SET(lmSpeed);
		MOTOR_RIGHT_DUTY_SET(rmSpeed);
		
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 65;

		lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

		MOTOR_LEFT_DUTY_SET(lmSpeed);
		MOTOR_RIGHT_DUTY_SET(rmSpeed);

		startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 66;

		
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
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

void gS_step_entry2(u8 gearRecod)
{
	static u8 lmSpeed = 0, rmSpeed = 0, lreco = 0, rreco = 0;
	static u32 startCount = 0;
	
	gearRecod = 2;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 64;

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
		
		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

		MOTOR_LEFT_DUTY_SET(lmSpeed);
		MOTOR_RIGHT_DUTY_SET(rmSpeed);
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 65;

		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
		//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

		MOTOR_LEFT_DUTY_SET(lmSpeed);
		MOTOR_RIGHT_DUTY_SET(rmSpeed);

		startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 66;
		
		if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
		}

	}
}





void AGV_Correct_gS_5(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 2;
	
	ctrlParasPtr->comflag = 6;
	

	if(step_gS == ctrlParasPtr->walkingstep)
	{
		
		gearRecod = gear;
		gS_step_gS3(gearRecod);
	}
	else if(step_entry == ctrlParasPtr->walkingstep)
	{
		gearRecod = 2;
		gS_step_entry(gearRecod);
	}
	
	
	if(DampingNone != ctrlParasPtr->dampingFlag)		// 如果启动阻尼
	{
		ctrlParasPtr->comflag = 67;
		
		if((SystemRunningTime - ctrlParasPtr->dampingTimeRec) > (FMSDS_Ptr->VelocityXt))		// 时间到, 恢复
		{
			printf("tim = %d, vxt = %d\r\n", (SystemRunningTime - ctrlParasPtr->dampingTimeRec), (FMSDS_Ptr->VelocityXt));
			ctrlParasPtr->comflag = 671;
			
			if(DampingLeft == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6711;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				printf("in left\r\n");
			}
			else if(DampingRight == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6712;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				printf("tim = %d, vxt = %d\r\n", (SystemRunningTime - ctrlParasPtr->dampingTimeRec), (FMSDS_Ptr->VelocityXt));
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				printf("in right\r\n");
			}
			
		}
	}

	
}


void AGV_Correct_gS_6(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 2;
	
	ctrlParasPtr->comflag = 6;
	

	if(step_gS == ctrlParasPtr->walkingstep)
	{
		gearRecod = gear;
		gS_step_gS2(gearRecod);
	}
	else if(step_entry == ctrlParasPtr->walkingstep)
	{
		gearRecod = 2;
		gS_step_entry2(gearRecod);
	}
	
	
	if(DampingNone != ctrlParasPtr->dampingFlag)		// 如果启动阻尼
	{
		ctrlParasPtr->comflag = 67;
		
		if((SystemRunningTime - ctrlParasPtr->dampingTimeRec) > (FMSDS_Ptr->VelocityXt >> 0))		// 时间到, 恢复
		{
			ctrlParasPtr->comflag = 671;
			if(DampingLeft == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6711;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				
				MOTOR_LEFT_DUTY_SET_F(lmSpeed);
				
			}
			else if(DampingRight == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6712;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				MOTOR_RIGHT_DUTY_SET_F(rmSpeed);
			}
			
		}
	}

	
}

void AGV_Correct_gS_7(u8 gear)
{
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;
	
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	gearRecod = gear;
	//printf("************\r\n");
	if((Agv_MS_CrossRoad != FMSDS_Pre_Ptr->AgvMSLocation) && (Agv_MS_Undefine != FMSDS_Pre_Ptr->AgvMSLocation) &&\
		(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3))
	{
		//printf("AgvMSLocation %d, %d\r\n",FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation);
		if(0 == ctrlParasPtr->FSflag)
		{
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				
				ctrlParasPtr->comflag = 61;
					
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					//if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Center)
					if(FMSDS_Pre_Ptr->AgvMSLocation - FMSDS_Ptr->AgvMSLocation > 0)
					{
						lreco++;
					}
					
				}

				#if 0
				if(AgvCent2Left == FMSDS_Ptr->agvDirection)				// 如果是简谐运动从中线往左偏移, 应该拉低右边电机duty, 以抵消往左偏的力量
				{
					if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将右边电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
					else												// 如果还没回到1格, 那么还应该持续拉右边电机的duty
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
						printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
					}
					
				}
				
				else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从左往中线回, 应该拉低左边电机的duty以达到减少回冲的速度
				{
					
					if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将左边的电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
					else												// 如果还没回到1格, 那么应该持续拉左边电机的duty
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						printf("2: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
					}
				}
				#endif

				#if 0
				if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将右边电机的duty恢复, 放开回拉的力量
				{
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				}
				else												// 如果还没回到1格, 那么还应该持续拉右边电机的duty
				{
					if(AgvLeft2Cent != FMSDS_Ptr->agvDirection)
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					}
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
					printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				}

				if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从左往中线回, 应该拉低左边电机的duty以达到减少回冲的速度
				{
					
					if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将左边的电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
					else												// 如果还没回到1格, 那么应该持续拉左边电机的duty
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						printf("2: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
					}
				}
				#else

				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

				if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将右边电机的duty恢复, 放开回拉的力量
				{
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
					//printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				}

				if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从左往中线回, 应该拉低左边电机的duty以达到减少回冲的速度
				{
					
					if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将左边的电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
						//printf("2: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
					}
				}
				
				#endif
				if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
				{
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);
				}
				else
				{
					printf("dutyErr1! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
				}
				
				//printf("lreco = %d\r\n", lreco);

				//lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][gearRecod];
				//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];

				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{		
				ctrlParasPtr->comflag = 62;

				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_9))
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

					#if 0
					if(AgvCent2Right == FMSDS_Ptr->agvDirection)			// 如果是简谐运动从中线往右偏移, 应该拉低左边电机duty, 以抵消往右偏的力量
					{
						if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将左边电机的duty恢复, 放开回拉的力量
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						}
						else												// 如果还没回到1格, 那么还应该持续拉左边电机的duty
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
							printf("3: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
						}
						
					}
					
					else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从右往中线回, 应该拉低右边电机的duty以达到减少回冲的速度
					{
						#if 1
						if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将右边的电机的duty恢复, 放开回拉的力量
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						}
						else												// 如果还没回到1格, 那么应该持续拉右边电机的duty
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
							printf("4: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
						}
						#else
						
						#endif
					}
					#endif

					#if 0
					if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将左边电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
					else												// 如果还没回到1格, 那么还应该持续拉左边电机的duty
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
						if(AgvRight2Cent != FMSDS_Ptr->agvDirection)
						{
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						}
						printf("3: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
					}

					if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从右往中线回, 应该拉低右边电机的duty以达到减少回冲的速度
					{
						
						if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将右边的电机的duty恢复, 放开回拉的力量
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						}
						else												// 如果还没回到1格, 那么应该持续拉右边电机的duty			
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
							printf("4: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
						}
					}

					#else

					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

					if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1)
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
						//printf("3: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
					}

					if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从右往中线回, 应该拉低右边电机的duty以达到减少回冲的速度
					{
						if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将右边的电机的duty恢复, 放开回拉的力量
						{
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
							//printf("4: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
						}
					}
					
					#endif
					
					if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
					{
						ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
						ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

						MOTOR_LEFT_DUTY_SET(lmSpeed);
						MOTOR_RIGHT_DUTY_SET(rmSpeed);
					}
					else
					{
						printf("dutyErr2! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
					}
					
					//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
					//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
					//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
		
					//ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					//ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
					
					//MOTOR_LEFT_DUTY_SET(lmSpeed);
					//MOTOR_RIGHT_DUTY_SET(rmSpeed);

					
				}
				else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_9) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
				{
					ctrlParasPtr->FSflag = 1;
					
					ctrlParasPtr->comflag = 622;
				#if 1
					lmSpeed = 0;
					rmSpeed = 0;
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);

					CHANGE_TO_STOP_MODE();
					Delay_ms(1000);
					CHANGE_TO_GO_STRAIGHT_MODE();
					
					lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]];
					//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
					rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]] + FRG[2][2];
					
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);

					
				#endif
					rreco = lreco + 3;
				}

				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->comflag = 63;

				// 加入阻尼模块
				// 阻尼begin
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				
			}
		
		
		
		}

		
	}
	
	
	
}

void AGV_Correct_gS_7_ug(u8 gear)
{
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0, falg2 = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;
	
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	gearRecod = gear;
	//printf("************\r\n");
	if((Agv_MS_CrossRoad != FMSDS_Pre_Ptr->AgvMSLocation) && (Agv_MS_Undefine != FMSDS_Pre_Ptr->AgvMSLocation) &&\
		(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3))
	{
		//printf("AgvMSLocation %d, %d\r\n",FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation);
		if(0 == ctrlParasPtr->FSflag)
		{
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				
				ctrlParasPtr->comflag = 61;

				
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
				
				
				
				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{		
				ctrlParasPtr->comflag = 62;
				
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));

				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->comflag = 63;

				
				
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				
			}
		
			if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
			{
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
			else
			{
				printf("dutyErr2! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
			}
		
		}

		
	}
	
	
	
}


void AGV_Correct_gS_7_ug2(u8 gear)
{
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0, falg2 = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;
	
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	gearRecod = gear;
	//printf("************\r\n");
	if((Agv_MS_CrossRoad != FMSDS_Pre_Ptr->AgvMSLocation) && (Agv_MS_Undefine != FMSDS_Pre_Ptr->AgvMSLocation) &&\
		(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3))
	{
		//printf("AgvMSLocation %d, %d\r\n",FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation);
		if(0 == ctrlParasPtr->FSflag)
		{
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				
				ctrlParasPtr->comflag = 61;

				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

				/*
				if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1)		// 
				{
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
					//printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				}
				*/

				
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
				
				if(AgvCent2Left == FMSDS_Ptr->agvDirection)
				{
					FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
					
					if(MSLRecode != FMSDS_Ptr->MaxRecoder)
					{
						MSLRecode = FMSDS_Ptr->MaxRecoder;
						printf("1: MaxRecoder = %d\r\n", (Agv_MS_Center - FMSDS_Ptr->MaxRecoder));
					}
					flag = 1;
				}
				else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)		// 
				{
					if(falg2 != (FMSDS_Ptr->AgvMSLocation - FMSDS_Ptr->MaxRecoder))
					{
						falg2 = (FMSDS_Ptr->AgvMSLocation - FMSDS_Ptr->MaxRecoder);
						printf("2: falg2 = %d\r\n", (FMSDS_Ptr->AgvMSLocation - FMSDS_Ptr->MaxRecoder));
					}

					if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3)
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Left_3 - FMSDS_Ptr->AgvMSLocation));
						
					}
					
					//printf("2: %d, %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->MaxRecoder), (FMSDS_Ptr->AgvMSLocation - FMSDS_Ptr->MaxRecoder), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - FMSDS_Ptr->MaxRecoder)));
				}
				
				
				
				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{		
				ctrlParasPtr->comflag = 62;

				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_9))
				{
					
					ctrlParasPtr->comflag = 621;
					
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
					
					if(AgvCent2Right == FMSDS_Ptr->agvDirection)
					{
						FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
						
						if(MSLRecode != FMSDS_Ptr->MaxRecoder)
						{
							MSLRecode = FMSDS_Ptr->MaxRecoder;
							printf("3: MaxRecoder = %d\r\n", (FMSDS_Ptr->MaxRecoder - Agv_MS_Center));
						}
					}
					else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从右往中线回, 应该拉低右边电机的duty以达到减少回冲的速度
					{
						if(falg2 != (FMSDS_Ptr->MaxRecoder - FMSDS_Ptr->AgvMSLocation))
						{
							falg2 = (FMSDS_Ptr->MaxRecoder - FMSDS_Ptr->AgvMSLocation);
							printf("4: falg2 = %d\r\n", (FMSDS_Ptr->MaxRecoder - FMSDS_Ptr->AgvMSLocation));
						}

						if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_3)
						{
							
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_3));
						}
						//printf("4: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
					}
					
					
					
					
				}
				

				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->comflag = 63;

				// 加入阻尼模块
				// 阻尼begin
				FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				
			}
		
			if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
			{
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
			else
			{
				printf("dutyErr2! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
			}
		
		}

		
	}
	
	
	
}



void AGV_Correct_gS_8(u8 gear)		// 3 mode
{
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;
	static u8 modeFlag = 0xff;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	gearRecod = gear;
	
	//printf("************\r\n");

	
	if((Agv_MS_CrossRoad != FMSDS_Pre_Ptr->AgvMSLocation) && (Agv_MS_Undefine != FMSDS_Pre_Ptr->AgvMSLocation) &&\
		(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3))
	{
		//printf("AgvMSLocation %d, %d\r\n",FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation);
		
		
		/********************************/
		if(0 == ctrlParasPtr->FSflag)			// 启动修正模式, 进入修正控制	
		{
			ctrlParasPtr->comflag = 61;
		}
		else if(1 == ctrlParasPtr->FSflag)		// 普通模式下
		{
			ctrlParasPtr->comflag = 62;
			
			if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End))
			{
				// 超过5格, 跑失控修正模式
				ctrlParasPtr->FSflag = 2;
				ctrlParasPtr->comflag = 621;
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				// 超过5格, 跑失控修正模式
				ctrlParasPtr->FSflag = 2;
				ctrlParasPtr->comflag = 622;
			}
			
			
		}
		
		
		
		
		/***********************实现************************/
		
		

		if(0 == ctrlParasPtr->FSflag)		
		{
			// 失控/启动修正模式
			ctrlParasPtr->comflag = 63;
			
			#if 0
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{	
				
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));		
				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{	
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));	
				
			}
			else if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
			{
				if(Agv_MS_Center == RMSDS_Ptr->AgvMSLocation)
				{
					
				}
				else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
				{
					
					//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - AgvGearK[gearRecod];				
				}
				else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
				{
					
					//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - AgvGearK[gearRecod];
				}
			}

			#else

			gearRecod = 3;

			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
		
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				ctrlParasPtr->comflag = 631;

				//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
				
				
				
				startCount = 0;
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{		
				ctrlParasPtr->comflag = 632;
				
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				//printf("flcd[%d] = %d\r\n", AgvGear[2], FLeftCompDuty[AgvGear[2]]);
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
				
				
				startCount = 0;
			}
			else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation == Agv_MS_Center))
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
					ctrlParasPtr->FSflag = 1;
					ctrlParasPtr->comflag = 6331;
					
					startCount = 0;
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
				}
				
			}
			


			#endif
			
		}
		else if(1 == ctrlParasPtr->FSflag)
		{
			// 普通模式,偏差在三格之内

			lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
			
			ctrlParasPtr->comflag = 64;
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				
				ctrlParasPtr->comflag = 641;
				

				if(AgvCent2Left == FMSDS_Ptr->agvDirection)
				{
					FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
				}
				
				//printf("1gearRecod = %d\r\n", gearRecod);
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - AgvGear7CDLF[Agv_MS_Center - FMSDS_Ptr->AgvMSLocation]; 		
				
				if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
				{
					ctrlParasPtr->comflag = 6411;
					if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_5))
					{
						ctrlParasPtr->comflag = 64111;
						lmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - AgvGear7CDLF[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation]; 		
						//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					
					}
					else if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_5)
					{
						ctrlParasPtr->comflag = 64112;	
						lmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - AgvGear7CDLF[Agv_MS_Left_1 - Agv_MS_Left_5]; 		
						//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
				}
				
				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{	
				ctrlParasPtr->comflag = 642;

				if(AgvCent2Right == FMSDS_Ptr->agvDirection)
				{
					FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
				}
				
				//printf("** 621 622 623  %d\r\n", FMSDS_Ptr->AgvMSLocation);
	
				
				
				//printf("2gearRecod = %d\r\n", gearRecod);
				
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - AgvGear7CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Center];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

				if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
				{
					ctrlParasPtr->comflag = 6421;
					if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_5))
					{

						ctrlParasPtr->comflag = 64211;
						rmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - AgvGear7CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
					}
					else if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_5)
					{
						ctrlParasPtr->comflag = 64212;
						rmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - AgvGear7CDLF[Agv_MS_Right_5 - Agv_MS_Right_1];
					}
				}
				
				
			}
			
			
			
			
		}
		else if(2 == ctrlParasPtr->FSflag)
		{
			// 
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
			
		}
		
		if(DampingNone != ctrlParasPtr->dampingFlag)		// 如果启动阻尼
		{
			ctrlParasPtr->comflag = 66;
			
			if((SystemRunningTime - ctrlParasPtr->dampingTimeRec) > 500)		// 时间到, 恢复
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

		if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
		{
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
		}
		else
		{
			printf("dutyErr2! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
		}
	
	}
	
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
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
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
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
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
				ctrlParasPtr->FSflag = 1;
				ctrlParasPtr->comflag = 6331;
				
				startCount = 0;
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			}
			
		}


		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	damping_func(250, gearRecod, lmSpeed, rmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
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
			
			if(centCount > 5000)
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
	u32 centCount = 0;
	static u32 startCount = 0;
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
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
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
	
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	damping_func(250, gearRecod, lmSpeed, rmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}

void back_slow(u8 gear)
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
	
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	damping_func(250, gearRecod, rmSpeed, lmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
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

void scale_1_mode(u8 gearRecod)
{
	static u8 lmSpeed = 0, rmSpeed = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	u8 AgvGearS1CDLF[MAX_GEAR_OFFSET] = {0, 2, 7, 8, 10, 12, 14, 16, 18, 20, 20};
	
	// 普通模式,偏差在1格之内调整

	lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
	rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
	
	ctrlParasPtr->comflag = 64;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;
		

		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
		}
		
		//printf("1gearRecod = %d\r\n", gearRecod);
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Center - FMSDS_Ptr->AgvMSLocation]; 		

		#if 0
		
		if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
		{
			ctrlParasPtr->comflag = 6411;
			if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_5))
			{
				ctrlParasPtr->comflag = 64111;
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation]; 		
				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
			
			}
			else if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_5)
			{
				ctrlParasPtr->comflag = 64112;	
				lmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - AgvGearS1CDLF[Agv_MS_Left_1 - Agv_MS_Left_5];		
				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
			}
		}

		#else

		
		
		#endif
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{	
		ctrlParasPtr->comflag = 642;

		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
		}
		
		//printf("** 621 622 623  %d\r\n", FMSDS_Ptr->AgvMSLocation);
		
		//printf("2gearRecod = %d\r\n", gearRecod);
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Center];

		#if 0
		
		if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
		{
			ctrlParasPtr->comflag = 6421;
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_5))
			{

				ctrlParasPtr->comflag = 64211;
				rmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
			}
			else if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_5)
			{
				ctrlParasPtr->comflag = 64212;
				rmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - AgvGearS1CDLF[Agv_MS_Right_5 - Agv_MS_Right_1];
			}
		}
		
		#else

		
		
		#endif
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

	#if 1
		// 加入阻尼模块
		// 阻尼begin	
		if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6341;
			ctrlParasPtr->dampingFlag = DampingLeft;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;
			
			//LocValu(FMSDS_Ptr->MaxRecoder);
			
			//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[1];
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
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[1];
			

			//printf("rmSpeed = %d\r\n", ctrlParasPtr->rightMotorSettedSpeed);
		}
		// 阻尼end
		//else if(AgvNone == FMSDS_Ptr->agvDirection)
	#endif
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	
	damping_func(1000, gearRecod, lmSpeed, rmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}


void scale_1_mode1(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	u8 AgvGearS1CDLF[MAX_GEAR_OFFSET] = {0, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11};
	static u8 sub = 0, flag1 = 0, flag2 = 0;
	// 普通模式,偏差在1格之内调整
	
	ctrlParasPtr->comflag = 64;

	gearRecod = gear;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;
		

		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
		}
		
		if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_2)
		{
			if(0 == flag1)
			{
				if(gear - 1 > sub)
				{
					sub++;
					printf("gearRecvd = %d\r\n", gearRecod - sub);
				}
				flag1 = 1;
			}
			
			if(1 == flag2)
			{
				flag2 = 0;
			}
			
		}
		
		//printf("1gearRecod = %d\r\n", gearRecod);
		lmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyLF[gearRecod];
		rmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Center - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{	
		ctrlParasPtr->comflag = 642;

		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
		}

		if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_2)
		{
			if(0 == flag2)
			{
				if(gear - 1 > sub)
				{
					sub++;
					printf("gearRecvd = %d\r\n", gearRecod - sub);
				}
				flag2 = 1;
			}

			if(1 == flag1)
			{
				flag1 = 0;	
			}
			
		}
		
		//printf("** 621 622 623  %d\r\n", FMSDS_Ptr->AgvMSLocation);
		//printf("2gearRecod = %d\r\n", gearRecod);
		
		
		lmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Center];
		rmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyRF[gearRecod];
		
				
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

	#if 1
		// 加入阻尼模块
		// 阻尼begin	
		if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 		// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6341;
			ctrlParasPtr->dampingFlag = DampingLeft;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;
			
			//LocValu(FMSDS_Ptr->MaxRecoder);
			
			//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			
			lmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[3];
			rmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyRF[gearRecod];
			
			//printf("lmSpeed = %d\r\n", ctrlParasPtr->leftMotorSettedSpeed);
		}
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
		{
			ctrlParasPtr->comflag = 6342;
			ctrlParasPtr->dampingFlag = DampingRight;
			ctrlParasPtr->dampingTimeRec = SystemRunningTime;

			//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
			lmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyLF[gearRecod];
			rmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[3];
			

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
				sub = 0;
				ctrlParasPtr->comflag = 6331;
				
				startCount = 0;
				
			}
			
		}

		
		lmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyLF[gearRecod];
		rmSpeed = AgvGear[gearRecod - sub] + AgvGearCompDutyRF[gearRecod];
	
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}
	
	
	damping_func(1000, gearRecod, lmSpeed, rmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}

void scale_1_mode2(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, lmSpeedR = 0, rmSpeedR = 0, modeFlag1 = 0, modeFlag2 = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	u8 AgvGearS1CDLF[MAX_GEAR_OFFSET] = {0, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec = AgvInits;
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;

	lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
	rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
	
	ctrlParasPtr->comflag = 64;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;
		

		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
		}

		if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_2)
		{
			//modeFlag1 = 1;
			lmSpeedR = gain;
		}

		if((AgvLeft2Cent == FMSDS_Ptr->agvDirection) && (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1))
		{
			lmSpeedR = 0;
		}
		
		//printf("1gearRecod = %d\r\n", gearRecod);
		
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Center - FMSDS_Ptr->AgvMSLocation] - rmSpeedR;		
		//printf("rmSpeedR = %d\r\n", rmSpeedR);
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
		//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Center - FMSDS_Ptr->AgvMSLocation];
		rmSpeedR = 0;
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{	
		ctrlParasPtr->comflag = 642;

		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
		}
		
		if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_2)
		{
			
			//modeFlag2 = 1;
			rmSpeedR = gain;
		}

		if((AgvRight2Cent == FMSDS_Ptr->agvDirection) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
		{
			lmSpeedR = 0;
		}
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Center] - lmSpeedR;		
		//printf("lmSpeedR = %d\r\n", lmSpeedR);
		//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Center];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];

		lmSpeedR = 0;
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
		
		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	
	damping_func(1000, gearRecod, lmSpeed, rmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}


void scale_1_mode3(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, lmSpeedR = 0, rmSpeedR = 0, modeFlag1 = 0, modeFlag2 = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	u8 AgvGearS1CDLF[MAX_GEAR_OFFSET] = {0, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec = AgvInits;
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;

	lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
	rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
	
	ctrlParasPtr->comflag = 64;


	if(AGV_Pat_Ptr->Midpoint > 0)
	{
		if((AGV_Pat_Ptr->Angle >= -1) && (AGV_Pat_Ptr->Angle <= 1))			// 如果角度在 ±1内
		{
			if(-1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 1;
			}
			else if(1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 2;
			}
		}
		else
		{
			
		}
	}
	else if(0 == AGV_Pat_Ptr->Midpoint)
	{
		if((AGV_Pat_Ptr->Angle >= -1) && (AGV_Pat_Ptr->Angle <= 1))			// 如果角度在 ±1内
		{
			if(-2 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 3;
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];
			}
			else if(2 == AGV_Pat_Ptr->Angle)
			{
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - 3;
			}
			
		}
		else
		{
			
		}
	}
	else if(AGV_Pat_Ptr->Midpoint < 0)
	{
		if((AGV_Pat_Ptr->Angle >= -1) && (AGV_Pat_Ptr->Angle <= 1))			// 如果角度在 ±1内
		{
			if(-1 == AGV_Pat_Ptr->Angle)
			{
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - 2;
			}
			else if(1 == AGV_Pat_Ptr->Angle)
			{
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - 1;
			}
			
		}
		else
		{
			
		}
	}

	
	damping_func(1000, gearRecod, lmSpeed, rmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
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

void scale_1_mode4(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, lmSpeed_pat = 0, rmSpeed_pat = 0, flaga = 0, flagt1 = 0, flagt2 = 0, acFlag = 0, lmflag = 0, rmflag = 0, t2flag = 0, lmSpeedbak = 0, rmSpeedbak = 0;
	u32 centCount = 0;
	static u32 startCount = 0, enterTF = 0, exitTF = 0;
	u8 AgvGearS1CDLF[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec = AgvInits, locRec2 = AgvInits, locRec3 = AgvInits, locRec4 = AgvInits;
	static u32 T1 = 0, T2 = 0, T3 = 0, centerTT = 0, ET1 = 0, countFlag = 0, ET0_5 = 0, T1pre = 0, T2pre = 0, T3pre = 0, acTST = 0;
	u8 add = 0;
	
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AgvCent2Left == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			
			if(AGV_Pat_Ptr->Midpoint > 0)
			{
				ctrlParasPtr->comflag = 64111;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641111;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641112;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641113;
					if(AGV_Pat_Ptr->Angle >= -2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
						
					}
				}
			}
			else if(0 == AGV_Pat_Ptr->Midpoint)
			{
				ctrlParasPtr->comflag = 64112;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641121;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641122;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641123;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				
			}
			else if(AGV_Pat_Ptr->Midpoint < 0)
			{
				ctrlParasPtr->comflag = 64113;
				
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641131;
					if(AGV_Pat_Ptr->Angle <= 2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
						
					}
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641132;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641133;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				
			}

			
			/*
			if((-2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
			{
				rmSpeed_pat = 2;
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed_pat = 0;
			}
			*/
			
			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6412;
			
		}
		
		
		
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
		{
			ctrlParasPtr->comflag = 6421;
			
			if(AgvCent2Right == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			
			if(AGV_Pat_Ptr->Midpoint > 0)
			{
				ctrlParasPtr->comflag = 64211;
				
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642111;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					lmflag = 1;
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642112;
					lmflag = 0;
					rmflag = 0;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642113;
					if(AGV_Pat_Ptr->Angle >= -2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
						lmflag = 0;
						rmflag = 0;
					}
					else
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
						rmflag = 1;
					}
				}
			}
			else if(0 == AGV_Pat_Ptr->Midpoint)
			{
				ctrlParasPtr->comflag = 64212;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642121;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					lmflag = 1;
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642122;
					lmflag = 0;
					rmflag = 0;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642123;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					rmflag = 0;
				}
			}
			else if(AGV_Pat_Ptr->Midpoint < 0)
			{
				ctrlParasPtr->comflag = 64213;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642131;
					if(AGV_Pat_Ptr->Angle <= 2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
						lmflag = 0;
						rmflag = 0;
					}
					else
					{
						
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
						lmflag = 1;
					}
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642132;
					lmflag = 0;
					rmflag = 0;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642133;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					rmflag = 1;
				}
			}

			
			/*
			if((-2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
			{
				rmSpeed_pat = 2;
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed_pat = 0;
			}
			*/
			
			

			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6422;
			
		}


		#if 0
		if((2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
		{
			lmSpeed_pat = 2;
		}
		else if(0 == AGV_Pat_Ptr->Angle)
		{
			lmSpeed_pat = 0;
		}

		if(Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation)
		{
			if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
			{
				printf("***VelocityXt = %d\r\n", FMSDS_Ptr->VelocityXt);
			}
			
		}
		#endif
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;

		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(-1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 1;
			}
			
		}
		else if(AGV_Pat_Ptr->Midpoint < 0)
		{
			if(1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 1;
			}
			
		}

		
		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}
	
	

#if 0
	if(locRec != FMSDS_Ptr->AgvMSLocation)
	{
		locRec = FMSDS_Ptr->AgvMSLocation;

		if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
		{
			if(0 == countFlag)
			{
				centerTT = FMSDS_Ptr->TimeRecoder;
				countFlag = 1;
				rec[recH].amlH = 0;
				rec[recH].amlrec[rec[recH].amlH] = Agv_MS_Center;
				rec[recH].amlH++;
				flaga = 1;
			}
			else if(4 == countFlag)
			{
				T3 = FMSDS_Ptr->TimeRecoder - ET0_5;
				//ET0_5 = FMSDS_Ptr->TimeRecoder;
				countFlag = 0;
				rec[recH].trec[2] = T3;
				printf("T3 = %d\r\n", T3);
				printf("recH = %d, amlH = %d\r\n", recH, rec[recH].amlH);
				showTrec();
				recH++;
				rec[recH].amlH = 0;
				flaga = 0;
				
			}
		}
		else if((Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation))
		{
			if(Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation)
			{
				if(1 == countFlag)
				{
					T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					ET1 = FMSDS_Ptr->TimeRecoder;
					countFlag = 2;
					rec[recH].trec[0] = T1;
					printf("T1 = %d\r\n", T1);
				}
			}
			else
			{
				if(1 == countFlag)
				{
					T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					ET1 = FMSDS_Ptr->TimeRecoder;
					countFlag = 3;
					rec[recH].trec[0] = T1;
					printf("T1 = %d\r\n", T1);
				}
			}
			
			
		}
		else if((Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_0_5 == FMSDS_Ptr->AgvMSLocation))
		{
			if(Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation)
			{
				if(2 == countFlag)
				{
					T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					ET1 = FMSDS_Ptr->TimeRecoder;
					countFlag = 4;
					rec[recH].trec[0] = T1;
					printf("T1 = %d\r\n", T1);
				}
			}
			else
			{
				if(3 == countFlag)
				{
					T2 = FMSDS_Ptr->TimeRecoder - ET1;
					ET0_5 = FMSDS_Ptr->TimeRecoder;
					countFlag = 4;
					rec[recH].trec[1] = T2;
					printf("T2 = %d\r\n", T2);
				}
			}
			
		}
		
	}
	
#else
	
	if(locRec != FMSDS_Ptr->AgvMSLocation)
	{
		locRec = FMSDS_Ptr->AgvMSLocation;

		if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
		{
			if(0 == countFlag)
			{
				//centerTT = FMSDS_Ptr->TimeRecoder;
				centerTT = SystemRunningTime;
				rec[recH].amlH = 0;
			}
			else if(3 == countFlag)
			{
				//T3 = FMSDS_Ptr->TimeRecoder - ET0_5;
				T3pre = T3;
				T3 = FMSDS_Ptr->VelocityXt;
				//ET0_5 = FMSDS_Ptr->TimeRecoder;
				countFlag = 0;
				rec[recH].trec[2] = T3;
				printf("T3 = %d\r\n", T3);
				printf("recH = %d, amlH = %d\r\n", recH, rec[recH].amlH);
				rec[recH].amlrec[rec[recH].amlH] = AgvInits;
				rec[recH].amlH++;
				rec[recH].amlrec[rec[recH].amlH] = Agv_MS_Center;
				rec[recH].amlH++;
				showTrec();
				recH++;
				rec[recH].amlH = 0;
				flaga = 0;
				
			}
		}
		else if((Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation))
		{
			
			if(Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation)
			{
				if(0 == countFlag)
				{
					//T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					//T1 = FMSDS_Ptr->VelocityXt;
					ET1 = SystemRunningTime;
					//T1 = ET1 - centerTT;
					T1pre = T1;
					T1 = FMSDS_Ptr->VelocityXt + FMSDS_Pre_Ptr->VelocityXt;
					countFlag = 1;
					flaga = 1;
					rec[recH].trec[0] = T1;
					flagt1 = 1;
					printf("T1 = %d\r\n", T1);
				}
				
			}
			else
			{
				if(0 == countFlag)
				{
					//T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					ET1 = SystemRunningTime;
					//T1 = ET1 - centerTT;
					T1pre = T1;
					T1 = FMSDS_Ptr->VelocityXt + FMSDS_Pre_Ptr->VelocityXt;
					countFlag = 2;
					flaga = 1;
					rec[recH].trec[0] = T1;
					flagt1 = 1;
					printf("T1 = %d\r\n", T1);
				}
				
			}
			
			
		}
		else if((Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_0_5 == FMSDS_Ptr->AgvMSLocation))
		{
			if(Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation)
			{
				if(1 == countFlag)
				{
					ET0_5 = SystemRunningTime;
					T2pre = T2;
					T2 = ET0_5 - ET1;
					//T2 = FMSDS_Ptr->TimeRecoder - ET1;
					//ET0_5 = FMSDS_Ptr->TimeRecoder;
					countFlag = 3;
					rec[recH].trec[1] = T2;
					//flagt2 = 1;
					printf("T2 = %d\r\n", T2);
				}
			}
			else
			{
				if(2 == countFlag)
				{
					ET0_5 = SystemRunningTime;
					T2pre = T2;
					T2 = ET0_5 - ET1;
					//T2 = FMSDS_Ptr->TimeRecoder - ET1;
					//ET0_5 = FMSDS_Ptr->TimeRecoder;
					countFlag = 3;
					rec[recH].trec[1] = T2;
					//flagt2 = 1;
					printf("T2 = %d\r\n", T2);
				}
			}
			
		}
		
	}


#endif

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
			/*
			else if(1 == flagt2)
			{
				flagt2 = 0;
				rec[recH].amlrec[rec[recH].amlH] = AgvInits;
				rec[recH].amlH++;
			}
			*/
		}
		
	}


	if(0 == acFlag)
	{
		if(T1pre > T1)
		{
			if((T1pre - T1) <= 0.2 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 4;
			}
			else if((T1pre - T1) <= 0.4 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 3;
			}
			else if((T1pre - T1) <= 0.6 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 2;
			}
		}
	}

#if 1

	if(acFlag)
	{
		if((SystemRunningTime - acTST > 3000) || (locRec3 != FMSDS_Ptr->AgvMSLocation))
		{
			acFlag = 0;
			acTST = 0;
		}
		else
		{
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				if(1 == lmflag)
				{
					lmSpeedbak = lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] - add;
				}
				else if(1 == rmflag)
				{
					rmSpeedbak = rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] - add;
				}
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				if(1 == lmflag)
				{
					lmSpeedbak = lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] - add;
				}
				else if(1 == rmflag)
				{
					rmSpeedbak = rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] - add;
				}	
			}
			
			
		}
	}

	if(0 == t2flag)
	{
		if(T2 > 5000)
		{
			t2flag = 1;
			locRec4 = FMSDS_Ptr->AgvMSLocation;
		}
	}
	

	if(1 == t2flag)
	{
		if(locRec3 != FMSDS_Ptr->AgvMSLocation)
		{
			t2flag = 0;
			acTST = 0;
		}
		else
		{
			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_2) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_1))
			{
				if(1 == lmflag)
				{
					lmSpeed = lmSpeedbak - 2;
				}
				else if(1 == rmflag)
				{
					rmSpeed = rmSpeedbak - 2;
				}
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_2))
			{
				if(1 == lmflag)
				{
					lmSpeed = lmSpeedbak - 2;
				}
				else if(1 == rmflag)
				{
					rmSpeed = rmSpeedbak - 2;
				}	
			}
		}
	}
	
#else
	if(acFlag)
	{
		if((SystemRunningTime - acTST > 5000) || (locRec3 != FMSDS_Ptr->AgvMSLocation))
		{
			acFlag = 0;
			acTST = 0;
		}
		else
		{
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
				{
					
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					
					if(AGV_Pat_Ptr->Midpoint > 0)
					{
						if(AGV_Pat_Ptr->Angle > 0)
						{
							//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
							lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] + 2;
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							if(AGV_Pat_Ptr->Angle >= -2)
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
							}
							else
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] + 2;
								
							}
						}
					}
					else if(0 == AGV_Pat_Ptr->Midpoint)
					{
						if(AGV_Pat_Ptr->Angle > 0)
						{
							//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
							lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] + 2;
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] + 2;
						}
					}
					else if(AGV_Pat_Ptr->Midpoint < 0)
					{
						
						if(AGV_Pat_Ptr->Angle > 0)
						{
							if(AGV_Pat_Ptr->Angle <= 2)
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
							}
							else
							{
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] + 2;
							}
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] + 2;
						}
					}

					
				}

				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
				{
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					
					if(AGV_Pat_Ptr->Midpoint > 0)
					{
						
						if(AGV_Pat_Ptr->Angle > 0)
						{
							//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
							lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] + 2;
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							if(AGV_Pat_Ptr->Angle >= -2)
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
							}
							else
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] + 2;
							}
						}
					}
					else if(0 == AGV_Pat_Ptr->Midpoint)
					{
						if(AGV_Pat_Ptr->Angle > 0)
						{
							//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
							lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] * 2;
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] + 2;
						}
					}
					else if(AGV_Pat_Ptr->Midpoint < 0)
					{
						if(AGV_Pat_Ptr->Angle > 0)
						{
							if(AGV_Pat_Ptr->Angle <= 2)
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
							}
							else
							{
								
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] + 2;
							}
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] + 2;
						}
					}
					
				}

				
			}
		}
	}

#endif

	
	
	
	damping_func(1000, gearRecod, lmSpeed, rmSpeed);
	
	
}

void scale_1_mode5(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, lmSpeed_pat = 0, rmSpeed_pat = 0;
	u32 centCount = 0;
	static u32 startCount = 0, enterTF = 0, exitTF = 0;
	u8 AgvGearS1CDLF[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec = AgvInits;
	static u32 T1 = 0, T2 = 0, T3 = 0, centerTT = 0, L1ET = 0, countFlag = 0, L0_5ET = 0, R1ET = 0, R0_5ET = 0;
	u32 OSC_TREC[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	static u8 OSC_Head = 0;
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AgvCent2Left == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			
			if(AGV_Pat_Ptr->Midpoint > 0)
			{
				ctrlParasPtr->comflag = 64111;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641111;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641112;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641113;
					if(AGV_Pat_Ptr->Angle >= -2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
						
					}
				}
			}
			else if(0 == AGV_Pat_Ptr->Midpoint)
			{
				ctrlParasPtr->comflag = 64112;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641121;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641122;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641123;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
			}
			else if(AGV_Pat_Ptr->Midpoint < 0)
			{
				ctrlParasPtr->comflag = 64113;
				
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641131;
					if(AGV_Pat_Ptr->Angle <= 2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
						
					}
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641132;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641133;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
			}

			
			/*
			if((-2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
			{
				rmSpeed_pat = 2;
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed_pat = 0;
			}
			*/
			
			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6412;
			
		}
		
		
		
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
		{
			ctrlParasPtr->comflag = 6421;
			
			if(AgvCent2Right == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			
			if(AGV_Pat_Ptr->Midpoint > 0)
			{
				ctrlParasPtr->comflag = 64211;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642111;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642112;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642113;
					if(AGV_Pat_Ptr->Angle >= -2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					}
				}
			}
			else if(0 == AGV_Pat_Ptr->Midpoint)
			{
				ctrlParasPtr->comflag = 64212;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642121;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642122;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642123;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
				}
			}
			else if(AGV_Pat_Ptr->Midpoint < 0)
			{
				ctrlParasPtr->comflag = 64213;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642131;
					if(AGV_Pat_Ptr->Angle <= 2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					}
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642132;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642133;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
				}
			}

			
			/*
			if((-2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
			{
				rmSpeed_pat = 2;
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed_pat = 0;
			}
			*/
			
			

			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6422;
			
		}


		#if 0
		if((2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
		{
			lmSpeed_pat = 2;
		}
		else if(0 == AGV_Pat_Ptr->Angle)
		{
			lmSpeed_pat = 0;
		}

		if(Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation)
		{
			if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
			{
				printf("***VelocityXt = %d\r\n", FMSDS_Ptr->VelocityXt);
			}
			
		}
		#endif
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;
		
		OSC_Head = 0;

		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(-1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 1;
			}
			
		}
		else if(AGV_Pat_Ptr->Midpoint < 0)
		{
			if(1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 1;
			}
			
		}

		
		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}


	if(locRec != FMSDS_Ptr->AgvMSLocation)
	{
		locRec = FMSDS_Ptr->AgvMSLocation;

		if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
		{
			if(0 == countFlag)
			{
				centerTT = FMSDS_Ptr->TimeRecoder;
				countFlag = 1;
			}
			else if(3 == countFlag)
			{
				T3 = FMSDS_Ptr->TimeRecoder - L0_5ET;
				L0_5ET = FMSDS_Ptr->TimeRecoder;
				countFlag = 0;
				printf("T3 = %d\r\n", T3);
			}
		}
		else if((Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation))
		{
			if(Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation)
			{
				if(1 == countFlag)
				{
					T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					L1ET = FMSDS_Ptr->TimeRecoder;
					countFlag = 2;
					printf("T1 = %d\r\n", T1);
				}

			}
			else
			{
				if(1 == countFlag)
				{
					T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					R1ET = FMSDS_Ptr->TimeRecoder;
					countFlag = 2;
					printf("T1 = %d\r\n", T1);
				}
			}
			
		}
		else if((Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_0_5 == FMSDS_Ptr->AgvMSLocation))
		{
			if(Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation)
			{
				if(2 == countFlag)
				{
					T2 = FMSDS_Ptr->TimeRecoder - L1ET;
					L0_5ET = FMSDS_Ptr->TimeRecoder;
					countFlag = 3;
					printf("T2 = %d\r\n", T2);
				}
			}
			else
			{
				if(2 == countFlag)
				{
					T2 = FMSDS_Ptr->TimeRecoder - R1ET;
					R0_5ET = FMSDS_Ptr->TimeRecoder;
					countFlag = 3;
					printf("T2 = %d\r\n", T2);
				}
			}
		}
		
	}
	
	damping_func(1000, gearRecod, lmSpeed, rmSpeed);
	//set_duty(lmSpeed, rmSpeed);
	
	
}

void scale_1_mode6(u8 gear)
{
	static u8 lmSpeed = 0, rmSpeed = 0, lmSpeed_pat = 0, rmSpeed_pat = 0, flaga = 0, flagt1 = 0, flagt2 = 0, acFlag = 0;
	u32 centCount = 0;
	static u32 startCount = 0, enterTF = 0, exitTF = 0;
	u8 AgvGearS1CDLF[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec = AgvInits, locRec2 = AgvInits, locRec3 = AgvInits;
	static u32 T1 = 0, T2 = 0, T3 = 0, centerTT = 0, ET1 = 0, countFlag = 0, ET0_5 = 0, T1pre = 0, T2pre = 0, T3pre = 0, acTST = 0;
	
	
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AgvCent2Left == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			
			if(AGV_Pat_Ptr->Midpoint > 0)
			{
				ctrlParasPtr->comflag = 64111;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641111;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641112;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641113;
					if(AGV_Pat_Ptr->Angle >= -2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
						
					}
				}
			}
			else if(0 == AGV_Pat_Ptr->Midpoint)
			{
				ctrlParasPtr->comflag = 64112;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641121;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641122;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641123;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
			}
			else if(AGV_Pat_Ptr->Midpoint < 0)
			{
				ctrlParasPtr->comflag = 64113;
				
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641131;
					if(AGV_Pat_Ptr->Angle <= 2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
						
					}
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641132;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641133;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
			}

			
			/*
			if((-2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
			{
				rmSpeed_pat = 2;
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed_pat = 0;
			}
			*/
			
			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6412;
			
		}
		
		
		
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
		{
			ctrlParasPtr->comflag = 6421;
			
			if(AgvCent2Right == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
			
			if(AGV_Pat_Ptr->Midpoint > 0)
			{
				ctrlParasPtr->comflag = 64211;
				
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642111;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642112;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642113;
					if(AGV_Pat_Ptr->Angle >= -2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					}
				}
			}
			else if(0 == AGV_Pat_Ptr->Midpoint)
			{
				ctrlParasPtr->comflag = 64212;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642121;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642122;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642123;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
				}
			}
			else if(AGV_Pat_Ptr->Midpoint < 0)
			{
				ctrlParasPtr->comflag = 64213;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642131;
					if(AGV_Pat_Ptr->Angle <= 2)
					{
						rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					}
					else
					{
						
						lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					}
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642132;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642133;
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
				}
			}

			
			/*
			if((-2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
			{
				rmSpeed_pat = 2;
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed_pat = 0;
			}
			*/
			
			

			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6422;
			
		}


		#if 0
		if((2 == AGV_Pat_Ptr->Angle) && (0 == AGV_Pat_Ptr->Midpoint))
		{
			lmSpeed_pat = 2;
		}
		else if(0 == AGV_Pat_Ptr->Angle)
		{
			lmSpeed_pat = 0;
		}

		if(Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation)
		{
			if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
			{
				printf("***VelocityXt = %d\r\n", FMSDS_Ptr->VelocityXt);
			}
			
		}
		#endif
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
		rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;

		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(-1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 1;
			}
			
		}
		else if(AGV_Pat_Ptr->Midpoint < 0)
		{
			if(1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - 1;
			}
			
		}

		
		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}
	
	

#if 0
	if(locRec != FMSDS_Ptr->AgvMSLocation)
	{
		locRec = FMSDS_Ptr->AgvMSLocation;

		if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
		{
			if(0 == countFlag)
			{
				centerTT = FMSDS_Ptr->TimeRecoder;
				countFlag = 1;
				rec[recH].amlH = 0;
				rec[recH].amlrec[rec[recH].amlH] = Agv_MS_Center;
				rec[recH].amlH++;
				flaga = 1;
			}
			else if(4 == countFlag)
			{
				T3 = FMSDS_Ptr->TimeRecoder - ET0_5;
				//ET0_5 = FMSDS_Ptr->TimeRecoder;
				countFlag = 0;
				rec[recH].trec[2] = T3;
				printf("T3 = %d\r\n", T3);
				printf("recH = %d, amlH = %d\r\n", recH, rec[recH].amlH);
				showTrec();
				recH++;
				rec[recH].amlH = 0;
				flaga = 0;
				
			}
		}
		else if((Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation))
		{
			if(Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation)
			{
				if(1 == countFlag)
				{
					T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					ET1 = FMSDS_Ptr->TimeRecoder;
					countFlag = 2;
					rec[recH].trec[0] = T1;
					printf("T1 = %d\r\n", T1);
				}
			}
			else
			{
				if(1 == countFlag)
				{
					T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					ET1 = FMSDS_Ptr->TimeRecoder;
					countFlag = 3;
					rec[recH].trec[0] = T1;
					printf("T1 = %d\r\n", T1);
				}
			}
			
			
		}
		else if((Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_0_5 == FMSDS_Ptr->AgvMSLocation))
		{
			if(Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation)
			{
				if(2 == countFlag)
				{
					T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					ET1 = FMSDS_Ptr->TimeRecoder;
					countFlag = 4;
					rec[recH].trec[0] = T1;
					printf("T1 = %d\r\n", T1);
				}
			}
			else
			{
				if(3 == countFlag)
				{
					T2 = FMSDS_Ptr->TimeRecoder - ET1;
					ET0_5 = FMSDS_Ptr->TimeRecoder;
					countFlag = 4;
					rec[recH].trec[1] = T2;
					printf("T2 = %d\r\n", T2);
				}
			}
			
		}
		
	}
#else

	if(locRec != FMSDS_Ptr->AgvMSLocation)
	{
		locRec = FMSDS_Ptr->AgvMSLocation;

		if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
		{
			if(0 == countFlag)
			{
				//centerTT = FMSDS_Ptr->TimeRecoder;
				centerTT = SystemRunningTime;
				rec[recH].amlH = 0;
			}
			else if(3 == countFlag)
			{
				//T3 = FMSDS_Ptr->TimeRecoder - ET0_5;
				T3pre = T3;
				T3 = FMSDS_Ptr->VelocityXt;
				//ET0_5 = FMSDS_Ptr->TimeRecoder;
				countFlag = 0;
				rec[recH].trec[2] = T3;
				printf("T3 = %d\r\n", T3);
				printf("recH = %d, amlH = %d\r\n", recH, rec[recH].amlH);
				rec[recH].amlrec[rec[recH].amlH] = AgvInits;
				rec[recH].amlH++;
				rec[recH].amlrec[rec[recH].amlH] = Agv_MS_Center;
				rec[recH].amlH++;
				showTrec();
				recH++;
				rec[recH].amlH = 0;
				flaga = 0;
				
			}
		}
		else if((Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation))
		{
			
			if(Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation)
			{
				if(0 == countFlag)
				{
					//T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					//T1 = FMSDS_Ptr->VelocityXt;
					ET1 = SystemRunningTime;
					//T1 = ET1 - centerTT;
					T1pre = T1;
					T1 = FMSDS_Ptr->VelocityXt + FMSDS_Pre_Ptr->VelocityXt;
					countFlag = 1;
					flaga = 1;
					rec[recH].trec[0] = T1;
					flagt1 = 1;
					printf("T1 = %d\r\n", T1);
				}
				
			}
			else
			{
				if(0 == countFlag)
				{
					//T1 = FMSDS_Ptr->TimeRecoder - centerTT;
					ET1 = SystemRunningTime;
					//T1 = ET1 - centerTT;
					T1pre = T1;
					T1 = FMSDS_Ptr->VelocityXt + FMSDS_Pre_Ptr->VelocityXt;
					countFlag = 2;
					flaga = 1;
					rec[recH].trec[0] = T1;
					flagt1 = 1;
					printf("T1 = %d\r\n", T1);
				}
				
			}
			
			
		}
		else if((Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_0_5 == FMSDS_Ptr->AgvMSLocation))
		{
			if(Agv_MS_Left_0_5 == FMSDS_Ptr->AgvMSLocation)
			{
				if(1 == countFlag)
				{
					ET0_5 = SystemRunningTime;
					T2pre = T2;
					T2 = ET0_5 - ET1;
					//T2 = FMSDS_Ptr->TimeRecoder - ET1;
					//ET0_5 = FMSDS_Ptr->TimeRecoder;
					countFlag = 3;
					rec[recH].trec[1] = T2;
					//flagt2 = 1;
					printf("T2 = %d\r\n", T2);
				}
			}
			else
			{
				if(2 == countFlag)
				{
					ET0_5 = SystemRunningTime;
					T2pre = T2;
					T2 = ET0_5 - ET1;
					//T2 = FMSDS_Ptr->TimeRecoder - ET1;
					//ET0_5 = FMSDS_Ptr->TimeRecoder;
					countFlag = 3;
					rec[recH].trec[1] = T2;
					//flagt2 = 1;
					printf("T2 = %d\r\n", T2);
				}
			}
			
		}
		
	}


#endif

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
			/*
			else if(1 == flagt2)
			{
				flagt2 = 0;
				rec[recH].amlrec[rec[recH].amlH] = AgvInits;
				rec[recH].amlH++;
			}
			*/
		}
		
	}


	if(0 == acFlag)
	{
		if(T1pre > T1)
		{
			if((T1pre - T1) > 0.4 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
		}
	}
	
	
	if(acFlag)
	{
		if((SystemRunningTime - acTST > 5000) || (locRec3 != FMSDS_Ptr->AgvMSLocation))
		{
			acFlag = 0;
			acTST = 0;
		}
		else
		{
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
				{
					
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					
					if(AGV_Pat_Ptr->Midpoint > 0)
					{
						if(AGV_Pat_Ptr->Angle > 0)
						{
							//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
							lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] * 2;
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							if(AGV_Pat_Ptr->Angle >= -2)
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
							}
							else
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] * 2;
								
							}
						}
					}
					else if(0 == AGV_Pat_Ptr->Midpoint)
					{
						if(AGV_Pat_Ptr->Angle > 0)
						{
							//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
							lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] * 2;
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] * 2;
						}
					}
					else if(AGV_Pat_Ptr->Midpoint < 0)
					{
						
						if(AGV_Pat_Ptr->Angle > 0)
						{
							if(AGV_Pat_Ptr->Angle <= 2)
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
							}
							else
							{
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] * 2;
							}
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] * 2;
						}
					}

					
				}

				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
				{
					rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
					
					if(AGV_Pat_Ptr->Midpoint > 0)
					{
						
						if(AGV_Pat_Ptr->Angle > 0)
						{
							//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
							lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] * 2;
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							if(AGV_Pat_Ptr->Angle >= -2)
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
							}
							else
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] * 2;
							}
						}
					}
					else if(0 == AGV_Pat_Ptr->Midpoint)
					{
						if(AGV_Pat_Ptr->Angle > 0)
						{
							//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
							lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] * 2;
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] * 2;
						}
					}
					else if(AGV_Pat_Ptr->Midpoint < 0)
					{
						if(AGV_Pat_Ptr->Angle > 0)
						{
							if(AGV_Pat_Ptr->Angle <= 2)
							{
								rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod];
							}
							else
							{
								
								lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] * 2;
							}
						}
						else if(0 == AGV_Pat_Ptr->Angle)
						{
						}
						else if(AGV_Pat_Ptr->Angle < 0)
						{
							rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] * 2;
						}
					}
					
				}

				
			}
		}
	}
	
	damping_func(1000, gearRecod, lmSpeed, rmSpeed);
	
		
}


void Get_T1(Trec *now)
{
	static u32 T1 = 0;
	
	if((Agv_MS_Left_1 == FMSDS_Ptr->AgvMSLocation) || (Agv_MS_Right_1 == FMSDS_Ptr->AgvMSLocation))
	{
		T1 = FMSDS_Ptr->VelocityXt;
		now->T1 = T1;
		now->T1_update = 1;
		
		//rec[recH].trec[0] = T1;
		printf("T1 = %d\r\n", T1);
		
	}
	
}

void T_monitor(Trec *now)
{
	static u8 flaga = 0, flagt1 = 0, flagt2 = 0, acFlag = 0, t2flag = 0;
	static u32 T1 = 0, T2 = 0, T3 = 0, centerTT = 0, ET1 = 0, countFlag = 0, ET0_5 = 0, T1pre = 0, T2pre = 0, T3pre = 0, acTST = 0;
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

				centerTT = SystemRunningTime;

				now->T3_update = 1;
				now->T3 = T3;
				now->All_update = 1;
				
				T1pre = T1;
				T2pre = T2;
				T3pre = T3;

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
	
	for(cir = 0; cir <= 10; cir++)
	{
		printf("%d: tim = %d, duty = %d, res = %d\r\n", cir, arr[cir].timRec, arr[cir].duty, arr[cir].result);
	}
	
	
}


void T1_Adapter(u8 *T1LSpeed, u8 *T1RSpeed)
{

	static Trec Tnow, Tpre;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, flag = 0, re = 0;

	static Agv_MS_Location locRec = AgvInits, maxRec = AgvInits;

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

		if((re >= 0) && (re <= 10))
		{
			Tnow.T1_update = 2;
			

			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				T1RSpeedin = adaptInfo[re].duty;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeedin = adaptInfo[re].duty;
				T1RSpeedin = 0;
			}
			flag = 1;
			locRec = FMSDS_Ptr->AgvMSLocation;
			
			recT1Tim = SystemRunningTime;
			
			printf("re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		if(SystemRunningTime - recT1Tim >= 4000)
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
			
			if((maxRec <= Agv_MS_Left_1_5) || (maxRec >= Agv_MS_Right_1_5))
			{
				adaptInfo[re].result = Small;
				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Small\r\n", re);
				}
				
				if(adaptInfo[re].duty < 10)
				{
					adaptInfo[re].duty++;
				}
				else
				{
					if(adaptInfo[re].goodDuty != 0)
					{
						adaptInfo[re].duty = adaptInfo[re].goodDuty;
					}
				}
			}
			else
			{
				adaptInfo[re].result = Good;
				if(1 == flag)
				{
					flag = 2;
					printf("re = %d: Good****************\r\n\r\n", re);
				}
				
			}

			show_adapt_info(adaptInfo);
			printf("\r\n");
		}
	}
	
	if(1 == Tnow.All_update)
	{
		Tpre = Tnow;

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

	static Trec Tnow, Tpre;

	static u8 T1LSpeedin = 0, T1RSpeedin = 0, flag = 0, re = 0;

	static Agv_MS_Location locRec = AgvInits, maxRec = AgvInits;

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

		if((re >= 0) && (re <= 10))
		{
			Tnow.T1_update = 2;
			

			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeedin = 0;
				T1RSpeedin = adaptInfoB[re].duty;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeedin = adaptInfoB[re].duty;
				T1RSpeedin = 0;
			}
			flag = 1;
			locRec = FMSDS_Ptr->AgvMSLocation;
			
			recT1Tim = SystemRunningTime;
			
			printf("B re = %d, T1LSpeedin = %d, T1RSpeedin = %d\r\n", re, T1LSpeedin, T1RSpeedin);
		}
		
		
	}
	
	
	if(2 == Tnow.T1_update)
	{
		if(SystemRunningTime - recT1Tim >= 4000)
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
			
			if((maxRec <= Agv_MS_Left_1_5) || (maxRec >= Agv_MS_Right_1_5))
			{
				adaptInfoB[re].result = Small;
				if(1 == flag)
				{
					flag = 2;
					printf("B re = %d: Small\r\n", re);
				}

				if(adaptInfoB[re].duty < 10)
				{
					adaptInfoB[re].duty++;
				}
				else
				{
					if(adaptInfoB[re].goodDuty != 0)
					{
						adaptInfoB[re].duty = adaptInfoB[re].goodDuty;
					}
					
				}
			}
			else
			{
				adaptInfoB[re].result = Good;
				if(1 == flag)
				{
					flag = 2;
					printf("B re = %d: Good****************\r\n\r\n", re);
				}
				
				adaptInfoB[re].goodDuty = adaptInfoB[re].duty;
			}

			show_adapt_info(adaptInfoB);
			printf("\r\n");
		}
	}
	
	if(1 == Tnow.All_update)
	{
		Tpre = Tnow;

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




void scale_1_mode7(u8 gear)
{
	static u8  lmSpeed_pat = 0, rmSpeed_pat = 0, lmSpeedbak = 0, rmSpeedbak = 0, lmflag = 0, rmflag = 0;
	u32 centCount = 0;
	static u32 startCount = 0, enterTF = 0, exitTF = 0;
	u8 AgvGearS1CDLF[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec3 = AgvInits, locRec4 = AgvInits;
	u8 add = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0;
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;

	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AgvCent2Left == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
						
			if(AGV_Pat_Ptr->Midpoint > 0)	// 这种情况在此处不可能发生
			{
				ctrlParasPtr->comflag = 64111;
				
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641111;
					lmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641112;
					lmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641113;

					if((AGV_Pat_Ptr->Angle >= -4) || (AGV_Pat_Ptr->Angle <= -2))
					{
						
					}
					else
					{
						rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
					}
				}
			}
			else if(0 == AGV_Pat_Ptr->Midpoint)
			{
				ctrlParasPtr->comflag = 64112;
				if(AGV_Pat_Ptr->Angle > 0)		// 这种情况在此处不可能发生
				{
					ctrlParasPtr->comflag = 641121;
					//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
					lmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641122;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641123;
					rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				
			}
			else if(AGV_Pat_Ptr->Midpoint < 0)
			{
				ctrlParasPtr->comflag = 64113;
				
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 641131;
					if((AGV_Pat_Ptr->Angle >= 2) || (AGV_Pat_Ptr->Angle <= 4))
					{
						
					}
					else
					{
						lmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
						
					}
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 641132;
					rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 641133;
					rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
				}
				
			}
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6412;
			rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		}
		
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
		{
			ctrlParasPtr->comflag = 6421;
			
			if(AgvCent2Right == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			
			if(AGV_Pat_Ptr->Midpoint > 0)
			{
				ctrlParasPtr->comflag = 64211;
				
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642111;
						
					lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					lmflag = 1;
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642112;
					lmflag = 0;
					rmflag = 0;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642113;
					if(AGV_Pat_Ptr->Angle >= -2)
					{
						
						lmflag = 0;
						rmflag = 0;
					}
					else
					{
						rmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
						rmflag = 1;
					}
				}
			}
			else if(0 == AGV_Pat_Ptr->Midpoint)
			{
				ctrlParasPtr->comflag = 64212;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642121;
						
					lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					lmflag = 1;
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642122;
					lmflag = 0;
					rmflag = 0;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642123;
					rmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					rmflag = 0;
				}
			}
			else if(AGV_Pat_Ptr->Midpoint < 0)
			{
				ctrlParasPtr->comflag = 64213;
				if(AGV_Pat_Ptr->Angle > 0)
				{
					ctrlParasPtr->comflag = 642131;
					if(AGV_Pat_Ptr->Angle <= 2)
					{
						
						lmflag = 0;
						rmflag = 0;
					}
					else
					{
						
						lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
						lmflag = 1;
					}
				}
				else if(0 == AGV_Pat_Ptr->Angle)
				{
					ctrlParasPtr->comflag = 642132;
					lmflag = 0;
					rmflag = 0;
				}
				else if(AGV_Pat_Ptr->Angle < 0)
				{
					ctrlParasPtr->comflag = 642133;
					rmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
					rmflag = 1;
				}
			}
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6422;
			lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		}


		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;

		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(-1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = 1;
			}
			
		}
		else if(AGV_Pat_Ptr->Midpoint < 0)
		{
			if(1 == AGV_Pat_Ptr->Angle)
			{
				rmSpeed = 1;
			}
			
		}
		
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}
	
	

#if 0
	


	if(0 == acFlag)
	{
		if(T1pre > T1)
		{
			if((T1pre - T1) <= 0.2 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 4;
			}
			else if((T1pre - T1) <= 0.4 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 3;
			}
			else if((T1pre - T1) <= 0.6 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 2;
			}
		}
	}

#endif
	

#if 0

	if(acFlag)
	{
		if((SystemRunningTime - acTST > 3000) || (locRec3 != FMSDS_Ptr->AgvMSLocation))
		{
			acFlag = 0;
			acTST = 0;
		}
		else
		{
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				if(1 == lmflag)
				{
					lmSpeedbak = lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] - add;
				}
				else if(1 == rmflag)
				{
					rmSpeedbak = rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] - add;
				}
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				if(1 == lmflag)
				{
					lmSpeedbak = lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] - add;
				}
				else if(1 == rmflag)
				{
					rmSpeedbak = rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] - add;
				}	
			}
			
			
		}
	}

	if(0 == t2flag)
	{
		if(T2 > 5000)
		{
			t2flag = 1;
			locRec4 = FMSDS_Ptr->AgvMSLocation;
		}
	}
	

	if(1 == t2flag)
	{
		if(locRec3 != FMSDS_Ptr->AgvMSLocation)
		{
			t2flag = 0;
			acTST = 0;
		}
		else
		{
			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_2) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_1))
			{
				if(1 == lmflag)
				{
					lmSpeed = lmSpeedbak - 2;
				}
				else if(1 == rmflag)
				{
					rmSpeed = rmSpeedbak - 2;
				}
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_2))
			{
				if(1 == lmflag)
				{
					lmSpeed = lmSpeedbak - 2;
				}
				else if(1 == rmflag)
				{
					rmSpeed = rmSpeedbak - 2;
				}	
			}
		}
	}
	

#endif	
	
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}

void scale_1_mode8(u8 gear)
{
	static u8  lmSpeed_pat = 0, rmSpeed_pat = 0, lmSpeedbak = 0, rmSpeedbak = 0, lmflag = 0, rmflag = 0;
	u32 centCount = 0;
	static u32 startCount = 0, enterTF = 0, exitTF = 0;
	u8 AgvGearPatAngl[50] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec3 = AgvInits, locRec4 = AgvInits;
	u8 add = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0;
	u8 maxLimt = 4;
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;

	
	if(AGV_Pat_Ptr->Midpoint > 0)
	{
		ctrlParasPtr->comflag = 641;

		if((AGV_Pat_Ptr->Angle >= -maxLimt) && (AGV_Pat_Ptr->Angle <= maxLimt))
		{
			ctrlParasPtr->comflag = 6411;
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 64111;
				lmSpeed = AGV_Pat_Ptr->Angle;
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				ctrlParasPtr->comflag = 64112;
				lmSpeed = 1;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 64113;
				
				if((AGV_Pat_Ptr->Angle <= -2) && (AGV_Pat_Ptr->Midpoint <= 2))
				{
					rmSpeed = -AGV_Pat_Ptr->Angle;
				}
				
			}
		}
		else
		{
			ctrlParasPtr->comflag = 6411;
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 64111;
				lmSpeed = maxLimt;
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 64112;
				
				rmSpeed = maxLimt;
			}
		}
		
		
	}
	else if(0 == AGV_Pat_Ptr->Midpoint)
	{
		ctrlParasPtr->comflag = 6412;
		if(AGV_Pat_Ptr->Angle > 0)		
		{
			ctrlParasPtr->comflag = 64121;
			//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = AGV_Pat_Ptr->Angle;
		}
		else if(0 == AGV_Pat_Ptr->Angle)
		{
			ctrlParasPtr->comflag = 64122;
		}
		else if(AGV_Pat_Ptr->Angle < 0)
		{
			ctrlParasPtr->comflag = 64123;
			rmSpeed = -AGV_Pat_Ptr->Angle;
		}
		
	}
	else if(AGV_Pat_Ptr->Midpoint < 0)
	{
		ctrlParasPtr->comflag = 6413;

		if((AGV_Pat_Ptr->Angle >= -maxLimt) && (AGV_Pat_Ptr->Angle <= maxLimt))
		{
			ctrlParasPtr->comflag = 64131;
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 641311;
				if((AGV_Pat_Ptr->Angle >= 2) && (AGV_Pat_Ptr->Midpoint >= -2))
				{
					ctrlParasPtr->comflag = 6413111;
					lmSpeed = AGV_Pat_Ptr->Angle;
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				ctrlParasPtr->comflag = 641312;
				rmSpeed = 1;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 641313;
				rmSpeed = -AGV_Pat_Ptr->Angle;
			}
		}
		else
		{
			ctrlParasPtr->comflag = 64132;
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 641321;
				lmSpeed = maxLimt;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 641322;
				rmSpeed = maxLimt;
			}
		}
		
		
		
	}
	
	
	

#if 0
	


	if(0 == acFlag)
	{
		if(T1pre > T1)
		{
			if((T1pre - T1) <= 0.2 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 4;
			}
			else if((T1pre - T1) <= 0.4 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 3;
			}
			else if((T1pre - T1) <= 0.6 * T1pre)
			{
				acFlag = 1;
				acTST = SystemRunningTime;
				locRec3 = FMSDS_Ptr->AgvMSLocation;
				add = 2;
			}
		}
	}

#endif
	

#if 0

	if(acFlag)
	{
		if((SystemRunningTime - acTST > 3000) || (locRec3 != FMSDS_Ptr->AgvMSLocation))
		{
			acFlag = 0;
			acTST = 0;
		}
		else
		{
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				if(1 == lmflag)
				{
					lmSpeedbak = lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] - add;
				}
				else if(1 == rmflag)
				{
					rmSpeedbak = rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation] - add;
				}
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				if(1 == lmflag)
				{
					lmSpeedbak = lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] - add;
				}
				else if(1 == rmflag)
				{
					rmSpeedbak = rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5] - add;
				}	
			}
			
			
		}
	}

	if(0 == t2flag)
	{
		if(T2 > 5000)
		{
			t2flag = 1;
			locRec4 = FMSDS_Ptr->AgvMSLocation;
		}
	}
	

	if(1 == t2flag)
	{
		if(locRec3 != FMSDS_Ptr->AgvMSLocation)
		{
			t2flag = 0;
			acTST = 0;
		}
		else
		{
			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_2) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_1))
			{
				if(1 == lmflag)
				{
					lmSpeed = lmSpeedbak - 2;
				}
				else if(1 == rmflag)
				{
					rmSpeed = rmSpeedbak - 2;
				}
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_2))
			{
				if(1 == lmflag)
				{
					lmSpeed = lmSpeedbak - 2;
				}
				else if(1 == rmflag)
				{
					rmSpeed = rmSpeedbak - 2;
				}	
			}
		}
	}
	

#endif	
	
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}

void scale_1_mode9(u8 gear)
{
	u8 AgvGearPatAngl[50] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec3 = AgvInits, locRec4 = AgvInits;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0;
	u8 maxLimt = 16;
	u32 T1 = 0, T2 = 0, T3 = 0;
	u8 T1Speed = 0, T2Speed = 0, T3Speed = 0;
	static u8 T1LSpeed = 0, T2LSpeed = 0, T3LSpeed = 0, T1RSpeed = 0, T2RSpeed = 0, T3RSpeed = 0, T2LSpeedRec = 0, T2RSpeedRec = 0;
	// 普通模式,偏差在1格之内调整
	static Trec Tnow, Tpre;
	
	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if(AGV_Pat_Ptr->Midpoint > 0)
	{
		ctrlParasPtr->comflag = 641;

		if((AGV_Pat_Ptr->Angle >= -maxLimt) && (AGV_Pat_Ptr->Angle <= maxLimt))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 64111;
				lmSpeed = AGV_Pat_Ptr->Angle;
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				ctrlParasPtr->comflag = 64112;
				//lmSpeed = 1;
				lmSpeed = 2;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 64113;
				
				if((AGV_Pat_Ptr->Angle <= -2) && (AGV_Pat_Ptr->Midpoint <= 2))
				{
					rmSpeed = -AGV_Pat_Ptr->Angle;
				}
				
			}
		}
		else
		{
			ctrlParasPtr->comflag = 6411;
			/*
			if(AGV_Pat_Ptr->Angle < -maxLimt)
			{
				if(AGV_Pat_Ptr->Midpoint <= 2)
				{
					ctrlParasPtr->comflag = 64111;
					rmSpeed = maxLimt / 2;
				}
			}
			else if(AGV_Pat_Ptr->Angle > maxLimt)
			{
				ctrlParasPtr->comflag = 64112;
				lmSpeed = maxLimt / 2;
			}
			*/
			
			ctrlParasPtr->FSflag = 0;
			printf("FMSD_Hex = %x, RMSD_Hex = %x\r\n", FMSDS_Ptr->MSD_Hex, RMSDS_Ptr->MSD_Hex);
			
		}
		
		
	}
	else if(0 == AGV_Pat_Ptr->Midpoint)
	{
		ctrlParasPtr->comflag = 6412;
		if(AGV_Pat_Ptr->Angle > 0)		
		{
			ctrlParasPtr->comflag = 64121;
			//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = 2;
		}
		else if(0 == AGV_Pat_Ptr->Angle)
		{
			ctrlParasPtr->comflag = 64122;
		}
		else if(AGV_Pat_Ptr->Angle < 0)
		{
			ctrlParasPtr->comflag = 64123;
			rmSpeed = 2;
		}
		
	}
	else if(AGV_Pat_Ptr->Midpoint < 0)
	{
		ctrlParasPtr->comflag = 6413;

		if((AGV_Pat_Ptr->Angle >= -maxLimt) && (AGV_Pat_Ptr->Angle <= maxLimt))
		{
			ctrlParasPtr->comflag = 64131;
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 641311;
				if((AGV_Pat_Ptr->Angle >= 2) && (AGV_Pat_Ptr->Midpoint >= -2))
				{
					ctrlParasPtr->comflag = 6413111;
					lmSpeed = AGV_Pat_Ptr->Angle;
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				ctrlParasPtr->comflag = 641312;
				//rmSpeed = 1;
				rmSpeed = 2;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 641313;
				rmSpeed = -AGV_Pat_Ptr->Angle;
			}
		}
		else
		{
			ctrlParasPtr->comflag = 64132;
			/*
			if(AGV_Pat_Ptr->Angle < -maxLimt)
			{
				ctrlParasPtr->comflag = 641321;
				rmSpeed = maxLimt / 2;
			}
			else if(AGV_Pat_Ptr->Angle > maxLimt)
			{
				if(AGV_Pat_Ptr->Midpoint >= -2)
				{
					ctrlParasPtr->comflag = 641322;
					lmSpeed = maxLimt / 2;
					
				}
			}
			*/
			ctrlParasPtr->FSflag = 0;
			
		}
		
		
	}
	
	T_monitor(&Tnow);

#if 1

	if(1 == Tnow.T1_update)
	{
		//u32 baseTime = 1200;
		
		Tnow.T1_update = 2;
		
		printf("Tnow.T1 = %d\r\n", Tnow.T1);
		
		if(Tnow.T1 < 200)			// 为原本的80%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 3;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 3;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(Tnow.T1 < 600)	// 为原本的60%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 2;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 2;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(Tnow.T1 < 1200)	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 1;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 1;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		
		
		
		
	}

#else
	if(1 == Tnow.T1_update)
	{
		Tnow.T1_update = 2;
		printf("Tpre.T1 = %d, Tnow.T1 = %d\r\n", Tpre.T1, Tnow.T1);
		if(Tpre.T1 > Tnow.T1)
		{
			if((Tpre.T1 - Tnow.T1) <= 0.2 * Tpre.T1)			// 为原本的80%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 1;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 1;
				}
				
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.4 * Tpre.T1)	// 为原本的60%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 2;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 2;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.6 * Tpre.T1)	// 为原本的40%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 3;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 3;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
	}

#endif	

	if(2 == Tnow.T1_update)
	{
		if(locRec3 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1 AngleDirection = %d\r\n", AGV_Pat_Ptr->AngleDirection);
			if(AGV_Pat_Ptr->AngleDirection < 0)
			//if((AGV_Pat_Ptr->Angle >= -4) && (AGV_Pat_Ptr->Angle <= 4))
			{
				printf("T1*******\r\n");
				Tnow.T1_update = 0;
				T1LSpeed = 0;
				T1RSpeed = 0;
			}
		}
	}

#if 0

	if(1 == Tnow.T2_update)
	{
		if(Tnow.T2 > 15000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 6;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 6;
			}
			
			Tnow.T2_update = 2;
			
		}
		else if(Tnow.T2 > 10000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 4;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 4;
			}
			
			Tnow.T2_update = 2;
			
		}
		else if(Tnow.T2 > 5000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 2;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 2;
			}
			
			Tnow.T2_update = 2;
			
		}

		if((T2LSpeedRec != T2LSpeed) || (T2RSpeedRec != T2RSpeed))
		{
			T2LSpeedRec = T2LSpeed;
			T2RSpeedRec = T2RSpeed;
			printf("T2LSpeed = %d, T2RSpeed = %d\r\n", T2LSpeed, T2RSpeed);
		}
		
	}
	
	
	if(2 == Tnow.T2_update)
	{
		Tnow.T2_update = 1;
		
		if(locRec4 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec4 = FMSDS_Ptr->AgvMSLocation;
			printf("T2 = %d\r\n", Tnow.T2);
			printf("T2 AngleDirection = %d\r\n", AGV_Pat_Ptr->AngleDirection);
			
			if(AGV_Pat_Ptr->AngleDirection < 0)
			{
				printf("T2*******\r\n");
				Tnow.T2_update = 3;
				T2LSpeed = 0;
				T2RSpeed = 0;
				locRec4 = AgvInits;
			}
		}
		
	}
	
#endif

	if(1 == Tnow.All_update)
	{
		Tpre = Tnow;

		Tnow.All_update = 0;
		Tnow.T1 = 0;
		Tnow.T1_update = 0;
		Tnow.T2 = 0;
		Tnow.T2_update = 0;
		Tnow.T3 = 0;
		Tnow.T3_update = 0;
	}
	
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - T1LSpeed - T2LSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - T1RSpeed - T2RSpeed;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}

void scale_1_mode9_back(u8 gear)
{
	u8 AgvGearPatAngl[50] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec3 = AgvInits, locRec4 = AgvInits;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0;
	u8 maxLimt = 8;
	u32 T1 = 0, T2 = 0, T3 = 0;
	u8 T1Speed = 0, T2Speed = 0, T3Speed = 0;
	static u8 T1LSpeed = 0, T2LSpeed = 0, T3LSpeed = 0, T1RSpeed = 0, T2RSpeed = 0, T3RSpeed = 0, T2LSpeedRec = 0, T2RSpeedRec = 0;
	// 普通模式,偏差在1格之内调整
	static Trec Tnow, Tpre;
	
	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if(AGV_Pat_Ptr->Midpoint > 0)
	{
		ctrlParasPtr->comflag = 641;

		if((AGV_Pat_Ptr->Angle >= -maxLimt) && (AGV_Pat_Ptr->Angle <= maxLimt))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 64111;
				lmSpeed = AGV_Pat_Ptr->Angle;
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				ctrlParasPtr->comflag = 64112;
				//lmSpeed = 1;
				lmSpeed = 2;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 64113;
				
				if((AGV_Pat_Ptr->Angle <= -2) && (AGV_Pat_Ptr->Midpoint <= 2))
				{
					rmSpeed = -AGV_Pat_Ptr->Angle;
				}
				
			}
		}
		else
		{
			ctrlParasPtr->comflag = 6411;
			/*
			if(AGV_Pat_Ptr->Angle < -maxLimt)
			{
				if(AGV_Pat_Ptr->Midpoint <= 2)
				{
					ctrlParasPtr->comflag = 64111;
					rmSpeed = maxLimt / 2;
				}
			}
			else if(AGV_Pat_Ptr->Angle > maxLimt)
			{
				ctrlParasPtr->comflag = 64112;
				lmSpeed = maxLimt / 2;
			}
			*/
			
			ctrlParasPtr->BSflag = 0;
			
		}
		
		
	}
	else if(0 == AGV_Pat_Ptr->Midpoint)
	{
		ctrlParasPtr->comflag = 6412;
		if(AGV_Pat_Ptr->Angle > 0)		
		{
			ctrlParasPtr->comflag = 64121;
			//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = 2;
		}
		else if(0 == AGV_Pat_Ptr->Angle)
		{
			ctrlParasPtr->comflag = 64122;
		}
		else if(AGV_Pat_Ptr->Angle < 0)
		{
			ctrlParasPtr->comflag = 64123;
			rmSpeed = 2;
		}
		
	}
	else if(AGV_Pat_Ptr->Midpoint < 0)
	{
		ctrlParasPtr->comflag = 6413;

		if((AGV_Pat_Ptr->Angle >= -maxLimt) && (AGV_Pat_Ptr->Angle <= maxLimt))
		{
			ctrlParasPtr->comflag = 64131;
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 641311;
				if((AGV_Pat_Ptr->Angle >= 2) && (AGV_Pat_Ptr->Midpoint >= -2))
				{
					ctrlParasPtr->comflag = 6413111;
					lmSpeed = AGV_Pat_Ptr->Angle;
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				ctrlParasPtr->comflag = 641312;
				//rmSpeed = 1;
				rmSpeed = 2;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 641313;
				rmSpeed = -AGV_Pat_Ptr->Angle;
			}
		}
		else
		{
			ctrlParasPtr->comflag = 64132;
			/*
			if(AGV_Pat_Ptr->Angle < -maxLimt)
			{
				ctrlParasPtr->comflag = 641321;
				rmSpeed = maxLimt / 2;
			}
			else if(AGV_Pat_Ptr->Angle > maxLimt)
			{
				if(AGV_Pat_Ptr->Midpoint >= -2)
				{
					ctrlParasPtr->comflag = 641322;
					lmSpeed = maxLimt / 2;
					
				}
			}
			*/
			ctrlParasPtr->BSflag = 0;
			
		}
		
		
	}
	
	T_monitor(&Tnow);

#if 1

	if(1 == Tnow.T1_update)
	{
		//u32 baseTime = 1200;
		
		Tnow.T1_update = 2;
		
		printf("Tnow.T1 = %d\r\n", Tnow.T1);
		
		if(Tnow.T1 < 200)			// 为原本的80%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 3;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 3;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(Tnow.T1 < 600)	// 为原本的60%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 2;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 2;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(Tnow.T1 < 1200)	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 1;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 1;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		
		
		
		
	}

#else
	if(1 == Tnow.T1_update)
	{
		Tnow.T1_update = 2;
		printf("Tpre.T1 = %d, Tnow.T1 = %d\r\n", Tpre.T1, Tnow.T1);
		if(Tpre.T1 > Tnow.T1)
		{
			if((Tpre.T1 - Tnow.T1) <= 0.2 * Tpre.T1)			// 为原本的80%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 1;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 1;
				}
				
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.4 * Tpre.T1)	// 为原本的60%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 2;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 2;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.6 * Tpre.T1)	// 为原本的40%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 3;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 3;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
	}

#endif	

	if(2 == Tnow.T1_update)
	{
		if(locRec3 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1 AngleDirection = %d\r\n", AGV_Pat_Ptr->AngleDirection);
			if(AGV_Pat_Ptr->AngleDirection < 0)
			//if((AGV_Pat_Ptr->Angle >= -4) && (AGV_Pat_Ptr->Angle <= 4))
			{
				printf("T1*******\r\n");
				Tnow.T1_update = 0;
				T1LSpeed = 0;
				T1RSpeed = 0;
			}
		}
	}

#if 0

	if(1 == Tnow.T2_update)
	{
		if(Tnow.T2 > 15000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 6;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 6;
			}
			
			Tnow.T2_update = 2;
			
		}
		else if(Tnow.T2 > 10000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 4;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 4;
			}
			
			Tnow.T2_update = 2;
			
		}
		else if(Tnow.T2 > 5000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 2;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 2;
			}
			
			Tnow.T2_update = 2;
			
		}

		if((T2LSpeedRec != T2LSpeed) || (T2RSpeedRec != T2RSpeed))
		{
			T2LSpeedRec = T2LSpeed;
			T2RSpeedRec = T2RSpeed;
			printf("T2LSpeed = %d, T2RSpeed = %d\r\n", T2LSpeed, T2RSpeed);
		}
		
	}
	
	
	if(2 == Tnow.T2_update)
	{
		Tnow.T2_update = 1;
		
		if(locRec4 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec4 = FMSDS_Ptr->AgvMSLocation;
			printf("T2 = %d\r\n", Tnow.T2);
			printf("T2 AngleDirection = %d\r\n", AGV_Pat_Ptr->AngleDirection);
			
			if(AGV_Pat_Ptr->AngleDirection < 0)
			{
				printf("T2*******\r\n");
				Tnow.T2_update = 3;
				T2LSpeed = 0;
				T2RSpeed = 0;
				locRec4 = AgvInits;
			}
		}
		
	}
	
#endif

	if(1 == Tnow.All_update)
	{
		Tpre = Tnow;

		Tnow.All_update = 0;
		Tnow.T1 = 0;
		Tnow.T1_update = 0;
		Tnow.T2 = 0;
		Tnow.T2_update = 0;
		Tnow.T3 = 0;
		Tnow.T3_update = 0;
	}
	
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - lmSpeed - T1LSpeed - T2LSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - rmSpeed - T1RSpeed - T2RSpeed;
	
	damping_func(1000, gearRecod, rmSpeedSet, lmSpeedSet);
	
	
}

void scale_1_mode10(u8 gear)
{
	static u8  lmSpeed_pat = 0, rmSpeed_pat = 0, lmSpeedbak = 0, rmSpeedbak = 0, lmflag = 0, rmflag = 0, flag = 0, flag2 = 0, cir = 0, flag3 = 0;
	static u32 startCount = 0, countTime = 0;
	static Agv_MS_Location locRec3 = AgvInits, locRec4 = AgvInits;
	u32 centCount = 0;
	u8 AgvGearS1CDLF[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0, gain = 3;
	u8 leftPullDuty[15] = {1, 2, 2, };
	static u32 time = 3000;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0;
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;

		
	#if 0
	
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AgvCent2Left == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6412;
			rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		}
		
	#else

	
	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

		if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_0_5)
		{
			if(0 == flag2)
			{
				if(countTime >= time)
				{
					locRec3 = FMSDS_Ptr->AgvMSLocation;
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
	else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
	{
		if((FMSDS_Ptr->AgvMSLocation >= locRec3) && (1 == flag2))
		{
			flag = 1;
			flag2 = 2;
			startCount = SystemRunningTime;
			printf("startCount***\r\n");
		}
		
	}

	if(1 == flag)
	{
		if(SystemRunningTime - startCount <= time)
		{
			if(FMSDS_Ptr->MaxRecoder <= Agv_MS_Left_2)
			{
				if(FMSDS_Ptr->MaxRecoder < Agv_MS_Left_7)
				{
					lmSpeedPull = 18;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_7)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 18;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_6)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 17;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_5)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 16;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_4_5)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 15;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_4)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 14;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_3_5)
				{
					
					lmSpeedPull = 13;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_3)
				{
					
					lmSpeedPull = 12;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_2_5)
				{
					
					lmSpeedPull = 10;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_2)
				{
					
					lmSpeedPull = 7;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_1_5)
				{
					lmSpeedPull = 3;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_1)
				{
					lmSpeedPull = 2;
				}

				if(0 == flag3)
				{
					flag3 =1;
					printf("MaxRecoder = %d\r\n", FMSDS_Ptr->MaxRecoder);
					printf("l-%d\r\n", lmSpeedPull);
				}
			}
			
		}
		else
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				printf("L add 1000\r\n");
				flag3 = 0;
				time = 200;
				startCount = SystemRunningTime;
			}
			else
			{
				flag = 2;
				time = 3000;
			}
			
		}
		
	}

	if(2 == flag)
	{
		printf("close***\r\n");
		flag = 3;
	}
	
	rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
	
	
	#endif
	
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

	#if 0
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
		{

		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6422;
			lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		}

	#else

		
		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

			if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_0_5)
			{
				if(0 == flag2)
				{
					if(countTime >= time)
					{
						locRec3 = FMSDS_Ptr->AgvMSLocation;
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
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
		{
			if((FMSDS_Ptr->AgvMSLocation <= locRec3) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}
		
		
		if(1 == flag)
		{
			if(SystemRunningTime - startCount <= time)
			{
				if(FMSDS_Ptr->MaxRecoder >= Agv_MS_Right_2)
				{
					if(FMSDS_Ptr->MaxRecoder > Agv_MS_Right_7)
					{
						rmSpeedPull = 18;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_7)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 18;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_6)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 17;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_5)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 16;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_4_5)
					{
						rmSpeedPull = 15;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_4)
					{
						rmSpeedPull = 14;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_3_5)
					{
						rmSpeedPull = 13;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_3)
					{
						rmSpeedPull = 12;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_2_5)
					{
						rmSpeedPull = 10;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_2)
					{
						rmSpeedPull = 8;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_1_5)
					{
						rmSpeedPull = 3;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_1)
					{
						rmSpeedPull = 2;
					}

					if(0 == flag3)
					{
						flag3 =1;
						printf("MaxRecoder = %d\r\n", FMSDS_Ptr->MaxRecoder);
						printf("r-%d\r\n", rmSpeedPull);
					}
				}
				
				
			}
			else
			{
				if(AGV_Pat_Ptr->Angle < 0)
				{
					printf("R add 1000\r\n");
					flag3 = 0;
					time = 200;
					startCount = SystemRunningTime;
				}
				else
				{
					flag = 2;
				}
				
			}
			
		}

		if(2 == flag)
		{
			printf("close***\r\n");
			flag = 3;
		}
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
	

	#endif
		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;
	
		lmSpeed = 0;
		rmSpeed = 0;
		cir = 0;
		countTime = 0;
		startCount = 0;
		time = 3000;

		flag3 = 0;

		flag = 0;
		flag2 = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}
	
		
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - lmSpeedPull;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - rmSpeedPull;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}


void scale_1_mode11(u8 gear)
{
	static u8  lmSpeed_pat = 0, rmSpeed_pat = 0, lmSpeedbak = 0, rmSpeedbak = 0, lmflag = 0, rmflag = 0, flag = 0, flag2 = 0, cir = 0;
	static u32 startCount = 0, countTime = 0;
	static Agv_MS_Location locRec3 = AgvInits, locRec4 = AgvInits;
	u32 centCount = 0;
	u8 AgvGearS1CDLF[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20};
	u8 gearRecod = 0, gain = 3;
	
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0;
	// 普通模式,偏差在1格之内调整

	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;

		
	#if 0
	
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AgvCent2Left == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6412;
			rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		}
		
	#else

	
	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

		if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_0_5)
		{
			if(0 == flag2)
			{
				if(countTime >= 3000)
				{
					locRec3 = FMSDS_Ptr->AgvMSLocation;
					printf("5000Point: ");
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
	else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
	{
		if((FMSDS_Ptr->AgvMSLocation >= locRec3) && (1 == flag2))
		{
			flag = 1;
			flag2 = 2;
			startCount = SystemRunningTime;
			printf("startCount***\r\n");
		}
		
	}

	if(1 == flag)
	{
		if(SystemRunningTime - startCount <= 3000)
		{
			if(FMSDS_Ptr->MaxRecoder <= Agv_MS_Left_3)
			{
				//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
				lmSpeedPull =10;
			}
			
		}
		else
		{
			flag = 2;
		}
		
	}

	if(2 == flag)
	{
		printf("close***\r\n");
		flag = 3;
	}
	
	rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
	
	
	#endif
	
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

	#if 0
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
		{

		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6422;
			lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		}

	#else

		
		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

			if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_0_5)
			{
				if(0 == flag2)
				{
					if(countTime >= 3000)
					{
						locRec3 = FMSDS_Ptr->AgvMSLocation;
						printf("5000Point: ");
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
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
		{
			if((FMSDS_Ptr->AgvMSLocation <= locRec3) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}
		
		
		if(1 == flag)
		{
			if(SystemRunningTime - startCount <= 3000)
			{
				if(FMSDS_Ptr->MaxRecoder >= Agv_MS_Right_3)
				{
					rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
					//rmSpeedPull = 10;
				}
				
				
			}
			else
			{
				flag = 2;
			}
			
		}

		if(2 == flag)
		{
			printf("close***\r\n");
			flag = 3;
		}
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
	

	#endif
		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;
	
		lmSpeed = 0;
		rmSpeed = 0;
		cir = 0;
		countTime = 0;
		startCount = 0;

		flag = 0;
		flag2 = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}
	
		
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - lmSpeedPull;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - rmSpeedPull;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}


void scale_1_mode12(u8 gear)
{
	static u8  lmSpeed_pat = 0, rmSpeed_pat = 0, lmSpeedbak = 0, rmSpeedbak = 0, lmflag = 0, rmflag = 0, flag = 0, flag2 = 0, cir = 0, flag3 = 0, pullFlag = 0;
	static u32 startCount = 0, countTime = 0;
	static Agv_MS_Location locRec1 = AgvInits, locRec2 = AgvInits, locRec3 = AgvInits, locRec4 = AgvInits;
	u32 centCount = 0;
	u8 AgvGearS1CDLF[20] = {1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0, gain = 3;
	u8 leftPullDuty[15] = {1, 2, 2, };
	static u32 time = 3000;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0;
	u32 T1 = 0;
	static u8 T1LSpeed = 0, T1RSpeed = 0;
	// 普通模式,偏差在1格之内调整
	static Trec Tnow, Tpre;
	
	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;

		
	#if 0
	
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AgvCent2Left == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6412;
			rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		}
		
	#else

	
	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

		if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_0_5)
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
	else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
	{
		if((FMSDS_Ptr->AgvMSLocation >= locRec1) && (1 == flag2))
		{
			flag = 1;
			flag2 = 2;
			startCount = SystemRunningTime;
			printf("startCount***\r\n");
		}
		
	}

	if(1 == flag)
	{
		if(SystemRunningTime - startCount <= time)
		{
			if(FMSDS_Ptr->MaxRecoder <= Agv_MS_Left_2)
			{
				if(FMSDS_Ptr->MaxRecoder < Agv_MS_Left_7)
				{
					lmSpeedPull = 18;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_7)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 18;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_6)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 17;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_5)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 16;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_4_5)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 15;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_4)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 14;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_3_5)
				{
					
					lmSpeedPull = 13;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_3)
				{
					
					lmSpeedPull = 12;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_2_5)
				{
					
					lmSpeedPull = 10;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_2)
				{
					
					lmSpeedPull = 7;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_1_5)
				{
					lmSpeedPull = 3;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_1)
				{
					lmSpeedPull = 2;
				}

				if(0 == flag3)
				{
					flag3 =1;
					printf("MaxRecoder = %d\r\n", FMSDS_Ptr->MaxRecoder);
					printf("l-%d\r\n", lmSpeedPull);
				}
			}
			
		}
		else
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				printf("L add 1000\r\n");
				flag3 = 0;
				time = 200;
				startCount = SystemRunningTime;
			}
			else
			{
				flag = 2;
				time = 3000;
			}
			
		}
		
	}

	if(2 == flag)
	{
		printf("close***\r\n");
		flag = 3;
	}
	
	rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
	
	
	#endif
	
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

	#if 0
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
		{

		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6422;
			lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		}

	#else

		
		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

			if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_0_5)
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
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
		{
			if((FMSDS_Ptr->AgvMSLocation <= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}
		
		
		if(1 == flag)
		{
			if(SystemRunningTime - startCount <= time)
			{
				if(FMSDS_Ptr->MaxRecoder >= Agv_MS_Right_2)
				{
					if(FMSDS_Ptr->MaxRecoder > Agv_MS_Right_7)
					{
						rmSpeedPull = 18;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_7)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 18;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_6)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 17;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_5)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 16;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_4_5)
					{
						rmSpeedPull = 15;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_4)
					{
						rmSpeedPull = 14;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_3_5)
					{
						rmSpeedPull = 13;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_3)
					{
						rmSpeedPull = 12;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_2_5)
					{
						rmSpeedPull = 10;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_2)
					{
						rmSpeedPull = 8;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_1_5)
					{
						rmSpeedPull = 3;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_1)
					{
						rmSpeedPull = 2;
					}

					if(0 == flag3)
					{
						flag3 =1;
						printf("MaxRecoder = %d\r\n", FMSDS_Ptr->MaxRecoder);
						printf("r-%d\r\n", rmSpeedPull);
					}
				}
				
				
			}
			else
			{
				if(AGV_Pat_Ptr->Angle < 0)
				{
					printf("R add 1000\r\n");
					flag3 = 0;
					time = 200;
					startCount = SystemRunningTime;
				}
				else
				{
					flag = 2;
				}
				
			}
			
		}

		if(2 == flag)
		{
			printf("close***\r\n");
			flag = 3;
		}
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
	

	#endif
		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;
	
		lmSpeed = 0;
		rmSpeed = 0;
		cir = 0;
		countTime = 0;
		startCount = 0;
		time = 3000;

		flag3 = 0;

		flag = 0;
		flag2 = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	
	T_monitor(&Tnow);

#if 1

	if(1 == Tnow.T1_update)
	{
		//u32 baseTime = 1200;
		
		Tnow.T1_update = 2;
		
		printf("Tnow.T1 = %d\r\n", Tnow.T1);
		
		if(Tnow.T1 < 100)	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 6;
				T1LSpeed = 0;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 6;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(  100 <= Tnow.T1 && Tnow.T1 < 200 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 5;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 5;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		
		else if(  200 <= Tnow.T1 && Tnow.T1 < 400 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 4;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 4;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(  400 <= Tnow.T1 && Tnow.T1 < 800 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 3;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 3;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(  800 <= Tnow.T1 && Tnow.T1 < 1200 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 2;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 2;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(  1200 <= Tnow.T1 && Tnow.T1 < 1500 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 1;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 1;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		
	}
	
#else

	if(1 == Tnow.T1_update)
	{
		Tnow.T1_update = 2;
		printf("Tpre.T1 = %d, Tnow.T1 = %d\r\n", Tpre.T1, Tnow.T1);
		if(Tpre.T1 > Tnow.T1)
		{
			if((Tpre.T1 - Tnow.T1) <= 0.2 * Tpre.T1)			// 为原本的80%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 1;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 1;
				}
				
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.4 * Tpre.T1)	// 为原本的60%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 2;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 2;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.6 * Tpre.T1)	// 为原本的40%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 4;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 4;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.8 * Tpre.T1)	// 为原本的20%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 6;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 6;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
	}

#endif	

	if(2 == Tnow.T1_update)
	{
		if(locRec3 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1 AngleDirection = %d\r\n", AGV_Pat_Ptr->AngleDirection);
			//if((AGV_Pat_Ptr->AngleDirection < 0) || (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_2))
			if(AGV_Pat_Ptr->AngleDirection < 0)
			{
				printf("T1*******\r\n");
				Tnow.T1_update = 0;
				T1LSpeed = 0;
				T1RSpeed = 0;
			}
		}
	}
	
	
	if(1 == Tnow.All_update)
	{
		Tpre = Tnow;

		Tnow.All_update = 0;
		Tnow.T1 = 0;
		Tnow.T1_update = 0;
		Tnow.T2 = 0;
		Tnow.T2_update = 0;
		Tnow.T3 = 0;
		Tnow.T3_update = 0;
	}
	
	if((0 != lmSpeedPull) || (0 != rmSpeedPull))
	{
		T1LSpeed = 0;
		T1RSpeed = 0;
	}
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - lmSpeedPull - T1LSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - rmSpeedPull - T1RSpeed;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}

void scale_1_mode12_back(u8 gear)
{
	static u8  lmSpeed_pat = 0, rmSpeed_pat = 0, lmSpeedbak = 0, rmSpeedbak = 0, lmflag = 0, rmflag = 0, flag = 0, flag2 = 0, cir = 0, flag3 = 0, pullFlag = 0;
	static u32 startCount = 0, countTime = 0;
	static Agv_MS_Location locRec1 = AgvInits, locRec2 = AgvInits, locRec3 = AgvInits, locRec4 = AgvInits;
	u32 centCount = 0;
	u8 AgvGearS1CDLF[20] = {1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0, gain = 3;
	u8 leftPullDuty[15] = {1, 2, 2, };
	static u32 time = 3000;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0;
	u32 T1 = 0;
	static u8 T1LSpeed = 0, T1RSpeed = 0;
	// 普通模式,偏差在1格之内调整
	static Trec Tnow, Tpre;
	
	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;

		
	#if 0
	
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AgvCent2Left == FMSDS_Ptr->agvDirection)
			{
				FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			}
			
			
			
			
		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6412;
			rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		}
		
	#else

	
	if(AgvCent2Left == FMSDS_Ptr->agvDirection)
	{
		FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

		if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_0_5)
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
	else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)
	{
		if((FMSDS_Ptr->AgvMSLocation >= locRec1) && (1 == flag2))
		{
			flag = 1;
			flag2 = 2;
			startCount = SystemRunningTime;
			printf("startCount***\r\n");
		}
		
	}

	if(1 == flag)
	{
		if(SystemRunningTime - startCount <= time)
		{
			if(FMSDS_Ptr->MaxRecoder <= Agv_MS_Left_2)
			{
				if(FMSDS_Ptr->MaxRecoder < Agv_MS_Left_7)
				{
					lmSpeedPull = 18;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_7)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 18;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_6)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 17;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_5)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 16;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_4_5)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 15;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_4)
				{
					
					//lmSpeedPull = AgvGearS1CDLF[Agv_MS_Right_0_5 - FMSDS_Ptr->MaxRecoder];
					lmSpeedPull = 14;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_3_5)
				{
					
					lmSpeedPull = 13;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_3)
				{
					
					lmSpeedPull = 12;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_2_5)
				{
					
					lmSpeedPull = 10;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_2)
				{
					
					lmSpeedPull = 7;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_1_5)
				{
					lmSpeedPull = 3;
				}
				else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Left_1)
				{
					lmSpeedPull = 2;
				}

				if(0 == flag3)
				{
					flag3 =1;
					printf("MaxRecoder = %d\r\n", FMSDS_Ptr->MaxRecoder);
					printf("l-%d\r\n", lmSpeedPull);
				}
			}
			
		}
		else
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				printf("L add 1000\r\n");
				flag3 = 0;
				time = 200;
				startCount = SystemRunningTime;
			}
			else
			{
				flag = 2;
				time = 3000;
			}
			
		}
		
	}

	if(2 == flag)
	{
		printf("close***\r\n");
		flag = 3;
	}
	
	rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
	
	
	#endif
	
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

	#if 0
		
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_5))
		{

		}
		else
		{
			// 这里一般要采取紧急措施了
			ctrlParasPtr->comflag = 6422;
			lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		}

	#else

		
		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

			if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_0_5)
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
		else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)
		{
			if((FMSDS_Ptr->AgvMSLocation <= locRec1) && (1 == flag2))
			{
				flag = 1;
				flag2 = 2;
				startCount = SystemRunningTime;
				printf("startCount***\r\n");
			}
			
		}
		
		
		if(1 == flag)
		{
			if(SystemRunningTime - startCount <= time)
			{
				if(FMSDS_Ptr->MaxRecoder >= Agv_MS_Right_2)
				{
					if(FMSDS_Ptr->MaxRecoder > Agv_MS_Right_7)
					{
						rmSpeedPull = 18;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_7)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 18;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_6)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 17;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_5)
					{
						//rmSpeedPull = AgvGearS1CDLF[FMSDS_Ptr->MaxRecoder - Agv_MS_Right_0_5];
						rmSpeedPull = 16;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_4_5)
					{
						rmSpeedPull = 15;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_4)
					{
						rmSpeedPull = 14;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_3_5)
					{
						rmSpeedPull = 13;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_3)
					{
						rmSpeedPull = 12;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_2_5)
					{
						rmSpeedPull = 10;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_2)
					{
						rmSpeedPull = 8;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_1_5)
					{
						rmSpeedPull = 3;
					}
					else if(FMSDS_Ptr->MaxRecoder == Agv_MS_Right_1)
					{
						rmSpeedPull = 2;
					}

					if(0 == flag3)
					{
						flag3 =1;
						printf("MaxRecoder = %d\r\n", FMSDS_Ptr->MaxRecoder);
						printf("r-%d\r\n", rmSpeedPull);
					}
				}
				
				
			}
			else
			{
				if(AGV_Pat_Ptr->Angle < 0)
				{
					printf("R add 1000\r\n");
					flag3 = 0;
					time = 200;
					startCount = SystemRunningTime;
				}
				else
				{
					flag = 2;
				}
				
			}
			
		}

		if(2 == flag)
		{
			printf("close***\r\n");
			flag = 3;
		}
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
	

	#endif
		
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;
	
		lmSpeed = 0;
		rmSpeed = 0;
		cir = 0;
		countTime = 0;
		startCount = 0;
		time = 3000;

		flag3 = 0;

		flag = 0;
		flag2 = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	
	T_monitor(&Tnow);

#if 1

	if(1 == Tnow.T1_update)
	{
		//u32 baseTime = 1200;
		
		Tnow.T1_update = 2;
		
		printf("Tnow.T1 = %d\r\n", Tnow.T1);
		
		if(Tnow.T1 < 100)	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 6;
				T1LSpeed = 0;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 6;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(  100 <= Tnow.T1 && Tnow.T1 < 200 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 5;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 5;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		
		else if(  200 <= Tnow.T1 && Tnow.T1 < 400 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 4;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 4;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(  400 <= Tnow.T1 && Tnow.T1 < 800 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 3;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 3;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(  800 <= Tnow.T1 && Tnow.T1 < 1200 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 2;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 2;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(  1200 <= Tnow.T1 && Tnow.T1 < 1500 )	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1LSpeed = 0;
				T1RSpeed = 1;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 1;
				T1RSpeed = 0;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		
	}
	
#else

	if(1 == Tnow.T1_update)
	{
		Tnow.T1_update = 2;
		printf("Tpre.T1 = %d, Tnow.T1 = %d\r\n", Tpre.T1, Tnow.T1);
		if(Tpre.T1 > Tnow.T1)
		{
			if((Tpre.T1 - Tnow.T1) <= 0.2 * Tpre.T1)			// 为原本的80%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 1;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 1;
				}
				
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.4 * Tpre.T1)	// 为原本的60%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 2;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 2;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.6 * Tpre.T1)	// 为原本的40%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 4;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 4;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.8 * Tpre.T1)	// 为原本的20%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 6;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 6;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
	}

#endif	

	if(2 == Tnow.T1_update)
	{
		if(locRec3 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1 AngleDirection = %d\r\n", AGV_Pat_Ptr->AngleDirection);
			//if((AGV_Pat_Ptr->AngleDirection < 0) || (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_2))
			if(AGV_Pat_Ptr->AngleDirection < 0)
			{
				printf("T1*******\r\n");
				Tnow.T1_update = 0;
				T1LSpeed = 0;
				T1RSpeed = 0;
			}
		}
	}
	
	
	if(1 == Tnow.All_update)
	{
		Tpre = Tnow;

		Tnow.All_update = 0;
		Tnow.T1 = 0;
		Tnow.T1_update = 0;
		Tnow.T2 = 0;
		Tnow.T2_update = 0;
		Tnow.T3 = 0;
		Tnow.T3_update = 0;
	}
	
	if((0 != lmSpeedPull) || (0 != rmSpeedPull))
	{
		T1LSpeed = 0;
		T1RSpeed = 0;
	}
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - lmSpeed - lmSpeedPull - T1LSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - rmSpeed - rmSpeedPull - T1RSpeed;
	
	damping_func(1000, gearRecod, rmSpeedSet, lmSpeedSet);
	
	
}



void scale_1_mode14(u8 gear)
{
	u8 AgvGearPatAngl[50] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
	u8 gearRecod = 0, gain = 3;
	static Agv_MS_Location locRec3 = AgvInits, locRec4 = AgvInits;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0;
	u8 maxLimt = 16;
	u32 T1 = 0, T2 = 0, T3 = 0;
	u8 T1Speed = 0, T2Speed = 0, T3Speed = 0;
	static u8 T1LSpeed = 0, T2LSpeed = 0, T3LSpeed = 0, T1RSpeed = 0, T2RSpeed = 0, T3RSpeed = 0, T2LSpeedRec = 0, T2RSpeedRec = 0;
	// 普通模式,偏差在1格之内调整
	static Trec Tnow, Tpre;
	
	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if(AGV_Pat_Ptr->Midpoint > 0)
	{
		ctrlParasPtr->comflag = 641;

		if((AGV_Pat_Ptr->Angle >= -maxLimt) && (AGV_Pat_Ptr->Angle <= maxLimt))
		{
			ctrlParasPtr->comflag = 6411;
			
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 64111;
				lmSpeed = AGV_Pat_Ptr->Angle;
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				ctrlParasPtr->comflag = 64112;
				//lmSpeed = 1;
				lmSpeed = 2;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 64113;
				
				if((AGV_Pat_Ptr->Angle <= -2) && (AGV_Pat_Ptr->Midpoint <= 2))
				{
					rmSpeed = -AGV_Pat_Ptr->Angle;
				}
				
			}
		}
		else
		{
			ctrlParasPtr->comflag = 6411;
			/*
			if(AGV_Pat_Ptr->Angle < -maxLimt)
			{
				if(AGV_Pat_Ptr->Midpoint <= 2)
				{
					ctrlParasPtr->comflag = 64111;
					rmSpeed = maxLimt / 2;
				}
			}
			else if(AGV_Pat_Ptr->Angle > maxLimt)
			{
				ctrlParasPtr->comflag = 64112;
				lmSpeed = maxLimt / 2;
			}
			*/
			
			ctrlParasPtr->FSflag = 0;
			printf("FMSD_Hex = %x, RMSD_Hex = %x\r\n", FMSDS_Ptr->MSD_Hex, RMSDS_Ptr->MSD_Hex);
			
		}
		
	}
	else if(0 == AGV_Pat_Ptr->Midpoint)
	{
		ctrlParasPtr->comflag = 6412;
		if(AGV_Pat_Ptr->Angle > 0)		
		{
			ctrlParasPtr->comflag = 64121;
			//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod];		
			lmSpeed = 2;
		}
		else if(0 == AGV_Pat_Ptr->Angle)
		{
			ctrlParasPtr->comflag = 64122;
		}
		else if(AGV_Pat_Ptr->Angle < 0)
		{
			ctrlParasPtr->comflag = 64123;
			rmSpeed = 2;
		}
		
	}
	else if(AGV_Pat_Ptr->Midpoint < 0)
	{
		ctrlParasPtr->comflag = 6413;

		if((AGV_Pat_Ptr->Angle >= -maxLimt) && (AGV_Pat_Ptr->Angle <= maxLimt))
		{
			ctrlParasPtr->comflag = 64131;
			if(AGV_Pat_Ptr->Angle > 0)
			{
				ctrlParasPtr->comflag = 641311;
				if((AGV_Pat_Ptr->Angle >= 2) && (AGV_Pat_Ptr->Midpoint >= -2))
				{
					ctrlParasPtr->comflag = 6413111;
					lmSpeed = AGV_Pat_Ptr->Angle;
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				ctrlParasPtr->comflag = 641312;
				//rmSpeed = 1;
				rmSpeed = 2;
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				ctrlParasPtr->comflag = 641313;
				rmSpeed = -AGV_Pat_Ptr->Angle;
			}
		}
		else
		{
			ctrlParasPtr->comflag = 64132;
			/*
			if(AGV_Pat_Ptr->Angle < -maxLimt)
			{
				ctrlParasPtr->comflag = 641321;
				rmSpeed = maxLimt / 2;
			}
			else if(AGV_Pat_Ptr->Angle > maxLimt)
			{
				if(AGV_Pat_Ptr->Midpoint >= -2)
				{
					ctrlParasPtr->comflag = 641322;
					lmSpeed = maxLimt / 2;
					
				}
			}
			*/
			ctrlParasPtr->FSflag = 0;
			
		}
		
	}
	
	T_monitor(&Tnow);

#if 1

	if(1 == Tnow.T1_update)
	{
		//u32 baseTime = 1200;
		
		Tnow.T1_update = 2;
		
		printf("Tnow.T1 = %d\r\n", Tnow.T1);
		
		if(Tnow.T1 < 200)			// 为原本的80%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 3;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 3;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(Tnow.T1 < 600)	// 为原本的60%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 2;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 2;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		else if(Tnow.T1 < 1200)	// 为原本的40%以上
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T1RSpeed = 1;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T1LSpeed = 1;
			}
			
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
		
		
		
		
	}

#else
	if(1 == Tnow.T1_update)
	{
		Tnow.T1_update = 2;
		printf("Tpre.T1 = %d, Tnow.T1 = %d\r\n", Tpre.T1, Tnow.T1);
		if(Tpre.T1 > Tnow.T1)
		{
			if((Tpre.T1 - Tnow.T1) <= 0.2 * Tpre.T1)			// 为原本的80%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 1;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 1;
				}
				
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.4 * Tpre.T1)	// 为原本的60%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 2;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 2;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			else if((Tpre.T1 - Tnow.T1) <= 0.6 * Tpre.T1)	// 为原本的40%以上
			{
				if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
				{
					T1RSpeed = 3;
				}
				else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
				{
					T1LSpeed = 3;
				}
				locRec3 = FMSDS_Ptr->AgvMSLocation;
			}
			
			printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
		}
	}

#endif	

	if(2 == Tnow.T1_update)
	{
		if(locRec3 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec3 = FMSDS_Ptr->AgvMSLocation;
			printf("T1 AngleDirection = %d\r\n", AGV_Pat_Ptr->AngleDirection);
			if(AGV_Pat_Ptr->AngleDirection < 0)
			//if((AGV_Pat_Ptr->Angle >= -4) && (AGV_Pat_Ptr->Angle <= 4))
			{
				printf("T1*******\r\n");
				Tnow.T1_update = 0;
				T1LSpeed = 0;
				T1RSpeed = 0;
			}
		}
	}

#if 0

	if(1 == Tnow.T2_update)
	{
		if(Tnow.T2 > 15000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 6;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 6;
			}
			
			Tnow.T2_update = 2;
			
		}
		else if(Tnow.T2 > 10000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 4;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 4;
			}
			
			Tnow.T2_update = 2;
			
		}
		else if(Tnow.T2 > 5000)
		{
			if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_0_5)
			{
				T2LSpeed = 2;
			}
			else if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_0_5)
			{
				T2RSpeed = 2;
			}
			
			Tnow.T2_update = 2;
			
		}

		if((T2LSpeedRec != T2LSpeed) || (T2RSpeedRec != T2RSpeed))
		{
			T2LSpeedRec = T2LSpeed;
			T2RSpeedRec = T2RSpeed;
			printf("T2LSpeed = %d, T2RSpeed = %d\r\n", T2LSpeed, T2RSpeed);
		}
		
	}
	
	
	if(2 == Tnow.T2_update)
	{
		Tnow.T2_update = 1;
		
		if(locRec4 != FMSDS_Ptr->AgvMSLocation)
		{
			locRec4 = FMSDS_Ptr->AgvMSLocation;
			printf("T2 = %d\r\n", Tnow.T2);
			printf("T2 AngleDirection = %d\r\n", AGV_Pat_Ptr->AngleDirection);
			
			if(AGV_Pat_Ptr->AngleDirection < 0)
			{
				printf("T2*******\r\n");
				Tnow.T2_update = 3;
				T2LSpeed = 0;
				T2RSpeed = 0;
				locRec4 = AgvInits;
			}
		}
		
	}
	
#endif

	if(1 == Tnow.All_update)
	{
		Tpre = Tnow;

		Tnow.All_update = 0;
		Tnow.T1 = 0;
		Tnow.T1_update = 0;
		Tnow.T2 = 0;
		Tnow.T2_update = 0;
		Tnow.T3 = 0;
		Tnow.T3_update = 0;
	}
	
			
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - T1LSpeed - T2LSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - T1RSpeed - T2RSpeed;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}

void scale_1_mode15(u8 gear)
{
	static u8  lmSpeed_pat = 0, rmSpeed_pat = 0, lmSpeedbak = 0, rmSpeedbak = 0, lmflag = 0, rmflag = 0, flag = 0, flag2 = 0, cir = 0, flag3 = 0, pullFlag = 0;
	static u32 startCount = 0, countTime = 0;
	static Agv_MS_Location locRec1 = AgvInits, locRec2 = AgvInits, locRec3 = AgvInits, locRec4 = AgvInits;
	u32 centCount = 0;
	u8 AgvGearS1CDLF[20] = {1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0, gain = 3;
	u8 leftPullDuty[15] = {1, 2, 2, };
	static u32 time = 3000;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0;
	u32 T1 = 0;
	static u8 T1LSpeed = 0, T1RSpeed = 0;
	// 普通模式,偏差在1格之内调整
	
	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;

		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			
		}
		
		rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

		}
		
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;
	
		lmSpeed = 0;
		rmSpeed = 0;
		cir = 0;
		countTime = 0;
		startCount = 0;
		time = 3000;

		flag3 = 0;

		flag = 0;
		flag2 = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	T1_Adapter(&T1LSpeed, &T1RSpeed);
	
	//lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - lmSpeedPull - T1LSpeed;
	
	//rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - rmSpeedPull - T1RSpeed;

	//printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
	
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - T1LSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - T1RSpeed;
	
	damping_func(1000, gearRecod, lmSpeedSet, rmSpeedSet);
	
	
}

void scale_1_mode15_back(u8 gear)
{
	static u8  lmSpeed_pat = 0, rmSpeed_pat = 0, lmSpeedbak = 0, rmSpeedbak = 0, lmflag = 0, rmflag = 0, flag = 0, flag2 = 0, cir = 0, flag3 = 0, pullFlag = 0;
	static u32 startCount = 0, countTime = 0;
	static Agv_MS_Location locRec1 = AgvInits, locRec2 = AgvInits, locRec3 = AgvInits, locRec4 = AgvInits;
	u32 centCount = 0;
	u8 AgvGearS1CDLF[20] = {1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 10, 10};
	u8 gearRecod = 0, gain = 3;
	u8 leftPullDuty[15] = {1, 2, 2, };
	static u32 time = 3000;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPull = 0, rmSpeedPull = 0;
	u32 T1 = 0;
	static u8 T1LSpeed = 0, T1RSpeed = 0;
	// 普通模式,偏差在1格之内调整
	
	gearRecod = gear;	
	
	ctrlParasPtr->comflag = 64;
	
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		
		ctrlParasPtr->comflag = 641;

		if(AgvCent2Left == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;
			
		}
		
		rmSpeed = AgvGearS1CDLF[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		ctrlParasPtr->comflag = 642;

		if(AgvCent2Right == FMSDS_Ptr->agvDirection)
		{
			FMSDS_Ptr->MaxRecoder = FMSDS_Ptr->AgvMSLocation;

		}
		
		
		lmSpeed = AgvGearS1CDLF[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeed_pat = 0;
		rmSpeed_pat = 0;
	
		lmSpeed = 0;
		rmSpeed = 0;
		cir = 0;
		countTime = 0;
		startCount = 0;
		time = 3000;

		flag3 = 0;

		flag = 0;
		flag2 = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
		
	}

	T1_Adapter_back(&T1LSpeed, &T1RSpeed);
	
	//lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeed - lmSpeedPull - T1LSpeed;
	
	//rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeed - rmSpeedPull - T1RSpeed;
	//printf("T1LSpeed = %d, T1RSpeed = %d\r\n", T1LSpeed, T1RSpeed);
	
	lmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod] - lmSpeed - T1LSpeed;
	
	rmSpeedSet = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod] - rmSpeed - T1RSpeed;
	
	damping_func(1000, gearRecod, rmSpeedSet, lmSpeedSet);
	
	
}


void AGV_Correct_gS_8ug(u8 gear)		// 3 mode
{
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0;
	u32 centCount = 0;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	
	gearRecod = gear;
	
	//printf("************\r\n");

	
	if((Agv_MS_CrossRoad != FMSDS_Pre_Ptr->AgvMSLocation) && (Agv_MS_Undefine != FMSDS_Pre_Ptr->AgvMSLocation) &&\
		(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3))
	{
		//printf("AgvMSLocation %d, %d\r\n",FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation);
		
		#if 0
		/********************************/
		if(0 == ctrlParasPtr->FSflag)			// 启动修正模式, 进入修正控制	
		{
			ctrlParasPtr->comflag = 61;
		}
		else if(2 == ctrlParasPtr->FSflag)		// 普通模式下
		{
			ctrlParasPtr->comflag = 62;
			
			if(((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End)) ||\
				((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End)))
			{
				// 超过5格, 跑失控修正模式
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->comflag = 621;
			}
			
			/*
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_2))
			{
				// 正常模式
				ctrlParasPtr->FSflag = 2;
				ctrlParasPtr->comflag = 622;
			}
			*/
		}
		#endif
		
		
		
		/***********************实现************************/
		
		

		if(0 == ctrlParasPtr->FSflag)		
		{
			// 启动模式
			gS_startup_mode(3);
			
		}
		else if(1 == ctrlParasPtr->FSflag)
		{
			// 偏差达到1格模式
			scale_1_mode15(gearRecod);
			
		}
		else if(2 == ctrlParasPtr->FSflag)
		{
			// 紧急模式
			gS_urgency_mode();
			
		}
		
		
		
		
			
	}
	
}

void AGV_Correct_back_ug(u8 gear)		// 3 mode
{
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0;
	u32 centCount = 0;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	
	gearRecod = gear;
	
	//printf("************\r\n");

	
	//if((Agv_MS_CrossRoad != FMSDS_Pre_Ptr->AgvMSLocation) && (Agv_MS_Undefine != FMSDS_Pre_Ptr->AgvMSLocation) &&\
		//(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3))
	if(1)
	{
		//printf("AgvMSLocation %d, %d\r\n",FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation);
		
		#if 0
		/********************************/
		if(0 == ctrlParasPtr->FSflag)			// 启动修正模式, 进入修正控制	
		{
			ctrlParasPtr->comflag = 61;
		}
		else if(2 == ctrlParasPtr->FSflag)		// 普通模式下
		{
			ctrlParasPtr->comflag = 62;
			
			if(((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End)) ||\
				((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_5) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End)))
			{
				// 超过5格, 跑失控修正模式
				ctrlParasPtr->FSflag = 0;
				ctrlParasPtr->comflag = 621;
			}
			
			/*
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_2) || (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_2))
			{
				// 正常模式
				ctrlParasPtr->FSflag = 2;
				ctrlParasPtr->comflag = 622;
			}
			*/
		}
		#endif
		
		
		
		/***********************实现************************/
		
		

		if(0 == ctrlParasPtr->BSflag)		
		{
			// 启动模式
			back_startup_mode(3);
			
		}
		else if(1 == ctrlParasPtr->BSflag)
		{
			// 偏差达到1格模式
			scale_1_mode15_back(gearRecod);
			
		}
		else if(2 == ctrlParasPtr->BSflag)
		{
			// 紧急模式
			gS_urgency_mode();
			
		}
		
		
			
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
	static u8 lmSpeed = 0, rmSpeed = 0, lreco = 0, rreco = 0;
	static u32 startCount = 0;
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
			lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLB[gearRecod];
			rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRB[gearRecod];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
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



void AGV_Correct_back_1(void)
{
	static u32 counter = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 2, lreco = 0, rreco = 0, flag = 0;
	//static u8 loffset = 0, roffset = 0;
	
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
					
					lmSpeed = AgvGear[gearRecod] + BLG[0][gearRecod];
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
		
		
		if(0 == flag)
		{
			counter = 0;

			if(lreco > rreco)
			{
				if(lreco - rreco >= 3)
				{
					if(BRG[0][gearRecod] > 0)
					{
						BRG[0][gearRecod]--;
					}
					else
					{
						BLG[0][gearRecod]++;
					}
					
					printf("loffset = %d, roffset = %d\r\n", BLG[0][gearRecod], BRG[0][gearRecod]);
					
					
					lreco = 0;
					rreco = 0;
				}
			}
			else if(lreco < rreco)
			{
				if(rreco - lreco >= 3)
				{
					if(BLG[0][gearRecod] > 0)
					{
						BLG[0][gearRecod]--;
					}
					else
					{
						BRG[0][gearRecod]++;
					}
					
					printf("loffset = %d, roffset = %d\r\n", BLG[0][gearRecod], BRG[0][gearRecod]);
					lreco = 0;
					rreco = 0;
				}
			}
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				ctrlParasPtr->comflag = 12;

				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))		// 做偏差在三格内
				{
					
				}

				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Center)
					{
						lreco++;
					}
					
				}
				
				lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + BLG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{		
				ctrlParasPtr->comflag = 13;

				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
					{
						rreco++;
					}
					
				}

				/*
				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
				{
					ctrlParasPtr->comflag = 131;
					
					//rreco++;
					//printf("rreco = %d\r\n", rreco);
					lmSpeed = AgvGear[gearRecod];
					rmSpeed = AgvGear[gearRecod];
				}
				else
				{
					//printf("gearRecod = %d\r\n", gearRecod);
					ctrlParasPtr->comflag = 132;
					lmSpeed = AgvGear[gearRecod];
					rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + roffset;
				}
				*/
				
				lmSpeed = AgvGear[gearRecod];
				rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + BRG[0][gearRecod];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
				
			}
		}
		else
		{
			
		}
		
		
		
	}
	
}




void AGV_Correct_back_2(void)
{
	static u32 counter = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 2, lreco = 0, rreco = 0, flag = 0;
	//static u8 loffset = 0, roffset = 5;
	
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
					
					lmSpeed = AgvGear[gearRecod] + BLG[0][gearRecod];
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
		
		
		if(lreco > rreco)
		{
			if(lreco - rreco >= 3)
			{
				if(BRG[0][gearRecod] > 0)
				{
					BRG[0][gearRecod]--;
				}
				else
				{
					BLG[0][gearRecod]++;
				}
				
				printf("loffset = %d, roffset = %d\r\n", BLG[0][gearRecod], BRG[0][gearRecod]);
				
				
				lreco = 0;
				rreco = 0;
			}
		}
		else if(lreco < rreco)
		{
			if(rreco - lreco >= 3)
			{
				if(BLG[0][gearRecod] > 0)
				{
					BLG[0][gearRecod]--;
				}
				else
				{
					BRG[0][gearRecod]++;
				}
				
				printf("loffset = %d, roffset = %d\r\n", BLG[0][gearRecod], BRG[0][gearRecod]);
				lreco = 0;
				rreco = 0;
			}
		}
		
		if(0 == flag)
		{
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				ctrlParasPtr->comflag = 12;
	
				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
				{
					if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
					{
						//if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Center)
						if(FMSDS_Pre_Ptr->AgvMSLocation - FMSDS_Ptr->AgvMSLocation > 0)
						{
							lreco++;
						}
						
					}
					
					//printf("lreco = %d\r\n", lreco);
	
					lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + BLG[0][gearRecod];
					rmSpeed = AgvGear[gearRecod];
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
	
					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);
				}
				else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3))
				{
					flag = 1;
					
					lmSpeed = 0;
					rmSpeed = 0;
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
	
					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);

					CHANGE_TO_STOP_MODE();
					Delay_ms(1000);
					CHANGE_TO_BACK_MODE();
					
					lmSpeed = AgvGear[2] + BLG[0][2] + BLG[2][gearRecod];
					rmSpeed = AgvGear[2];
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
	
					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);
				}
	
				
				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{		
				ctrlParasPtr->comflag = 13;

				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_3))
				{
					if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
					{
						//if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
						if(FMSDS_Ptr->AgvMSLocation - FMSDS_Pre_Ptr->AgvMSLocation > 0)
						{
							rreco++;
						}
						
					}
					
					
					lmSpeed = AgvGear[gearRecod];
					rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + BRG[0][gearRecod];
		
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
					
					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);

				}
				else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
				{
					flag = 1;
					
					lmSpeed = 0;
					rmSpeed = 0;
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
	
					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);

					CHANGE_TO_STOP_MODE();
					Delay_ms(1000);
					CHANGE_TO_BACK_MODE();
					
					lmSpeed = AgvGear[2];
					rmSpeed = AgvGear[2] + BRG[0][2] + BRG[2][gearRecod];
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
	
					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);
				}
				
				
				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				lmSpeed = AgvGear[gearRecod] + BLG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + BRG[0][gearRecod];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
			}

		}

		






		
		else
		{
			if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				flag = 0;
				
				lmSpeed = AgvGear[gearRecod] + BLG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + BRG[0][gearRecod];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
			}
		}
		
		
		
	}
	
}


void AGV_Correct_back_3(u8 gear)
{
	static u32 startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	
	
	if(lreco > rreco)
	{
		
		if(lreco - rreco >= 3)
		{
			
			if(BRightCompDuty[AgvGear[gearRecod]] > 0)
			{
				
				BRightCompDuty[AgvGear[gearRecod]]--;
			}
			else
			{
				
				BLeftCompDuty[AgvGear[gearRecod]]++;
			}
			
			printf("loffset = %d, roffset = %d\r\n", BLeftCompDuty[AgvGear[gearRecod]], BRightCompDuty[AgvGear[gearRecod]]);
			
			
			lreco = 0;
			rreco = 0;
		}
	}
	else if(lreco < rreco)
	{
		
		if(rreco - lreco >= 3)
		{
			
			if(BLeftCompDuty[AgvGear[gearRecod]] > 0)
			{
				
				BLeftCompDuty[AgvGear[gearRecod]]--;
			}
			else
			{
				
				BRightCompDuty[AgvGear[gearRecod]]++;
			}
			
			printf("loffset = %d, roffset = %d\r\n", BLeftCompDuty[AgvGear[gearRecod]], BRightCompDuty[AgvGear[gearRecod]]);
			
			lreco = 0;
			rreco = 0;
		}
	}



	
	if(0 == ctrlParasPtr->BSflag)
	{
			
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			
			ctrlParasPtr->comflag = 12;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				
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
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_3))
			{
				ctrlParasPtr->BSflag = 1;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_BACK_MODE();
				
				//lmSpeed = AgvGear[2] + FLG[0][2] + FLG[2][gearRecod];
				lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]] + FLG[2][gearRecod];
				rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
				
				lreco = rreco + 3;
			}

			
			
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 13;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_3))
			{
				
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					
					//if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
					if(FMSDS_Ptr->AgvMSLocation - FMSDS_Pre_Ptr->AgvMSLocation > 0)
					{
						rreco++;
					}
					
				}
				
				
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				
			}
			else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->BSflag = 1;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_BACK_MODE();
				
				lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]] + FRG[2][2];
				
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				
				rreco = lreco + 3;
			}
			
			
			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
			lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
			rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
			
			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			
		}
		
		
		
	}











	
	else
	{


		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 12;

			//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
			lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 13;

			lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]];
			rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);

			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation == Agv_MS_Center))
		{
			
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
				ctrlParasPtr->BSflag = 0;
			
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				//rmSpeed = AgvGear[gearRecod] + FRG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				startCount = 0;
			}
			
		}
	}
	
	
	
}


void AGV_Correct_back_4(u8 gear)
{
	static u32 startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0;
	//u8 LTM_flag = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	
	
	if(lreco > rreco)
	{
		
		if(lreco - rreco >= 3)
		{
			
			if(BRightCompDuty[AgvGear[gearRecod]] > 0)
			{
				
				BRightCompDuty[AgvGear[gearRecod]]--;
			}
			else
			{
				
				BLeftCompDuty[AgvGear[gearRecod]]++;
			}
			
			printf("loffset = %d, roffset = %d\r\n", BLeftCompDuty[AgvGear[gearRecod]], BRightCompDuty[AgvGear[gearRecod]]);
			
			
			lreco = 0;
			rreco = 0;
		}
	}
	else if(lreco < rreco)
	{
		
		if(rreco - lreco >= 3)
		{
			
			if(BLeftCompDuty[AgvGear[gearRecod]] > 0)
			{
				
				BLeftCompDuty[AgvGear[gearRecod]]--;
			}
			else
			{
				
				BRightCompDuty[AgvGear[gearRecod]]++;
			}
			
			printf("loffset = %d, roffset = %d\r\n", BLeftCompDuty[AgvGear[gearRecod]], BRightCompDuty[AgvGear[gearRecod]]);
			
			lreco = 0;
			rreco = 0;
		}
	}



	
	if(0 == ctrlParasPtr->BSflag)
	{
			
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			
			ctrlParasPtr->comflag = 12;

			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				
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
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
			}
			#if 1
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3))
			{
				ctrlParasPtr->BSflag = 1;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_BACK_MODE();
				
				//lmSpeed = AgvGear[2] + FLG[0][2] + FLG[2][gearRecod];
				lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]] + FLG[2][gearRecod];
				rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]];
				
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
			ctrlParasPtr->comflag = 13;

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_3))
			{
				
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					
					//if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Center)
					if(FMSDS_Ptr->AgvMSLocation - FMSDS_Pre_Ptr->AgvMSLocation > 0)
					{
						rreco++;
					}
					
				}
				
				
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				
			}
			#if 1
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				ctrlParasPtr->BSflag = 1;
				
				lmSpeed = 0;
				rmSpeed = 0;
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				CHANGE_TO_STOP_MODE();
				Delay_ms(1000);
				CHANGE_TO_BACK_MODE();
				
				lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]];
				//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
				rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]] + FRG[2][2];
				
				
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				rreco = lreco + 3;
			}
			#endif
			
			
		}
		#if 0
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			
			if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 631;
				ctrlParasPtr->dampingFlag = DampingLeft;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;

				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;

				MOTOR_RIGHT_DUTY_SET(lmSpeed);
			}
			else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 632;
				ctrlParasPtr->dampingFlag = DampingRight;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;
				
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(rmSpeed);
			}
			// 阻尼end
			//else if(AgvNone == FMSDS_Ptr->agvDirection)
			else
			{
				ctrlParasPtr->comflag = 633;

				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
			}
			
		}
		#endif
		
		
	}











	
	else
	{
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 12;

			//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
			lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]] + DutyTableLow[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);
			
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 13;

			lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]];
			rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]] + DutyTableLow[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(rmSpeed);
			MOTOR_RIGHT_DUTY_SET(lmSpeed);

			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation == Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation == Agv_MS_Center))
		{
			
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
				ctrlParasPtr->BSflag = 0;
			
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				//rmSpeed = AgvGear[gearRecod] + FRG[0][gearRecod];
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
				MOTOR_RIGHT_DUTY_SET(lmSpeed);

				startCount = 0;
			}
			
		}
	}
	
	

	if(DampingNone != ctrlParasPtr->dampingFlag)		// 如果启动阻尼
	{
		ctrlParasPtr->comflag = 67;
		
		if((SystemRunningTime - ctrlParasPtr->dampingTimeRec) > (FMSDS_Ptr->VelocityXt >> 0))		// 时间到, 恢复
		{
			ctrlParasPtr->comflag = 671;
			if(DampingLeft == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6711;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				
				MOTOR_RIGHT_DUTY_SET(lmSpeed);
				
			}
			else if(DampingRight == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6712;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(rmSpeed);
			}
			
		}
	}


	
}


void AGV_Correct_back_5(u8 gear)
{
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0;
	u32 centCount = 0;
	
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	
	gearRecod = gear;
	if((Agv_MS_CrossRoad != FMSDS_Pre_Ptr->AgvMSLocation) && (Agv_MS_Undefine != FMSDS_Pre_Ptr->AgvMSLocation) &&\
		(SubAbsV(FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation) <= 3))
	{
		//printf("AgvMSLocation %d, %d\r\n",FMSDS_Ptr->AgvMSLocation, FMSDS_Pre_Ptr->AgvMSLocation);
		if(0 == ctrlParasPtr->FSflag)
		{
			

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
			{
				
				ctrlParasPtr->comflag = 61;
					
				if(FMSDS_Ptr->MSD_Hex != FMSDS_Pre_Ptr->MSD_Hex)
				{
					//if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Center)
					if(FMSDS_Pre_Ptr->AgvMSLocation - FMSDS_Ptr->AgvMSLocation > 0)
					{
						lreco++;
					}
					
				}

		#if 0
				if(AgvCent2Left == FMSDS_Ptr->agvDirection) 			// 如果是简谐运动从中线往左偏移, 应该拉低右边电机duty, 以抵消往左偏的力量
				{
					if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将右边电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
					else												// 如果还没回到1格, 那么还应该持续拉右边电机的duty
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
						printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
					}
					
				}
				
				else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从左往中线回, 应该拉低左边电机的duty以达到减少回冲的速度
				{
					
					if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将左边的电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
					else												// 如果还没回到1格, 那么应该持续拉左边电机的duty
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						printf("2: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
					}
				}
		#endif

		#if 0
				if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将右边电机的duty恢复, 放开回拉的力量
				{
					lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				}
				else												// 如果还没回到1格, 那么还应该持续拉右边电机的duty
				{
					if(AgvLeft2Cent != FMSDS_Ptr->agvDirection)
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
					}
					rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
					printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				}

				if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 	// 如果是简谐运动从左往中线回, 应该拉低左边电机的duty以达到减少回冲的速度
				{
					
					if(FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将左边的电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
					else												// 如果还没回到1格, 那么应该持续拉左边电机的duty
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						printf("2: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
					}
				}
		#else

				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];

				if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将右边电机的duty恢复, 放开回拉的力量
				{
					rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
					//printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				}

				if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 	// 如果是简谐运动从左往中线回, 应该拉低左边电机的duty以达到减少回冲的速度
				{
					
					if(FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1)		// 如果已经回到1格了, 那么应该将左边的电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
						//printf("2: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
					}
				}
				
		#endif
				if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
				{
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(rmSpeed);
					MOTOR_RIGHT_DUTY_SET(lmSpeed);
				}
				else
				{
					printf("dutyErr1! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
				}
				
				//printf("lreco = %d\r\n", lreco);

				//lmSpeed = AgvGear[gearRecod] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][gearRecod];
				//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];

				
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{		
				ctrlParasPtr->comflag = 62;

				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_9))
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

			#if 0
					if(AgvCent2Right == FMSDS_Ptr->agvDirection)			// 如果是简谐运动从中线往右偏移, 应该拉低左边电机duty, 以抵消往右偏的力量
					{
						if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将左边电机的duty恢复, 放开回拉的力量
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						}
						else												// 如果还没回到1格, 那么还应该持续拉左边电机的duty
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
							printf("3: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
						}
						
					}
					
					else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从右往中线回, 应该拉低右边电机的duty以达到减少回冲的速度
					{
				#if 1
						if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将右边的电机的duty恢复, 放开回拉的力量
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						}
						else												// 如果还没回到1格, 那么应该持续拉右边电机的duty
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
							printf("4: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
						}
				#else
						
				#endif
					}
			#endif

			#if 0
					if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将左边电机的duty恢复, 放开回拉的力量
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
						rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
					}
					else												// 如果还没回到1格, 那么还应该持续拉左边电机的duty
					{
						lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
						if(AgvRight2Cent != FMSDS_Ptr->agvDirection)
						{
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						}
						printf("3: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
					}

					if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从右往中线回, 应该拉低右边电机的duty以达到减少回冲的速度
					{
						
						if(FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将右边的电机的duty恢复, 放开回拉的力量
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
						}
						else												// 如果还没回到1格, 那么应该持续拉右边电机的duty			
						{
							lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
							rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
							printf("4: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
						}
					}

			#else

					lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
					rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];

					if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1)
					{
						lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
						//printf("3: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
					}

					if(AgvRight2Cent == FMSDS_Ptr->agvDirection)		// 如果是简谐运动从右往中线回, 应该拉低右边电机的duty以达到减少回冲的速度
					{
						if(FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1)		// 如果已经回到1格了, 那么应该将右边的电机的duty恢复, 放开回拉的力量
						{
							rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));
							//printf("4: %d, %d\r\n", (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center), (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center)));
						}
					}
					
			#endif
					
					if((CHECK_MOTOR_SET_DUTY(lmSpeed)) && (CHECK_MOTOR_SET_DUTY(rmSpeed)))
					{
						ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
						ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

						MOTOR_LEFT_DUTY_SET(rmSpeed);
						MOTOR_RIGHT_DUTY_SET(lmSpeed);
					}
					else
					{
						printf("dutyErr2! lms = %d, rms = %d\r\n\r\n", lmSpeed, rmSpeed);
					}
					
					//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
					//rmSpeed = AgvGear[gearRecod] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1] + FRG[0][gearRecod];
					//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
		
					//ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					//ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
					
					//MOTOR_LEFT_DUTY_SET(lmSpeed);
					//MOTOR_RIGHT_DUTY_SET(rmSpeed);

					
				}
				else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_9) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
				{
					ctrlParasPtr->FSflag = 1;
					
					ctrlParasPtr->comflag = 622;
		#if 1
					lmSpeed = 0;
					rmSpeed = 0;
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);

					CHANGE_TO_STOP_MODE();
					Delay_ms(1000);
					CHANGE_TO_GO_STRAIGHT_MODE();
					
					lmSpeed = AgvGear[2] + BLeftCompDuty[AgvGear[2]];
					//rmSpeed = AgvGear[2] + FRG[0][2] + FRG[2][2];
					rmSpeed = AgvGear[2] + BRightCompDuty[AgvGear[2]] + FRG[2][2];
					
					
					ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
					ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

					MOTOR_LEFT_DUTY_SET(lmSpeed);
					MOTOR_RIGHT_DUTY_SET(rmSpeed);

					
					
		#endif
					rreco = lreco + 3;
				}

				
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->comflag = 63;

				// 加入阻尼模块
				// 阻尼begin
				
				lmSpeed = AgvGear[gearRecod] + BLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + BRightCompDuty[AgvGear[gearRecod]];
							
				
				
			}
		
		
		
		}

		
	}
	
	
	
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

void AGV_Correct(void)
{
	u32 result = 0;
	u8 range = 3, lmSpeed = 0, rmSpeed = 0;
	static u8 duty = 100;
	static u8 lcompduty = 0, rcompduty = 0;


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
		else					// 	在误差范围内, 则打印数据, 然后换挡		
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

		//result = ctrlParasPtr->leftHallIntervalTime - ctrlParasPtr->rightHallIntervalTime;
		result = SubAbsV(ctrlParasPtr->HLavg, ctrlParasPtr->HRavg);
		
		//printf("res = %d, HLavg = %d, HRavg = %d\r\n", result, ctrlParasPtr->HLavg, ctrlParasPtr->HRavg);

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
			lcompduty = 0;
			rcompduty = 0;
			
			duty--;
			
		}

		lmSpeed = duty + lcompduty;
		rmSpeed = duty + rcompduty;
		//printf("ls = %d, rs = %d\r\n\r\n", lmSpeed, rmSpeed);
		//printf("\r\n");
		

		
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

		#endif
		
		
		ctrlParasPtr->avgFlag = 0;
	}
	
	
	
}


void AGV_Correct_1(void)
{
	u32 result = 0;
	u8 range = 0, lmSpeed = 0, rmSpeed = 0;
	static u8 duty = 15;
	static u8 lcompduty = 0, rcompduty = 0;
	static u8 gear = 3;


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
		

		
		if(gear >= 5)
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
	u8 flag = 2;

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

		if((FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_4) || (FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_4))
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
	
}


void AGV_Walking_Test(void)
{
	MOTOR_POWER = 0;
	
	#if 1
	CHANGE_TO_GO_STRAIGHT_MODE();
	//CHANGE_TO_TEST_MODE();
	//CHANGE_TO_CIR_LEFT_MODE();
	//CHANGE_TO_CIR_LEFT_MODE();
	#else
	CHANGE_TO_BACK_MODE();
	#endif
	//MOTOR_LEFT_STOP_PIN_SET();
	#if 1
	ctrlParasPtr->settedSpeed = AgvGear[8];
	ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	
	MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
	MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
	#endif
	//CHANGE_TO_BACK_MODE();
}

void AGV_Walking_Test2(void)
{
	MOTOR_POWER = 0;

	#if 1
	CHANGE_TO_GO_STRAIGHT_MODE();
	#else
	CHANGE_TO_BACK_MODE();
	#endif
	//MOTOR_LEFT_STOP_PIN_SET();
	ctrlParasPtr->settedSpeed = 100;
	ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	
	MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->settedSpeed);
	MOTOR_LEFT_DUTY_SET(ctrlParasPtr->settedSpeed);
}

void STATION_1AND2_WalkControl(void)
{
	
	if(step_gS == ctrlParasPtr->walkingstep)
	{
		//CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
		CHANGE_TO_GO_STRAIGHT_MODE();
		
		ctrlParasPtr->gear = 7;

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
		else if(1 == ctrlParasPtr->crossRoadCount)
		{
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}

	
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 7;
		
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
		
		ctrlParasPtr->gear = 7;
		
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			//printf("2LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
			
			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				
				//printf("info: %d, %d, %d, %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->crossRoadCount, ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
				ctrlParasPtr->walkingstep = step_gVeer;
			}
		}
		else if(2 == ctrlParasPtr->crossRoadCount)
		{
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				
				//printf("info: %d, %d, %d, %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->crossRoadCount, ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
				ctrlParasPtr->walkingstep = step_gVeer;
			}
		}
		
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 7;

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
		
		ctrlParasPtr->gear = 7;

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			//printf("updateFlag = %d, cr = %d\r\n", RFID_Info_Ptr->updateFlag, ctrlParasPtr->crossRoadCount);
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
		else if(3 == ctrlParasPtr->crossRoadCount)
		{
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}
		
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 7;

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
		
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			ctrlParasPtr->gear = 7;
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			//printf("4LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
			
		}
		else if(4 == ctrlParasPtr->crossRoadCount)
		{
			ctrlParasPtr->gear = 7;
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}
		
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 7;

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

		ctrlParasPtr->gear = 7;
			
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
		else if(5 == ctrlParasPtr->crossRoadCount)
		{
			if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGS[ctrlParasPtr->crossRoadCount].HallCountLeft))
			{
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				ctrlParasPtr->walkingstep = step_gVeer;
				
			}
		}
		
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		ctrlParasPtr->gear = 7;

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;

			RFID_Info_Ptr->rfidData = 0;
		}

	}
}



void Hall_Count(void)
{
	u8 cir = 0;
	static u32 rfid = 0x0000;
	
	static Agv_MS_Location mslRec = AgvInits;
	/*
	if(goStraightStatus == ctrlParasPtr->agvStatus)
	{
		if((0x0000 != FMSDS_Ptr->MSD_Hex) && (0xFFFF != FMSDS_Ptr->MSD_Hex))
		{
			if(1 == RFID_Info_Ptr->updateFlag)
			{
				
				if(rfid != RFID_Info_Ptr->rfidData)
				{
					rfid = RFID_Info_Ptr->rfidData;
					HallCountArr[RFID_Info_Ptr->rfidData].HallCountLeft = ctrlParasPtr->leftHallCounter;
					HallCountArr[RFID_Info_Ptr->rfidData].HallCountRight = ctrlParasPtr->rightHallCounter;
					ctrlParasPtr->leftHallCounter = 0;
					ctrlParasPtr->rightHallCounter = 0;
					
					for(cir = 1; cir <= RFID_Info_Ptr->rfidData; cir++)
					{
						printf("ID = %d, HCL = %d, HCR = %d\r\n", cir, HallCountArr[cir].HallCountLeft, HallCountArr[cir].HallCountRight);
					}
				}
				
			}
		}
		
	}
	*/

	if((goStraightStatus == ctrlParasPtr->agvStatus) || (gSslow == ctrlParasPtr->agvStatus))
	{
		if(mslRec != FMSDS_Ptr->AgvMSLocation)
		{
			mslRec = FMSDS_Ptr->AgvMSLocation;
			
			if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
			{
				ctrlParasPtr->rightHallCounter = 0;
				ctrlParasPtr->leftHallCounter = 0;
				printf("1Hclean\r\n");
				
			}
		}
	}
	else if(backStatus == ctrlParasPtr->agvStatus)
	{
		if(mslRec != FMSDS_Ptr->AgvMSLocation)
		{
			mslRec = FMSDS_Ptr->AgvMSLocation;
			
			if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
			{
				ctrlParasPtr->rightHallCounter = 0;
				ctrlParasPtr->leftHallCounter = 0;
				printf("2Hclean\r\n");
			}
		}
	}
	else if(bSslow == ctrlParasPtr->agvStatus)
	{
		if(mslRec != FMSDS_Ptr->AgvMSLocation)
		{
			mslRec = FMSDS_Ptr->AgvMSLocation;
			
			if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
			{
				ctrlParasPtr->rightHallCounter = 0;
				ctrlParasPtr->leftHallCounter = 0;
				printf("3Hclean\r\n");
			}
		}
	}
	
	
}


void CrossRoad_Count(void)
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
					if(ctrlParasPtr->crossRoadCount < 5)
					{
						ctrlParasPtr->rightHallCounter = 0;
						ctrlParasPtr->leftHallCounter = 0;
						ctrlParasPtr->crossRoadCount++;
						
						printf("1Hclean\r\n");
						printf("crossRoadCount = %d\r\n", ctrlParasPtr->crossRoadCount);
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
					if(ctrlParasPtr->crossRoadCount > 0)
					{
						ctrlParasPtr->rightHallCounter = 0;
						ctrlParasPtr->leftHallCounter = 0;
						ctrlParasPtr->crossRoadCount--;
						
						printf("2Hclean\r\n");
						printf("crossRoadCount = %d\r\n", ctrlParasPtr->crossRoadCount);
					}
					else
					{
						printf("crossRoadCount error\r\n");
					}
				}
				
			}

			
		}
	}
	
	
}

void Walking_Step_Controler(void)
{
	u8 flag = 0;
	
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
		static u8 stepFlag = 0;
		
		if((SpinStation_1 == ctrlParasPtr->goalStation) || \
			(SpinStation_3 == ctrlParasPtr->goalStation) || \
			(SpinStation_5 == ctrlParasPtr->goalStation) || \
			(SpinStation_7 == ctrlParasPtr->goalStation) || \
			(SpinStation_9 == ctrlParasPtr->goalStation))
		{
			CHANGE_TO_CIR_LEFT_MODE();
			//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
			printf("info1: %d, %d, %d, %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->crossRoadCount, ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
		}
		else if((SpinStation_2 == ctrlParasPtr->goalStation) || \
				(SpinStation_4 == ctrlParasPtr->goalStation) || \
				(SpinStation_6 == ctrlParasPtr->goalStation) || \
				(SpinStation_8 == ctrlParasPtr->goalStation) || \
				(SpinStation_10 == ctrlParasPtr->goalStation))
		{
			CHANGE_TO_CIR_RIGHT_MODE();
			//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
			printf("info2: %d, %d, %d, %d\r\n", ctrlParasPtr->goalRFIDnode, ctrlParasPtr->crossRoadCount, ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
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
	else if(step_entry == ctrlParasPtr->walkingstep)
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
		
		//if((0 == LMT_IN1) || (0 == LMT_IN2))		// 接近开关响应或者是超过磁条了
		if((0 == LMT_IN1) || (0 == LMT_IN2) || (0x0000 == FMSDS_Ptr->MSD_Hex))
		{
			CHANGE_TO_STOP_MODE();
			//Delay_ns(1);
			
			RFID_Info_Ptr->rfidData = 0;
			flag = 0;
			ctrlParasPtr->walkingstep = step_catch;
		}

		#endif	
	}
	else if(step_catch == ctrlParasPtr->walkingstep)
	{
		#if 0
		
		if(1 == LMT_SW)		// 已经抓到货物了
		{
			//Delay_ns(3);
			
			ECV_POWER_OFF();

			ctrlParasPtr->walkingstep = step_exit;
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
	else if(step_exit == ctrlParasPtr->walkingstep)
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
			

			if(1 == fla)
			{
				//printf("LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
				if((ctrlParasPtr->rightHallCounter >= CrossRoadHallCountArrGB[ctrlParasPtr->crossRoadCount].HallCountRight) ||\
				(ctrlParasPtr->leftHallCounter >= CrossRoadHallCountArrGB[ctrlParasPtr->crossRoadCount].HallCountLeft))
				{
					fla = 0;
					CHANGE_TO_STOP_MODE();
					//Delay_ns(1);
					ctrlParasPtr->walkingstep = step_weigh;
				}
			}
			
			
		}
	}
	else if(step_weigh == ctrlParasPtr->walkingstep)
	{
		ECV_POWER_ON();
		FECV_STOP();
		BECV_STOP();
		WECV_UP();
		Delay_ns(3);
		
		FECV_UP();
		BECV_UP();

		Delay_ns(3);
		WECV_STOP();

		WECV_DOWN();
		Delay_ns(6);

		
		ECV_POWER_OFF();
		ctrlParasPtr->walkingstep = step_bVeer;
	}
	else if(step_bVeer == ctrlParasPtr->walkingstep)
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
						stepFlag = 0;
						ctrlParasPtr->BSflag = 0;
						ctrlParasPtr->walkingstep = step_gB;
					}
				}
				
				
			}
		}
		
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		CHANGE_TO_BACK_MODE();
		FECV_DOWN();
		BECV_DOWN();
		ECV_POWER_ON();
		//ctrlParasPtr->BSflag = 1;
		if(0x0000 == FMSDS_Ptr->MSD_Hex)
		{
			CHANGE_TO_STOP_MODE();
			Delay_ns(3);
			ctrlParasPtr->FSflag = 0;
			ctrlParasPtr->crossRoadCount = 0;
			//Delay_ns(5);
			ECV_POWER_OFF();
			RFID_Info_Ptr->updateFlag = 0;
			ctrlParasPtr->walkingstep = step_stop;

			ctrlParasPtr->goalStation = ControlCenter;
			ctrlParasPtr->goalRFIDnode = 0x0000;
		}
	}
	
	
}

void AGV_Walking_Stop(void)
{	
	ctrlParasPtr->settedSpeed = 0;
	ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	
	MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->settedSpeed);
	MOTOR_LEFT_DUTY_SET(ctrlParasPtr->settedSpeed);

	CHANGE_TO_STOP_MODE();

	MOTOR_POWER = 1;
}


void AGV_SpeedCalu(void)
{
	
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
	
}


void Motion_Ctrl_Init(void)
{
	u8 cir = 0;
	Motion_Ctrl_GPIO_Init();
	
	PG_EXTI_CFG();
	
	CHANGE_TO_STOP_MODE();
	
	E_pushrod_Gpio_Init();

	SW_Gpio_Init();
	
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
	ctrlParasPtr->crossRoadCount = 0;
	
	x3x2x1_left_setting();
	x3x2x1_right_setting();
	
	motionOptsPtr->motor_up = motion_up;
	motionOptsPtr->motor_down = motion_down;
	motionOptsPtr->motor_left = motion_left;
	motionOptsPtr->motor_right = motion_right;
	motionOptsPtr->motor_stop = motion_stop;
	motionOptsPtr->agv_walk_stop = AGV_Walking_Stop;
	motionOptsPtr->agv_walk_test2 = AGV_Walking_Test2;

	agv_walking_func[stopStatus] = walking_stopStatus;
	agv_walking_func[goStraightStatus] = walking_goStraight;
	agv_walking_func[backStatus] = walking_backStatus;
	agv_walking_func[cirLeft] = walking_cirLeft;
	agv_walking_func[cirRight] = walking_cirRight;


	ZBandRFIDmapping[ControlCenter] = 0x0000;
	//ZBandRFIDmapping[SpinStation_2] = 0xD358;
	ZBandRFIDmapping[SpinStation_1] = 0x0001;
	ZBandRFIDmapping[SpinStation_2] = 0x0002;
	//ZBandRFIDmapping[SpinStation_3] = 0xD358;
	//ZBandRFIDmapping[SpinStation_2] = 0xF1C3;
	ZBandRFIDmapping[SpinStation_3] = 0x0003;
	ZBandRFIDmapping[SpinStation_4] = 0x0004;
	ZBandRFIDmapping[SpinStation_5] = 0x0005;
	ZBandRFIDmapping[SpinStation_6] = 0x0006;
	ZBandRFIDmapping[SpinStation_7] = 0x0007;
	ZBandRFIDmapping[SpinStation_8] = 0x0008;
	ZBandRFIDmapping[SpinStation_9] = 0x0009;
	ZBandRFIDmapping[SpinStation_10] = 0x000a;

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

		adaptInfoB[cir].result = Good;
		adaptInfoB[cir].timRec = 0;
		adaptInfoB[cir].duty = 0;
		adaptInfoB[cir].goodDuty = 0;
	}
	
	adaptInfo[0].timRec = 0;
	adaptInfo[0].duty = 6;

	adaptInfo[1].timRec = 0;
	adaptInfo[1].duty = 5;

	adaptInfo[2].timRec = 0;
	adaptInfo[2].duty = 4;

	adaptInfo[3].timRec = 0;
	adaptInfo[3].duty = 4;

	adaptInfo[4].timRec = 0;
	adaptInfo[4].duty = 3;

	adaptInfo[5].timRec = 0;
	adaptInfo[5].duty = 3;

	adaptInfo[6].timRec = 0;
	adaptInfo[6].duty = 2;

	adaptInfo[7].timRec = 0;
	adaptInfo[7].duty = 2;

	adaptInfo[8].timRec = 0;
	adaptInfo[8].duty = 2;

	adaptInfo[9].timRec = 0;
	adaptInfo[9].duty = 1;

	adaptInfo[10].timRec = 0;
	adaptInfo[10].duty = 1;

	
	adaptInfoB[0].timRec = 0;
	adaptInfoB[0].duty = 6;

	adaptInfoB[1].timRec = 0;
	adaptInfoB[1].duty = 5;

	adaptInfoB[2].timRec = 0;
	adaptInfoB[2].duty = 5;

	adaptInfoB[3].timRec = 0;
	adaptInfoB[3].duty = 4;

	adaptInfoB[4].timRec = 0;
	adaptInfoB[4].duty = 4;

	adaptInfoB[5].timRec = 0;
	adaptInfoB[5].duty = 3;

	adaptInfoB[6].timRec = 0;
	adaptInfoB[6].duty = 3;

	adaptInfoB[7].timRec = 0;
	adaptInfoB[7].duty = 2;

	adaptInfoB[8].timRec = 0;
	adaptInfoB[8].duty = 2;

	adaptInfoB[9].timRec = 0;
	adaptInfoB[9].duty = 1;

	adaptInfoB[10].timRec = 0;
	adaptInfoB[10].duty = 1;
	
}



