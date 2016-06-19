#include "motion_control.h"
#include "cfg_gpio.h"
#include "timer_opts.h"
#include "pwm_opts.h"
#include "magn_sensor.h"
#include "zigbee.h"

u16 ZBandRFIDmapping[11];

u8 AgvGear[MAX_GEAR_NUM] = {0, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
u8 AgvGearCompDutyLF[MAX_GEAR_NUM] = {0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
u8 AgvGearCompDutyRF[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 AgvGearCompDutyLB[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 AgvGearCompDutyRB[MAX_GEAR_NUM] = {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
u8 AgvGearK[MAX_GEAR_NUM] = {1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 2, 2, 2, 16, 17, 18, 19, 20, 21, 22};


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

void AGV_Correct_gS_8(u8 gear)		// 3 mode
{
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;
	static u8 mode = 0, modeFlag = 0xff;
	
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
		if(0 == ctrlParasPtr->FSflag)		// 普通模式下
		{
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 左偏移
			{
				
				ctrlParasPtr->comflag = 61;
					
				if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_3) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
				{
					// 偏差在三格以内, 跑正常模式代码
					mode = 1;
				}
				else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_8) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_3))
				{
					// 超过三格, 跑振荡模式
					mode = 2;
				}
				else
				{
					// 超过9格, 跑失控修正模式
					mode = 0;
				}

			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))	// 右偏移
			{		
				ctrlParasPtr->comflag = 62;

				if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_3))
				{
					// 偏差在三格以内, 跑正常模式代码
					mode = 1;
				}
				else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_3) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_8))
				{
					// 超过三格, 跑振荡模式
					mode = 2;
				}
				else
				{
					// 超过9格, 跑失控修正模式
					mode = 0;
				}
				
			}
			
		}
		else	// 失控/启动修正模式, 进入修正控制	
		{
			mode = 0;
		}

		if(modeFlag != mode)
		{
			printf("mode = %d\r\n", mode);
			modeFlag = mode;
		}
		
		/********实现*********/

		lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
		rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];

		if(0 == mode)		
		{
			// 失控/启动修正模式
			#if 1

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
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
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
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
			}
			
			
			#else

			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
				//printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				ctrlParasPtr->comflag = 12;
				
				startCount = 0;
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));				
				//printf("2: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				ctrlParasPtr->comflag = 13;

				startCount = 0;
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
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
						//ctrlParasPtr->FSflag = 0;
						
						startCount = 0;
					}
				}
				
			}

			#endif

			
		}
		else if(1 == mode)
		{
			// 普通模式,偏差在三格之内
			
			if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))
			{
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation));
				//printf("1: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod]* (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				ctrlParasPtr->comflag = 12;
				
				startCount = 0;
			}
			else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
			{
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - (AgvGearK[gearRecod] * (FMSDS_Ptr->AgvMSLocation - Agv_MS_Center));				
				//printf("2: %d, %d\r\n", (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation), (AgvGearK[gearRecod] * (Agv_MS_Center - FMSDS_Ptr->AgvMSLocation)));
				ctrlParasPtr->comflag = 13;

				startCount = 0;
			}
			else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
			}
			
		}
		else if(2 == mode)
		{
			// 振荡模式
			
			
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
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
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


void RFID_Node_Analy(void)
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

		if(0x0000 == FMSDS_Ptr->MSD_Hex)
		{
			
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->BSflag = 1;
				backStatus_change();
				printf("backStatus_change\r\n");
				
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->FSflag = 1;
				goStraight_change();
				printf("goStraight_change\r\n");
			}
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
	ctrlParasPtr->settedSpeed = AgvGear[5];
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
		CHANGE_TO_GO_STRAIGHT_MODE();
		
		ctrlParasPtr->gear = 7;

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			printf("LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				Delay_ms(500);
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
			printf("LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				Delay_ms(500);
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
			RFID_Info_Ptr->updateFlag = 0;
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			printf("LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				Delay_ms(500);
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
		
		ctrlParasPtr->gear = 7;

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			printf("LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				Delay_ms(500);
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
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
			printf("LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				Delay_ms(500);
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
		static u8 stepFlag = 0;
		
		if((SpinStation_1 == ctrlParasPtr->goalStation) || \
			(SpinStation_3 == ctrlParasPtr->goalStation) || \
			(SpinStation_5 == ctrlParasPtr->goalStation) || \
			(SpinStation_7 == ctrlParasPtr->goalStation) || \
			(SpinStation_9 == ctrlParasPtr->goalStation))
		{
			CHANGE_TO_CIR_LEFT_MODE();
			
		}
		else if((SpinStation_2 == ctrlParasPtr->goalStation) || \
				(SpinStation_4 == ctrlParasPtr->goalStation) || \
				(SpinStation_6 == ctrlParasPtr->goalStation) || \
				(SpinStation_8 == ctrlParasPtr->goalStation) || \
				(SpinStation_10 == ctrlParasPtr->goalStation))
		{
			CHANGE_TO_CIR_RIGHT_MODE();
			
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
				ctrlParasPtr->gear = 1;
				stepFlag = 2;
			}
		}
		else if(2 == stepFlag)
		{	
			if((0xFFFF != FMSDS_Ptr->MSD_Hex) || (0xFFFF != RMSDS_Ptr->MSD_Hex))
			{
				if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Center))
				{
					CleanAllSpeed();
					
					CHANGE_TO_STOP_MODE();
					Delay_ms(1000);
					stepFlag = 0;
					ctrlParasPtr->walkingstep = step_entry;
				}
				#if 1
				else if((RMSDS_Ptr->AgvMSLocation >= Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
				{
					CleanAllSpeed();
					
					CHANGE_TO_STOP_MODE();
					Delay_ns(1);
					stepFlag = 0;
					ctrlParasPtr->walkingstep = step_entry;
				}
				#endif
			}
		}
		
	}
	else if(step_entry == ctrlParasPtr->walkingstep)
	{
		static u8 flag = 0;
		
		CHANGE_TO_GO_STRAIGHT_MODE();
		

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
			ctrlParasPtr->gear = 4;
		}
		else
		{
			ctrlParasPtr->gear = 2;
		}
		
		if((0 == LMT_IN1) || (0 == LMT_IN2))		// 接近开关响应或者是超过磁条了
		{
			CHANGE_TO_STOP_MODE();
			Delay_ns(1);
			
			RFID_Info_Ptr->rfidData = 0;
			flag = 0;
			ctrlParasPtr->walkingstep = step_catch;
		}

		#endif	
	}
	else if(step_catch == ctrlParasPtr->walkingstep)
	{
		#if 1
		
		if(1 == LMT_SW)		// 已经抓到货物了
		{
			//Delay_ns(3);
			
			ECV_POWER_OFF();

			ctrlParasPtr->walkingstep = step_exit;
		}
		else
		{
			FECV_UP();
			//BECV_UP();
			ECV_POWER_ON();
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
		
		#endif
		
	}
	else if(step_exit == ctrlParasPtr->walkingstep)
	{
		CHANGE_TO_BACK_MODE();
		
		ctrlParasPtr->gear = 4;
		
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			//printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);

			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CHANGE_TO_STOP_MODE();
				Delay_ns(1);
				ctrlParasPtr->walkingstep = step_weigh;
			}
		}
	}
	else if(step_weigh == ctrlParasPtr->walkingstep)
	{
		FECV_UP();
		BECV_DOWN();
		
		ECV_POWER_ON();
		
		Delay_ns(2);
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
				ctrlParasPtr->gear = 1;
				stepFlag = 2;
			}
		}
		else if(2 == stepFlag)
		{	
			if((0xFFFF != FMSDS_Ptr->MSD_Hex) || (0xFFFF != RMSDS_Ptr->MSD_Hex))
			{
				if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Center))
				{
					CleanAllSpeed();
					
					CHANGE_TO_STOP_MODE();
					Delay_ms(1000);
					stepFlag = 0;
					ctrlParasPtr->walkingstep = step_gB;
				}
				#if 1
				else if((RMSDS_Ptr->AgvMSLocation >= Agv_MS_Center) && (RMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
				{
					CleanAllSpeed();
					
					CHANGE_TO_STOP_MODE();
					Delay_ms(500);
					stepFlag = 0;
					ctrlParasPtr->walkingstep = step_gB;
				}
				#endif
			}
		}
		
	}
	else if(step_gB == ctrlParasPtr->walkingstep)
	{
		CHANGE_TO_BACK_MODE();
		
		if(0x0000 == FMSDS_Ptr->MSD_Hex)
		{
			CHANGE_TO_STOP_MODE();
			FECV_DOWN();
			BECV_DOWN();
			ECV_POWER_ON();
			Delay_ns(5);
			ECV_POWER_OFF();
			
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
	ctrlParasPtr->FSflag = 1;
	ctrlParasPtr->BSflag = 1;
	ctrlParasPtr->dampingFlag = DampingNone;
	ctrlParasPtr->goalRFIDnode = 0;
	ctrlParasPtr->goalStation = ControlCenter;
	ctrlParasPtr->walkingstep = step_stop;
	
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
	//ZBandRFIDmapping[SpinStation_1] = 0xD358;
	ZBandRFIDmapping[SpinStation_1] = 0x0001;
	ZBandRFIDmapping[SpinStation_2] = 0x0002;
	//ZBandRFIDmapping[SpinStation_3] = 0xD358;
	//ZBandRFIDmapping[SpinStation_4] = 0xF1C3;
	ZBandRFIDmapping[SpinStation_3] = 0x0003;
	ZBandRFIDmapping[SpinStation_4] = 0x0004;
	ZBandRFIDmapping[SpinStation_5] = 0x0005;
	ZBandRFIDmapping[SpinStation_6] = 0x0006;
	ZBandRFIDmapping[SpinStation_7] = 0x0007;
	ZBandRFIDmapping[SpinStation_8] = 0x0008;
	ZBandRFIDmapping[SpinStation_9] = 0x0009;
	ZBandRFIDmapping[SpinStation_10] = 0x000a;
}



