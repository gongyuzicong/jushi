#include "motion_control.h"
#include "cfg_gpio.h"
#include "timer_opts.h"
#include "pwm_opts.h"
#include "magn_sensor.h"



u8 AgvGear[MAX_GEAR_NUM] = {0, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
u8 AgvGearCompDutyL[MAX_GEAR_NUM] = {0, 2, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};


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
						 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,\
						 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,\
						 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,\
						 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,\
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

u8 DutyTableLow[10] = {2, 4, 6, 8, 10, 14, 14, 14, 14, 14};
u8 DutyTable[10] = {2, 8, 15, 20, 25, 30, 30, 30, 30, 30};
//u8 DutyTable[10] = {2, 4, 6, 8, 8, 8, 8, 8, 8, 8};

u8 DutyTable_Duty10[10] = {0, 0, 2, 3, 4, 5, 6, 7, 8, 9};


ControlerParaStruct ctrlParas;
ControlerParaStruct_P ctrlParasPtr = &ctrlParas;
MotionOperaterStruct motionOpts;
MotionOperaterStruct_P motionOptsPtr = &motionOpts;

u32 responseTime = 0;

void (*agv_walking_func[5]) (void);

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


void AGVPattern(void)
{
	
	
	
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

void walking_goStraight(void)
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

void walking_backStatus(void)
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

void walking_cirLeft(void)
{
	static u8 lmSpeed = 0, rmSpeed = 0, stepFlag = 0;
	u8 dutyTable[10] = {1, 5, 5, 5, 5, 5, 5, 6, 7, 8};

	if(0 == stepFlag)
	{
		lmSpeed = rmSpeed = 15;
		
		MOTOR_LEFT_DUTY_SET(lmSpeed);
		MOTOR_RIGHT_DUTY_SET(rmSpeed);

		if(0xFFFF == FMSDS_Ptr->MSD_Hex)
		{
			stepFlag = 1;
		}
	}
	else if(1 == stepFlag)
	{
		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_End))
		{
			lmSpeed = rmSpeed = dutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
			
			if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
			{
				stepFlag = 0;
				goStraight_change();
			}
			
		}
		else
		{
			lmSpeed = rmSpeed = 7;
		}
		
		MOTOR_LEFT_DUTY_SET(lmSpeed);
		MOTOR_RIGHT_DUTY_SET(rmSpeed);
	}
	
	
}

void walking_cirRight(void)
{
	
}

void walking_stopStatus(void)
{
	//motion_stop_pwm();
}


/**********Motor Basic Control Mode: End****************/

void AGV_Correct_gS(void)
{
	static u32 counter = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 2, lreco = 0, rreco = 0;
	static u8 loffset = 0, roffset = 5;
	
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
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;

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

				
				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
				}
				
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


				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					rreco = lreco + 3;
				}
				
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

			if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->changeModeFlag = 0;
			}
			
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
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;
	
	
	ctrlParasPtr->comflag = 2;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;

	
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

				

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->comflag = 21121;
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					
				}
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


				

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->comflag = 211321;
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					
				}
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

			if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->changeModeFlag = 0;
			}
			
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
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0, gearRecod = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 9;
	
	
	counter = 0;
	
	
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

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					
				}
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

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->comflag = 9141;
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					ctrlParasPtr->comflag = 9142;
					rreco = lreco + 3;
				}
				
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

			if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->changeModeFlag = 0;
			}
			
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
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	
	
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

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
				}
				
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

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					rreco = lreco + 3;
				}
				
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


			if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->changeModeFlag = 0;
			}
		}
		
		
		
	}











	
	else
	{


		if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
		{
			ctrlParasPtr->comflag = 12;

			//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
			//lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			//rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]];
			lmSpeed = AgvGear[2] + AgvGearCompDutyL[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation];
			rmSpeed = AgvGear[2];
			
			ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
			ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

			MOTOR_LEFT_DUTY_SET(lmSpeed);
			MOTOR_RIGHT_DUTY_SET(rmSpeed);
			
			
			startCount = 0;
		}
		else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
		{		
			ctrlParasPtr->comflag = 13;

			//lmSpeed = AgvGear[2] + FLeftCompDuty[AgvGear[2]];
			//rmSpeed = AgvGear[2] + FRightCompDuty[AgvGear[2]] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];
			lmSpeed = AgvGear[2] + AgvGearCompDutyL[2];
			rmSpeed = AgvGear[2] + DutyTable[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_1];

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
				ctrlParasPtr->BSflag = 0;
			
				//lmSpeed = AgvGear[gearRecod] + FLG[0][gearRecod];
				//rmSpeed = AgvGear[gearRecod] + FRG[0][gearRecod];
				//lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];
				//rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyL[2];
				rmSpeed = AgvGear[gearRecod];
	
				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_LEFT_DUTY_SET(lmSpeed);
				MOTOR_RIGHT_DUTY_SET(rmSpeed);

				startCount = 0;
			}
			
		}
	}
	
	
	
}

void AGV_Correct_gS_5(u8 gear)
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
		gearRecod = gear;
		
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

				
				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
				}
				
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


				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					rreco = lreco + 3;
				}
				
			}

			
		}
		else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
		{
			ctrlParasPtr->comflag = 63;

			// 加入阻尼模块
			// 阻尼begin
			
			if(AgvLeft2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 631;
				ctrlParasPtr->dampingFlag = DampingLeft;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;

				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]] - DutyTable[1];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
			}
			else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)			// 如果是左偏之后拉回来的
			{
				ctrlParasPtr->comflag = 632;
				ctrlParasPtr->dampingFlag = DampingRight;
				ctrlParasPtr->dampingTimeRec = SystemRunningTime;
				
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]] - DutyTable[1];
				
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
				
				MOTOR_RIGHT_DUTY_SET(rmSpeed);
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
			

			if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->changeModeFlag = 0;
			}
			
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
				lmSpeed = AgvGear[gearRecod] + FLeftCompDuty[AgvGear[gearRecod]];

				ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
				
				MOTOR_LEFT_DUTY_SET(lmSpeed);
				
			}
			else if(DampingRight == ctrlParasPtr->dampingFlag)
			{
				ctrlParasPtr->comflag = 6712;
				ctrlParasPtr->dampingFlag = DampingNone;
				FMSDS_Ptr->agvDirection = AgvNone;
				rmSpeed = AgvGear[gearRecod] + FRightCompDuty[AgvGear[gearRecod]];
				
				ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

				MOTOR_RIGHT_DUTY_SET(rmSpeed);
			}
			
		}
	}
	
}




void AGV_Correct_back_1(void)
{
	static u32 counter = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 2, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 0;
	
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
	static u8 loffset = 0, roffset = 5;
	
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
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	
	
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

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
				}
				
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

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					rreco = lreco + 3;
				}
				
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


			if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->changeModeFlag = 0;
			}
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
	static u32 counter = 0, startCount = 0;
	static u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 3, lreco = 0, rreco = 0, flag = 0;
	static u8 loffset = 0, roffset = 5;
	static Agv_MS_Location MSLRecode = AgvInits;
	u8 LTM_flag = 0;
	u32 centCount = 0;

	gearRecod = gear;
	
	ctrlParasPtr->comflag = 6;
	
	//if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)

	counter = 0;
	
	
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

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
				}
				
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

				if(ctrlParasPtr->changeModeFlag)
				{
					ctrlParasPtr->changeModeFlag = 0;
				}
				else
				{
					rreco = lreco + 3;
				}
				
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


			if(RMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
			{
				ctrlParasPtr->changeModeFlag = 0;
			}
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




void AVG_Calu_Program(void)
{
	static u32 HLavg = 0, HRavg = 0, TLpre = 0, TLnow = 0, TRpre = 0, TRnow = 0;
	
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
	static u8 gearRecod = 4, lreco = 0, rreco = 0, flag = 0, duty = 100;
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
	static u8 gearRecod = 4, lreco = 0, rreco = 0, flag = 0, duty = 15;
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




void AGV_Walking(void)
{
	
	if(AutomaticMode == ctrlParasPtr->agvWalkingMode)
	{
		
		agv_walking_func[ctrlParasPtr->agvStatus]();
		
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
	static u8 flag = 1;
	
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
				ctrlParasPtr->changeModeFlag = 1;
				ctrlParasPtr->BSflag = 1;
				backStatus_change();
				printf("backStatus_change\r\n");
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				ctrlParasPtr->changeModeFlag = 1;
				ctrlParasPtr->FSflag = 1;
				goStraight_change();
				printf("goStraight_change\r\n");
			}
		}
		
		#endif
	}

}

void AGV_Walking_Test(void)
{
	MOTOR_POWER = 0;
	
	#if 1
	CHANGE_TO_GO_STRAIGHT_MODE();
	//CHANGE_TO_TEST_MODE();
	//CHANGE_TO_CIR_LEFT_MODE();
	#else
	CHANGE_TO_BACK_MODE();
	#endif
	//MOTOR_LEFT_STOP_PIN_SET();
	#if 1
	ctrlParasPtr->settedSpeed = AgvGear[3];
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

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
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

void Motion_Ctrl_Init(void)
{
	
	Motion_Ctrl_GPIO_Init();
	
	PG_EXTI_CFG();
	
	CHANGE_TO_STOP_MODE();
	
	
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
	ctrlParasPtr->changeModeFlag = 0;
	ctrlParasPtr->FSflag = 1;
	ctrlParasPtr->BSflag = 1;
	ctrlParasPtr->dampingFlag = DampingNone;
	
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
}



