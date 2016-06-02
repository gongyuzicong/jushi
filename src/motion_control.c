#include "motion_control.h"
#include "cfg_gpio.h"
#include "timer_opts.h"
#include "pwm_opts.h"
#include "magn_sensor.h"

#define MAX_SPEED_LIMIT (100 - MAX_STEP_SPEED_INC)
#define MAX_STEP_SPEED_INC	1

#define X3X2X1_MAX_SPEED_LIMIT		0x07

#define MOTOR_SPEED_RESPON_TIME		(100)


ControlerParaStruct ctrlParas;
ControlerParaStruct_P ctrlParasPtr = &ctrlParas;
MotionOperaterStruct motionOpts;
MotionOperaterStruct_P motionOptsPtr = &motionOpts;
u32 responseTime = 0;
void (*agv_walking_func[5]) (void);

#define MOTOR_RIGHT_DUTY_SET(speed)			(pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, speed))
#define MOTOR_LEFT_DUTY_SET(speed)			(pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, speed))

#define MOTOR_RIGHT_DUTY_SET_Setted()			{pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, ctrlParasPtr->rightMotorSettedSpeed);}
#define MOTOR_LEFT_DUTY_SET_Setted()			{pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, ctrlParasPtr->leftMotorSettedSpeed);}


#define MOTOR_RIGHT_DUTY_OFFSET()		{pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, ctrlParasPtr->settedSpeed + ctrlParasPtr->rightMotorSpeedOffset);}
#define MOTOR_LEFT_DUTY_OFFSET()		{pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, ctrlParasPtr->settedSpeed + ctrlParasPtr->leftMotorSpeedOffset);}


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

/**********Motor Basic Control Unit: Begin****************/

void Motor_Right_Set(u8 speed)
{
	if((speed >= 0) && (speed <= 100))
	{
		MOTOR_RIGHT_DUTY_SET(speed);
	}
}

void Motor_Left_Set(u8 speed)
{
	if((speed >= 0) && (speed <= 100))
	{
		MOTOR_LEFT_DUTY_SET(speed);
	}
}

void Motor_Right_CR(u8 speed)		// 正转
{
	
	if((speed >= 0) && (speed <= 100))
	{
		pwmOptsPtr_1->Duty_Cycle_OC3_Set(pwmParaPtr_1, speed);
	}

	MOTOR_RIGHT_CR_PIN_SET();
}

void Motor_Right_CCR(u8 speed)	// 反转
{
	

	#if 1
	if((speed >= 0) && (speed <= 100))
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
	
	
	if((speed >= 0) && (speed <= 100))
	{
		pwmOptsPtr_1->Duty_Cycle_OC4_Set(pwmParaPtr_1, speed);
	}

	
	MOTOR_LEFT_CR_PIN_SET();
}

void Motor_Left_CCR(u8 speed)		// 反转
{
	
	
	if((speed >= 0) && (speed <= 100))
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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
		
		ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
		ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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
	
	ctrlParasPtr->leftMotorSpeed = ctrlParasPtr->leftMotorSettedSpeed;
	ctrlParasPtr->rightMotorSpeed = ctrlParasPtr->rightMotorSettedSpeed;

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


// 巨石这什么烂摊子, 可以去吃屎吗 ^_^
void walking_goStraight(void)
{
	#if 0
	
	MSDF_Opts_Ptr->magn_show(FMSDS_Ptr);
	MSDF_Opts_Ptr->magn_show(FMSDS_Pre_Ptr);
	
	// 一个礼拜写一个控制算法,客户你这么厉害你爸妈知道吗 ^_^ 客户你行你来啊
	if(AgvCent2Left == FMSDS_Ptr->agvDirection)		// 往外偏移,加速  客户你妈妈叫你回家吃饭你知道吗 ^_^
	{
		if(AgvCent2Left == FMSDS_Pre_Ptr->agvDirection)		// 情况没有改善,右侧电机减速
		{
			ctrlParasPtr->rightMotorSpeedOffset -= ((5 - FMSDS_Ptr->RightRemain) * 1);
			MOTOR_RIGHT_DUTY_OFFSET();
			printf("1rightMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->rightMotorSpeedOffset);
		}
		else
		{
			ctrlParasPtr->leftMotorSpeedOffset += ((5 - FMSDS_Ptr->RightRemain) * 1);
			MOTOR_LEFT_DUTY_OFFSET();
			printf("2leftMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->leftMotorSpeedOffset);
		}
	}
	else if(AgvCent2Right == FMSDS_Ptr->agvDirection)	// 往外偏移,加速
	{
		if(AgvCent2Right == FMSDS_Pre_Ptr->agvDirection)		// 情况没有改善,左侧电机减速
		{
			ctrlParasPtr->leftMotorSpeedOffset -= ((5 - FMSDS_Ptr->LeftRemain) * 1);
			MOTOR_LEFT_DUTY_OFFSET();
			printf("3leftMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->leftMotorSpeedOffset);
		}
		else
		{
			ctrlParasPtr->rightMotorSpeedOffset += ((5 - FMSDS_Ptr->LeftRemain) * 1);
			MOTOR_RIGHT_DUTY_OFFSET();
			printf("4rightMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->rightMotorSpeedOffset);
		}
	}
	else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)	// 往回偏移,减速
	{
		#if 0
		
		if((0 != ctrlParasPtr->rightMotorSpeedOffset) || (0 != ctrlParasPtr->leftMotorSpeedOffset))
		{
			if(0 != ctrlParasPtr->leftMotorSpeedOffset)
			{
				ctrlParasPtr->leftMotorSpeedOffset -= (ctrlParasPtr->leftMotorSpeedOffset / 2);
				MOTOR_LEFT_DUTY_OFFSET();
			}
			else
			{
				ctrlParasPtr->rightMotorSpeedOffset -= (ctrlParasPtr->rightMotorSpeedOffset / 2);
				MOTOR_RIGHT_DUTY_OFFSET();
			}
		}
		else
		{
			
		}
		
		#endif

		ctrlParasPtr->rightMotorSpeedOffset -= (ctrlParasPtr->rightMotorSpeedOffset / 2);
		MOTOR_RIGHT_DUTY_OFFSET();
		printf("5rightMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->rightMotorSpeedOffset);
	}
	else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 	// 往回偏移,减速
	{
		ctrlParasPtr->leftMotorSpeedOffset -= (ctrlParasPtr->leftMotorSpeedOffset / 2);
		MOTOR_LEFT_DUTY_OFFSET();
		printf("6leftMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->leftMotorSpeedOffset);
	}
	else if(AgvLeft2Right == FMSDS_Ptr->agvDirection)	// 左到右极速偏移,加减速同时作用
	{
		ctrlParasPtr->leftMotorSpeedOffset -= (ctrlParasPtr->leftMotorSpeedOffset / 2);
		MOTOR_LEFT_DUTY_OFFSET();
		printf("7leftMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->leftMotorSpeedOffset);
		ctrlParasPtr->rightMotorSpeedOffset += ((5 - FMSDS_Ptr->LeftRemain) * 1);
		MOTOR_RIGHT_DUTY_OFFSET();
		printf("8rightMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->rightMotorSpeedOffset);
	}
	else if(AgvRight2Left == FMSDS_Ptr->agvDirection)	// 右到左极速偏移,加减速同时作用
	{
		ctrlParasPtr->rightMotorSpeedOffset -= (ctrlParasPtr->rightMotorSpeedOffset / 2);
		MOTOR_RIGHT_DUTY_OFFSET();
		printf("9rightMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->rightMotorSpeedOffset);
		ctrlParasPtr->leftMotorSpeedOffset += ((5 - FMSDS_Ptr->RightRemain) * 1);
		MOTOR_LEFT_DUTY_OFFSET();
		printf("10leftMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->leftMotorSpeedOffset);
	}
	else if(AgvCenter == FMSDS_Ptr->agvDirection)		// 在中央位置
	{
		printf("11rightMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->rightMotorSpeedOffset);
		printf("12leftMotorSpeedOffset = %d\r\n\r\n", ctrlParasPtr->leftMotorSpeedOffset);
	}

	#else

	MSDF_Opts_Ptr->magn_show(FMSDS_Ptr);
	MSDF_Opts_Ptr->magn_show(FMSDS_Pre_Ptr);

	if(AgvCent2Left == FMSDS_Ptr->agvDirection)			// 往外偏移,加速
	{
		if(ctrlParasPtr->leftMotorSettedSpeed < 11)
		{
			ctrlParasPtr->leftMotorSettedSpeed = 11;
			ctrlParasPtr->rightMotorSettedSpeed = 9;
			MOTOR_RIGHT_DUTY_SET_Setted();
			MOTOR_LEFT_DUTY_SET_Setted();
		}
		
	}
	else if(AgvCent2Right == FMSDS_Ptr->agvDirection)	// 往外偏移,加速
	{
		if(ctrlParasPtr->rightMotorSettedSpeed < 11)
		{
			ctrlParasPtr->leftMotorSettedSpeed = 9;
			ctrlParasPtr->rightMotorSettedSpeed = 11;
			MOTOR_RIGHT_DUTY_SET_Setted();
			MOTOR_LEFT_DUTY_SET_Setted();
		}
	}
	else if(AgvRight2Cent == FMSDS_Ptr->agvDirection)	// 往回偏移,减速
	{
		
	}
	else if(AgvLeft2Cent == FMSDS_Ptr->agvDirection) 	// 往回偏移,减速
	{
		
	}
	else if(AgvLeft2Right == FMSDS_Ptr->agvDirection)	// 左到右极速偏移,加减速同时作用
	{
		
	}
	else if(AgvRight2Left == FMSDS_Ptr->agvDirection)	// 右到左极速偏移,加减速同时作用
	{
		
	}
	else if(AgvCenter == FMSDS_Ptr->agvDirection)		// 在中央位置
	{
		
	}
	
	#endif
}

void walking_backStatus(void)
{
	
}

void walking_cirLeft(void)
{
	
}

void walking_cirRight(void)
{
	
}

void walking_stopStatus(void)
{
	//motion_stop_pwm();
}


/**********Motor Basic Control Mode: End****************/

void AGV_Walking(void)
{
	//static u32 responseTime = 0;
	
	if(AutomaticMode == ctrlParasPtr->agvWalkingMode)
	{
		// 1. 扫描并且计算磁传感器相关数据
		MSDF_Opts_Ptr->MS_Scan();		
		

		//printf("responseTime = %d\r\n", responseTime);
		//if(RESPONSE_TIME_CALU(ctrlParasPtr->settedSpeed) == responseTime)

		// 2. 根据处理到的数据, 对电机做相应的调速(P Control)(此处要预留一定的响应时间)
		//printf("responseTime = %d\r\n", responseTime);
		//printf("SystemRunningTime = %d\r\n", SystemRunningTime);
		if(responseTime >= MOTOR_SPEED_RESPON_TIME)
		{
			responseTime = 0;
			agv_walking_func[ctrlParasPtr->agvStatus]();
		}

		
		// 3. 监测电机转速, 并且结合传感器数据, 对电机的PWM做相应的调整(D Control)
		
		
	}
	else if(ManualMode == ctrlParasPtr->agvWalkingMode)
	{
		
	}
}

void AGV_Walking_Test(void)
{
	MOTOR_POWER = 0;

	#if 1
	CHANGE_TO_GO_STRAIGHT_MODE();
	#else
	CHANGE_TO_BACK_MODE();
	#endif
	//MOTOR_LEFT_STOP_PIN_SET();
	ctrlParasPtr->settedSpeed = 10;
	ctrlParasPtr->leftMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	
	MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->settedSpeed);
	MOTOR_LEFT_DUTY_SET(ctrlParasPtr->settedSpeed);
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

void Motion_Ctrl_Init(void)
{
	
	Motion_Ctrl_GPIO_Init();
	
	CHANGE_TO_STOP_MODE();
	
	
	ctrlParasPtr->agvStatus = stopStatus;
	ctrlParasPtr->settedSpeed = 0;
	ctrlParasPtr->rightMotorSpeed = 0;
	ctrlParasPtr->rightMotorSettedSpeed = 0;
	ctrlParasPtr->leftMotorSpeed = 0;
	ctrlParasPtr->leftMotorSettedSpeed = 0;
	ctrlParasPtr->leftInc = 0;
	ctrlParasPtr->rightInc = 0;
	ctrlParasPtr->speedMode = PWM_MODE;
	ctrlParasPtr->speedModeValue_Right = 0x00;
	ctrlParasPtr->speedModeValue_Left = 0x00;
	ctrlParasPtr->agvWalkingMode = AutomaticMode;
	
	x3x2x1_left_setting();
	x3x2x1_right_setting();
	
	motionOptsPtr->motor_up = motion_up;
	motionOptsPtr->motor_down = motion_down;
	motionOptsPtr->motor_left = motion_left;
	motionOptsPtr->motor_right = motion_right;
	motionOptsPtr->motor_stop = motion_stop;
	motionOptsPtr->agv_walk = AGV_Walking;
	motionOptsPtr->agv_walk_test = AGV_Walking_Test;
	motionOptsPtr->agv_walk_stop = AGV_Walking_Stop;
	motionOptsPtr->agv_walk_test2 = AGV_Walking_Test2;

	agv_walking_func[stopStatus] = walking_stopStatus;
	agv_walking_func[goStraightStatus] = walking_goStraight;
	agv_walking_func[backStatus] = walking_backStatus;
	agv_walking_func[cirLeft] = walking_cirLeft;
	agv_walking_func[cirRight] = walking_cirRight;
}



