#include "motion_control.h"
#include "cfg_gpio.h"

ControlerParaStruct ctrlParas;
ControlerParaStruct_P ctrlParasPtr = &ctrlParas;
MotionOperaterStruct motionOpts;
MotionOperaterStruct_P motionOptsPtr = &motionOpts;


/**********Motor Basic Control Unit: Begin****************/
void Motor_Right_CR(ControlerParaStruct_P ctrlPtr)		// 正转
{
	MOTOR_RIGHT_BK = 1;
	MOTOR_RIGHT_EN = 0;
	
	pwm3_optsPtr->Duty_Cycle_OC1_Set(pwM3ParaPtr, ctrlPtr->settedSpeed);
}

void Motor_Right_CCR(ControlerParaStruct_P ctrlPtr)	// 反转
{
	MOTOR_RIGHT_BK = 1;
	MOTOR_RIGHT_FR = 0;
	MOTOR_RIGHT_EN = 0;

	pwm3_optsPtr->Duty_Cycle_OC1_Set(pwM3ParaPtr, ctrlPtr->settedSpeed);
}

void Motor_Left_CR(ControlerParaStruct_P ctrlPtr)		// 正转
{
	MOTOR_LEFT_BK = 1;
	MOTOR_LEFT_EN = 0;

	pwm3_optsPtr->Duty_Cycle_OC2_Set(pwM3ParaPtr, ctrlPtr->settedSpeed);
}

void Motor_Left_CCR(ControlerParaStruct_P ctrlPtr)		// 反转
{
	MOTOR_LEFT_BK = 1;
	MOTOR_LEFT_FR = 0;
	MOTOR_LEFT_EN = 0;

	pwm3_optsPtr->Duty_Cycle_OC2_Set(pwM3ParaPtr, ctrlPtr->settedSpeed);
}

/**********Motor Basic Control Unit: End****************/



/**********Motor Basic Control Mode: Begin****************/
void AGV_Brake(void)
{
	MOTOR_RIGHT_BK = 0;
	MOTOR_LEFT_BK = 0;
}

void AGV_Go_Straight_Start(void)
{
	
}

void AGV_Go_Straight_Stop(void)
{
	
}

void AGV_Go_Straight_Distance(void)
{
	
}

void AGV_Back_Start(void)
{
	
}

void AGV_Back_Stop(void)
{
	
}

void AGV_Back_Distance(void)
{
	
}

void AGV_Turn_Left_Start(void)
{
	
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

/**********Motor Basic Control Mode: End****************/



void Motion_Ctrl_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

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
}

void Motion_Ctrl_Init(void)
{
	Motion_Ctrl_GPIO_Init();
	
	MOTOR_RIGHT_EN = 1;
	MOTOR_RIGHT_FR = 1;
	MOTOR_RIGHT_BK = 0;
	
	MOTOR_LEFT_EN = 1;
	MOTOR_LEFT_FR = 1;
	MOTOR_LEFT_BK = 0;
}



