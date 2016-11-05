#include "cfg_gpio.h"
#include "timer_opts.h"
#include "pwm_opts.h"
#include "magn_sensor.h"
#include "zigbee.h"
//#include "mpu6050.h"
#include "i2c_opts.h"
#include "zigbee.h"
#include "buffer.h"
#include "rtc.h" 
#include "circle_recoder.h"
#include "ecv_control.h"

#include "motion_control.h"

#define ABSOLU(value)	(value >= 0 ? value : (-value))
#define MAX_ADAPT_NUM	20
#define MAX_DAMP_ADAPT_NUM	15

#define USE_HALL_GVEER 0

#define MAX_IN			21
#define MAX_OUT 		21
#define ShiftTrigTime	15000

#define CIR_HALL		165

#define USE_R_DEC_SPEED 0

#define decTim 5000

/*
HallCount CrossRoadHallCountArrGS[6];
HallCount CrossRoadHallCountArrGB[6];
*/

u16 ZBandRFIDmapping[11];

u8 	AgvGear[MAX_GEAR_NUM] = {0, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
u8 	AgvGearCompDutyLF[MAX_GEAR_NUM] = {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
u8 	AgvGearCompDutyRF[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 	AgvGearCompDutyLB[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 	AgvGearCompDutyRB[MAX_GEAR_NUM] = {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

#if USE_MPU6050
MPU6050_Para 		mpu6050DS;
MPU6050_Para_P 		mpu6050DS_ptr = &mpu6050DS;
#endif

Buzzer_Ctrl 	BuzzerCtrl;
Buzzer_Ctrl_P 	BuzzerCtrlPtr = &BuzzerCtrl;

ControlerParaStruct 	ctrlParas;
ControlerParaStruct_P 	ctrlParasPtr = &ctrlParas;
//MotionOperaterStruct motionOpts;
//MotionOperaterStruct_P motionOptsPtr = &motionOpts;


u32 responseTime = 0;

void (*agv_walking[StatusEnd]) (u8);
void (*walking_step[step_end])(void);

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


void walking_cir(u8 duty)
{
	static u8 lmSpeed = 0, rmSpeed = 0;

	//lmSpeed = duty + AgvGearCompDutyLF[2];
	//rmSpeed = duty + AgvGearCompDutyLF[2];	
	lmSpeed = ctrlParasPtr->cirDuty + AgvGearCompDutyLF[2];
	rmSpeed = ctrlParasPtr->cirDuty + AgvGearCompDutyLF[2];
	
	ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
	ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;
	
	MOTOR_LEFT_DUTY_SET(lmSpeed);
	MOTOR_RIGHT_DUTY_SET(rmSpeed);
		
}

void walking_cir_hall(u8 duty)
{
	static u8 lmSpeed = 0, rmSpeed = 0;

	if(ctrlParasPtr->rightHallCounter >= ctrlParasPtr->rightHallCounterCMP)
	{
		rmSpeed = 0;
	}
	else
	{
		rmSpeed = duty + AgvGearCompDutyLF[2];
	}

	if(ctrlParasPtr->leftHallCounter >= ctrlParasPtr->leftHallCounterCMP)
	{
		lmSpeed = 0;
	}
	else
	{
		lmSpeed = duty + AgvGearCompDutyLF[2];
	}

	if(CHECK_MOTOR_SET_DUTY(lmSpeed) && CHECK_MOTOR_SET_DUTY(rmSpeed))
	{
		ctrlParasPtr->leftMotorSettedSpeed = lmSpeed;
		ctrlParasPtr->rightMotorSettedSpeed = rmSpeed;

		MOTOR_LEFT_DUTY_SET(lmSpeed);
		MOTOR_RIGHT_DUTY_SET(rmSpeed);
	}
	
}


void walking_stopStatus(u8 gear)
{
	//motion_stop_pwm();
}

void NullFunc(u8 gear)
{
	
}

/**********Motor Basic Control Mode: End****************/


void gS_startup_mode6(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;

	#if USE_WECV
	static u8 flag = 0;
	#endif
	
	static u32 startCount = 0;
	//static Agv_MS_Location mslRec = AgvInits;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};
	//u8 gainDuty[15] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8};
	
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

		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
		//startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		rmSpeedP = 0;
		
		//startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
		
		lmSpeedP = 0;
		rmSpeedP = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}

	
	if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_0_5) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_0_5))
	{
		if(0 == startCount)
		{
			startCount = SystemRunningTime;
		}
		else
		{
			centCount = SystemRunningTime - startCount;
		}
		//printf("centCount = %d\r\n", centCount);
		if(centCount >= ShiftTrigTime)
		{

			#if USE_WECV
			/*
			Ecv_Para temp;
			temp.Dir	= ECV_UP;
			flag = 1;
			CHANGE_TO_STOP_MODE();
			WECV_Str_Ptr->ECV_SetPara(&temp);
			*/
			WECV_Str_Ptr->Dir = ECV_UP;
			flag = 1;
			CHANGE_TO_STOP_MODE();
			printf("********************************\r\n");
			
			#else

			ctrlParasPtr->FSflag = 1;
			ctrlParasPtr->gear = 7;
			ctrlParasPtr->comflag = 6331;

			#endif
			
			startCount = 0;
			
		}
	}
	else
	{
		startCount = 0;
	}

	#if USE_WECV

	
	if(1 == flag)
	{
		if(WECV_DOWN_LIMT_SW_RESP)
		{
			ctrlParasPtr->FSflag 	= 1;
			ctrlParasPtr->gear 		= 12;
			WECV_Str_Ptr->ECV_Clean_Use_Status();
			
			CHANGE_TO_GO_STRAIGHT_MODE();
			flag = 0;
		}
	}
	
	#endif
	
	
	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	set_duty(lmSpeed, rmSpeed);
	
}

void bS_startup_mode8(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	static u8 flag = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	//static Agv_MS_Location mslRec = AgvInits;
	//u8 gainDuty[11] = {1, 4, 6, 8, 10, 12, 12, 12, 12, 12};
	u8 gainDuty[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 12, 12, 12};
	//u8 gainDuty[15] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8};
	
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

		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
		//startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		rmSpeedP = 0;
		
		//startCount = 0;
	}
	else if(FMSDS_Ptr->AgvMSLocation == Agv_MS_Center)
	{
		ctrlParasPtr->comflag = 634;
		
		lmSpeedP = 0;
		rmSpeedP = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}

	#if 1
	
	if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
	{
		
		if((RMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (RMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
		{
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
			}
			
			//printf("centCount = %d\r\n", centCount);
			if(centCount >= ShiftTrigTime - 12000)
			{
				if(1 == ctrlParasPtr->Use_WECV)
				{
					Ecv_Para temp;
					temp.Dir 	= ECV_UP;
					WECV_Str_Ptr->ECV_SetPara(&temp);
					flag 		= 1;
					//CHANGE_TO_STOP_MODE();
				}
				else
				{
					ctrlParasPtr->BSflag = 1;
					ctrlParasPtr->gear = 15;
				}
				
				ctrlParasPtr->comflag = 6331;
				
				startCount = 0;
				
			}
		}
		else
		{
			startCount = 0;
		}
		
	}
	else
	{
		startCount = 0;
	}

	if(1 == ctrlParasPtr->Use_WECV)
	{
		if(1 == flag)
		{
			
			if(ECV_UP_LIMT == WECV_Str_Ptr->Location)
			{
				ctrlParasPtr->BSflag = 1;
				ctrlParasPtr->gear = 15;
				WECV_Str_Ptr->ECV_Clean_Use_Status();
				//CHANGE_TO_BACK_MODE();
				//printf("#########################\r\n");
				flag = 0;
			}
			
		}
		else
		{
			if(ECV_UP_LIMT == WECV_Str_Ptr->Location)
			{
				WECV_Str_Ptr->Dir = ECV_DOWN;
			}
			
		}
	}

	#else

	if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_5) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_5))
	{
		ctrlParasPtr->BSflag 	= 1;
		ctrlParasPtr->gear 		= 12;
	}
	
	#endif

	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP - 1;

	set_duty(rmSpeed, lmSpeed);
	
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

		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
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

	set_duty(lmSpeed, rmSpeed);
	
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

		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
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

	set_duty(rmSpeed, lmSpeed);
	
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


void Pattern_ctrl5(u8 *T1LSpeed, u8 *T1RSpeed)
{
	u8 lmSpeed = 0, rmSpeed = 0;
	static Agv_MS_Location msRec = AgvInits;
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


void so_This_is_P(u8 *lmSpeedPat_PP, u8 *rmSpeedPat_PP)
{	
	//u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16};
	//u8 AgvPatAngOut[MAX_OUT] = {1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10};
	//u8 AgvPatAngOut[21] = {1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
	//u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 3, 3, 5, 5, 7, 7, 9, 9, 11, 11, 13, 13, 15, 15, 15, 15};
	u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 3, 3, 5, 5, 11, 11, 13, 13, 15, 15, 18, 18, 20, 20, 20, 20};
	//u8 MidpointDuty[21] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4};
	u8 lmSpeedPat_P = 0, rmSpeedPat_P = 0, tempAngle = 0;
	u8 maxLimt = MAX_OUT - 1;

	// 1.判别车体的运动方向, 区分控制逻辑(正数:车体往远离磁条方向; 负数:车体靠近磁条方向)
	if(AGV_Pat_Ptr->MidpointDirection >= 0)		// 1.2 车体中点为远离磁条方向
	{
		// 2.判别车体在左边/右边,然后区分控制逻辑
		if(AGV_Pat_Ptr->Midpoint > 0)	// 2.1 车体中点在磁条右边, 远离磁条方向
		{

			if(AGV_Pat_Ptr->Midpoint <= 20)
			{
				
			}
			else if(AGV_Pat_Ptr->Midpoint > 20)
			{
				
			}
			
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

					//if((tempAngle > 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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
					
					//if((tempAngle > 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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
					
					//if((tempAngle >= 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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


	ctrlParasPtr->LP_duty = lmSpeedPat_P;
	ctrlParasPtr->RP_duty = rmSpeedPat_P;
	
	*lmSpeedPat_PP = lmSpeedPat_P;
	*rmSpeedPat_PP = rmSpeedPat_P;
}

void so_This_is_P2(u8 *lmSpeedPat_PP, u8 *rmSpeedPat_PP)
{	
	//u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16};
	//u8 AgvPatAngOut[MAX_OUT] = {1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10};
	//u8 AgvPatAngOut[21] = {1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
	u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 3, 3, 5, 5, 7, 7, 9, 9, 11, 11, 13, 13, 15, 15, 15, 15};
	u8 lmSpeedPat_P = 0, rmSpeedPat_P = 0, tempAngle = 0;
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

					//if((tempAngle > 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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
					
					//if((tempAngle > 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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
					
					//if((tempAngle >= 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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


	ctrlParasPtr->LP_duty = lmSpeedPat_P;
	ctrlParasPtr->RP_duty = rmSpeedPat_P;
	
	*lmSpeedPat_PP = lmSpeedPat_P;
	*rmSpeedPat_PP = rmSpeedPat_P;
}

void so_This_is_P3(u8 *lmSpeedPat_PP, u8 *rmSpeedPat_PP)
{	
	u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 3, 3, 5, 5, 7, 7, 9, 9, 11, 11, 13, 13, 15, 15, 15, 15};
	u8 lmSpeedPat_P = 0, rmSpeedPat_P = 0, tempAngle = 0;
	u8 maxLimt = MAX_OUT - 1;
	
	// 1.判别车体的运动方向, 区分控制逻辑(正数:车体往远离磁条方向; 负数:车体靠近磁条方向)
	if(AGV_Pat_Ptr->MidpointDirection >= 0)		// 1.2 车体中点为远离磁条方向
	{
		// 2.判别车体在左边/右边,然后区分控制逻辑
		if(AGV_Pat_Ptr->Midpoint > 0)	// 2.1 车体中点在磁条右边, 远离磁条方向
		{
			if(AGV_Pat_Ptr->Midpoint <= 20)
			{
				
			}
			else if(AGV_Pat_Ptr->Midpoint > 20)
			{
				
			}
			
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

					//if((tempAngle > 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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
					
					//if((tempAngle > 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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
					
					//if((tempAngle >= 0) && (tempAngle < MAX_OUT))
					if(tempAngle < MAX_OUT)
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


	ctrlParasPtr->LP_duty = lmSpeedPat_P;
	ctrlParasPtr->RP_duty = rmSpeedPat_P;
	
	*lmSpeedPat_PP = lmSpeedPat_P;
	*rmSpeedPat_PP = rmSpeedPat_P;
}


void so_This_is_D(u8 *lmSpeedPat_DP, u8 *rmSpeedPat_DP)
{
	u8 lmSpeedPat_D = 0, rmSpeedPat_D = 0;
	
	if(AGV_Pat_Ptr->MidpointDirection < 0)	// 1.2 车体中点方向是往磁条靠拢
	{
		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(AGV_Pat_Ptr->Angle < 0)		
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

void so_This_is_D4(u8 *lmSpeedPat_DP, u8 *rmSpeedPat_DP)
{
	u8 lmSpeedPat_D = 0, rmSpeedPat_D = 0, tempAngle = 0;
	//u8 AgvPatAngIn[MAX_IN] = {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16};
	//u8 AgvPatAngIn[MAX_IN] = {2, 3, 3, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 18, 18, 18, 18};
	u8 AgvPatAngIn[MAX_IN] = {1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 12, 12, 14, 14, 16, 16, 16, 16};
	u8 MidpointAngleLimit[21] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5};
	u8 limt = 0;

	if((AGV_Pat_Ptr->Midpoint >= -20) && (AGV_Pat_Ptr->Midpoint < 0))
	{
		limt = MidpointAngleLimit[-AGV_Pat_Ptr->Midpoint];
	}
	else if((AGV_Pat_Ptr->Midpoint >= 0) && (AGV_Pat_Ptr->Midpoint <= 20))
	{
		limt = MidpointAngleLimit[AGV_Pat_Ptr->Midpoint];
	}
	else
	{
		limt = MidpointAngleLimit[20];
	}
	
	if(AGV_Pat_Ptr->MidpointDirection < 0)	// 1.2 车体中点方向是往磁条靠拢
	{
		if(AGV_Pat_Ptr->Midpoint > 0)
		{
			if(AGV_Pat_Ptr->Angle < 0)
			{
				if((AGV_Pat_Ptr->Angle < -limt) && (AGV_Pat_Ptr->Angle > -MAX_IN))
				{
					tempAngle = -AGV_Pat_Ptr->Angle;
					
					if((tempAngle > limt) && (tempAngle < MAX_IN))
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
			else if(AGV_Pat_Ptr->Angle > 0) 	// 甩尾
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
				
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				
			}
			else if(AGV_Pat_Ptr->Angle < 0)
			{
				
				
			}
		}

		
		else if(AGV_Pat_Ptr->Midpoint < 0)
		{
			if(AGV_Pat_Ptr->Angle > 0)
			{
				if(AGV_Pat_Ptr->Angle < 20)
				{
					limt = MidpointAngleLimit[AGV_Pat_Ptr->Angle];
				}
				else
				{
					limt = MidpointAngleLimit[20];
				}
				
			}
			else if(0 == AGV_Pat_Ptr->Angle)
			{
				lmSpeedPat_D = 1;
			}
			else if(AGV_Pat_Ptr->Angle < 0) 	// 甩尾
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

void so_This_is_D5(u8 *lmSpeedPat_DP, u8 *rmSpeedPat_DP)
{
	u8 lmSpeedPat_D = 0, rmSpeedPat_D = 0, tempAngle = 0;
	//u8 AgvPatAngIn[MAX_IN] = {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16};
	//u8 AgvPatAngIn[MAX_IN] = {2, 3, 3, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 18, 18, 18, 18};
	u8 AgvPatAngIn[MAX_IN] = {1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 12, 12, 14, 14, 16, 16, 16, 16};
	u8 MidpointAngleLimit[21] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5};
	u8 limt1 = 0;

	if((AGV_Pat_Ptr->Midpoint >= -20) && (AGV_Pat_Ptr->Midpoint < 0))
	{
		limt1 = MidpointAngleLimit[-AGV_Pat_Ptr->Midpoint] * 2;
	}
	else if((AGV_Pat_Ptr->Midpoint >= 0) && (AGV_Pat_Ptr->Midpoint <= 20))
	{
		limt1 = MidpointAngleLimit[AGV_Pat_Ptr->Midpoint] * 2;
	}
	else
	{
		limt1 = MidpointAngleLimit[20] * 2;
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


void AcceCtrl_Func(AccCtrlParaStr_P ptr, u8 *basc_speed)
{
	if(1 == ptr->AccEnableFlag)
	{
		if(ACC_CTRL_MODE_TIME == ptr->AccMode)
		{
			
			
		}
		else if(ACC_CTRL_MODE_HALL == ptr->AccMode)
		{
			
			
		}

		if(ptr->AccSpeedRec >  ptr->AccMaxSpeed)
		{
			ptr->AccSpeedRec =  ptr->AccMaxSpeed;
		}
		
	}
	else
	{
		ptr->AccSpeedRec = 0;
		
		
	}

	*basc_speed = ptr->AccSpeedRec;
	
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


void scale_1_mode20(u8 gear)
{
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeedPat_P = 0, rmSpeedPat_P = 0, lmSpeedPat_D = 0, rmSpeedPat_D = 0;
	static Agv_MS_Location msfRec = AgvInits, msrRec = AgvInits;
		
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;

	so_This_is_P(&lmSpeedPat_P, &rmSpeedPat_P);
	
	so_This_is_D5(&lmSpeedPat_D, &rmSpeedPat_D);

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

void scale_1_mode20_back(u8 gear)
{
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeedPat_P = 0, rmSpeedPat_P = 0, lmSpeedPat_D = 0, rmSpeedPat_D = 0;
	static Agv_MS_Location msfRec = AgvInits, msrRec = AgvInits;
		
	gearRecod = gear;
	
	ctrlParasPtr->comflag = 64;

	so_This_is_P(&lmSpeedPat_P, &rmSpeedPat_P);
	
	so_This_is_D5(&lmSpeedPat_D, &rmSpeedPat_D);

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
		if(ECV_UP_LIMT == WECV_Str_Ptr->Location)
		{
			lmSpeedPat_P *= 2;
			rmSpeedPat_P *= 2;
		}
		
		lmSpeedSet = AgvGear[gearRecod] - lmSpeedPat_P - lmSpeedPat_D;
		
		rmSpeedSet = AgvGear[gearRecod] - rmSpeedPat_P - rmSpeedPat_D;
		
	}

	set_duty_Com(lmSpeedSet, rmSpeedSet);
}


void HighSpeed_Protect2(void)
{
	
	if((FMSDS_Ptr->AgvMSLocation <= Agv_MS_Left_7) || (FMSDS_Ptr->AgvMSLocation >= Agv_MS_Right_7))
	{
		if(goStraightStatus == ctrlParasPtr->agvStatus)
		{
			ctrlParasPtr->FSflag = 0;
			ctrlParasPtr->gear = 7;
		}
		else if(backStatus == ctrlParasPtr->agvStatus)
		{
			ctrlParasPtr->BSflag = 0;
			ctrlParasPtr->gear = 7;
			
		}
	}
	
}


void ManualModeFunc(ManualMode_Ctrl modeCtrl)
{
	
	switch(modeCtrl)
	{
		case Man_Stop:
			#if USE_ECV
			FECV_Str_Ptr->Dir = ECV_STOP;
			BECV_Str_Ptr->Dir = ECV_STOP;
			#else
			CHANGE_TO_STOP_MODE();
			CleanAllSpeed();
			#endif
			break;

		case Man_Forward:
			#if USE_ECV
			FECV_Str_Ptr->Dir 				= ECV_UP;
			FECV_Str_Ptr->EcvHallCountCmp 	= 100;
			FECV_Str_Ptr->EcvSpeed 			= 100;
			FECV_Str_Ptr->HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
			#else
			CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
			walking_cir(10);
			#endif
			break;

		case Man_Backward:
			#if USE_ECV
			FECV_Str_Ptr->Dir 				= ECV_DOWN;
			FECV_Str_Ptr->EcvHallCountCmp 	= 100;
			FECV_Str_Ptr->EcvSpeed 			= 100;
			FECV_Str_Ptr->HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
			#else
			CHANGE_TO_BACK_SLOW_MODE();
			walking_cir(10);
			#endif
			break;

		case Man_CirL:
			#if USE_ECV
			BECV_Str_Ptr->Dir 				= ECV_UP;
			BECV_Str_Ptr->EcvHallCountCmp 	= 100;
			BECV_Str_Ptr->EcvSpeed 			= 100;
			BECV_Str_Ptr->HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
			#else
			CHANGE_TO_CIR_LEFT_MODE();
			walking_cir(8);
			#endif
			break;

		case Man_CirR:
			#if USE_ECV
			BECV_Str_Ptr->Dir 				= ECV_DOWN;
			BECV_Str_Ptr->EcvHallCountCmp 	= 100;
			BECV_Str_Ptr->EcvSpeed 			= 100;
			BECV_Str_Ptr->HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
			#else
			CHANGE_TO_CIR_RIGHT_MODE();
			walking_cir(8);
			#endif
			break;

		default:
			break;
	}
}

void ManualModeEcvCtrlFunc(void)
{
	static u16 F_Hall = 0, R_Hall = 0;
	
	if(Return_SW_LF_Respond)
	{
		FECV_Str_Ptr->Dir 			= ECV_UP;
		FECV_Str_Ptr->EcvSpeed 		= 100;
		FECV_Str_Ptr->HallCountMode = ECV_USE_HALL_COUNT_MODE_DISABLE;
		
	}
	else if(Return_SW_LR_Respond)
	{
		FECV_Str_Ptr->Dir 			= ECV_DOWN;
		FECV_Str_Ptr->EcvSpeed 		= 100;
		FECV_Str_Ptr->HallCountMode = ECV_USE_HALL_COUNT_MODE_DISABLE;
		
	}
	else
	{
		FECV_Str_Ptr->Dir 			= ECV_STOP;
		
	}

	if(Return_SW_RF_Respond)
	{
		BECV_Str_Ptr->Dir 			= ECV_UP;
		BECV_Str_Ptr->EcvSpeed 		= 50;
		BECV_Str_Ptr->HallCountMode = ECV_USE_HALL_COUNT_MODE_DISABLE;
		
	}
	else if(Return_SW_RR_Respond)
	{
		BECV_Str_Ptr->Dir 			= ECV_DOWN;
		BECV_Str_Ptr->EcvSpeed 		= 50;
		BECV_Str_Ptr->HallCountMode = ECV_USE_HALL_COUNT_MODE_DISABLE;
		
	}
	else
	{
		BECV_Str_Ptr->Dir 			= ECV_STOP;
	}

	if(F_Hall != FECV_Str_Ptr->EcvHallCount)
	{
		printf("FECV_HALL = %d\r\n", FECV_Str_Ptr->EcvHallCount);
	}
		
	if(R_Hall != BECV_Str_Ptr->EcvHallCount)
	{
		printf("BECV_HALL = %d\r\n", BECV_Str_Ptr->EcvHallCount);
	}
	
	FECV_Str_Ptr->ECV_Ctrl_Function(FECV_Str_Ptr);
	BECV_Str_Ptr->ECV_Ctrl_Function(BECV_Str_Ptr);
	
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
			// 低速启动模式
			gS_startup_mode6(4);
		}
		else if(1 == ctrlParasPtr->FSflag)
		{
			
			
			scale_1_mode20(gearRecod);
			HighSpeed_Protect2();
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
			bS_startup_mode8(4);
		}
		else if(1 == ctrlParasPtr->BSflag)
		{
			
			//printf("gearRecod = %d\r\n", gearRecod);
			scale_1_mode20_back(gearRecod);
			HighSpeed_Protect2();
		}
		else if(3 == ctrlParasPtr->BSflag)
		{
			back_slow2(5);
			
		}
		
		ctrlParasPtr->FSflag = 0;
	}
	
}


void gS_step_exit(u8 gearRecod)
{
	static u8 lmSpeed = 0, rmSpeed = 0;
	u8 DutyTableLow[10] = {2, 4, 8, 10, 12, 16, 16, 16, 16, 16};
	//u32 centCount = 0;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Center))			// 往外偏移,加速
	{
		ctrlParasPtr->comflag = 64;

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

void AVG_Calu_Program1(void)
{
	static u32 HLavg = 0, HRavg = 0;
	
	HLavg = (ctrlParasPtr->HLavg + ctrlParasPtr->leftHallIntervalTime) / 2;
	if(HLavg > 100)
	{
		ctrlParasPtr->HLavg = HLavg;
	}
	else
	{
		//printf("HLavg = %d, TLnow = %d\r\n", ctrlParasPtr->HLavg, TLnow);
	}

	HRavg = (ctrlParasPtr->HRavg + ctrlParasPtr->rightHallIntervalTime) / 2;
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
	static u32 timRec = 0;

	duty = AgvGear[gear];

	// 延时一秒, 重新检测速度
	if(0 == timRec)
	{
		timRec = SystemRunningTime;
	}
	else
	{
		if(SystemRunningTime - timRec >= 10000)
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
			
			timRec = 0;
		}
	}
	
	
}

void AGV_Correct_2(void)
{
	u32 result = 0;
	u8 range = 0, lmSpeed = 0, rmSpeed = 0;
	static u8 duty = 0;
	static u8 lcompduty = 0, rcompduty = 0;
	static u8 gear = 10;
	static u32 timRec = 0;

	duty = AgvGear[gear];

	// 延时一秒, 重新检测速度

	if(Delay_Func(&timRec, 1000))
	{
		
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
		result = SubAbsV(ctrlParasPtr->leftHallIntervalTime, ctrlParasPtr->rightHallIntervalTime);
		
		printf("res = %d, leftHallIntervalTime = %d, rightHallIntervalTime = %d\r\n", result, ctrlParasPtr->leftHallIntervalTime, ctrlParasPtr->rightHallIntervalTime);

		// 目前设定的范围值为 ±5, 如果左右两边时间在这个范围内, 则认定两边速度是相等的
		if(result > range)			// 如果两个时间大于偏差范围
		{
			if(ctrlParasPtr->leftHallIntervalTime > ctrlParasPtr->rightHallIntervalTime)	// 如果是左边时间大于右边时间, 则左边转速比右边慢, 则需要左边加速/右边减速
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
		
	}
	
}


void Get_Zigbee_Info_From_Buf(void)
{
	//if((step_stop == ctrlParasPtr->walkingstep) && (zigbeeQueueCtrl.Total > 0) && (ctrlParasPtr->crossRoadCountF >= 3))
	if((ctrlParasPtr->walkingstep >= step_origin) && (zigbeeQueueCtrl.Total > 0))
	{
		ReqQueueStr info;
		
		info.Req_Station = 0;
		
		get_zigbeeData(&info);

		//printf("get_station = %d\r\n", info.Req_Station);

		if(0 == info.Req_Station)
		{
			printf("Req_Station error!\r\n");
		}
		else
		{
			Zigbee_Ptr->recvValidDataFlag = 1;
			//Zigbee_Ptr->recvId = data;
			Zigbee_Ptr->runningInfo = info;
			
			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->lock = 1;
			CircleInfoStrPtr->Station = info.Req_Station;
			CircleInfoStrPtr->RESPOND_TIME = BackgroudRTC_Rec;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			#endif

			printf("\r\n********************* get station ID: %d\t", info.Req_Station);
			printf("20%02x-%02x-%02x, %02x:%02x:%02x ***************************\r\n", BackgroudRTC_Rec.year, BackgroudRTC_Rec.month, BackgroudRTC_Rec.day, BackgroudRTC_Rec.hour, BackgroudRTC_Rec.minute, BackgroudRTC_Rec.second);
		}
		
		//zigbeeRecvDataBuf_Delete();
	}
}

void RFID_Goal_Node_Analy(void)
{
	if(1 == Zigbee_Ptr->recvValidDataFlag)
	{
		Zigbee_Ptr->recvValidDataFlag = 0;
		ctrlParasPtr->rifdAdaptFlag = 0;
		if((ZBandRFIDmapping[SpinStation_1] == Zigbee_Ptr->runningInfo.Req_Station) || (ZBandRFIDmapping[SpinStation_2] == Zigbee_Ptr->runningInfo.Req_Station))
		{
			ctrlParasPtr->goalRFIDnode = STATION_1AND2_RFID;
			//printf("%04x, %04x\r\n", ZBandRFIDmapping[SpinStation_1], Zigbee_Ptr->runningInfo.Req_Station);
			if(ZBandRFIDmapping[SpinStation_1] == Zigbee_Ptr->runningInfo.Req_Station)
			{
				ctrlParasPtr->goalStation = SpinStation_1;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_2;
			}
			
			ctrlParasPtr->walkingstep = step_gS;
		}
		else if((ZBandRFIDmapping[SpinStation_3] == Zigbee_Ptr->runningInfo.Req_Station) || (ZBandRFIDmapping[SpinStation_4] == Zigbee_Ptr->runningInfo.Req_Station))
		{
			ctrlParasPtr->goalRFIDnode = STATION_3AND4_RFID;
			if(ZBandRFIDmapping[SpinStation_3] == Zigbee_Ptr->runningInfo.Req_Station)
			{
				ctrlParasPtr->goalStation = SpinStation_3;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_4;
			}
			ctrlParasPtr->walkingstep = step_gS;
		}
		else if((ZBandRFIDmapping[SpinStation_5] == Zigbee_Ptr->runningInfo.Req_Station) || (ZBandRFIDmapping[SpinStation_6] == Zigbee_Ptr->runningInfo.Req_Station))
		{
			ctrlParasPtr->goalRFIDnode = STATION_5AND6_RFID;
			if(ZBandRFIDmapping[SpinStation_5] == Zigbee_Ptr->runningInfo.Req_Station)
			{
				ctrlParasPtr->goalStation = SpinStation_5;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_6;
			}
			ctrlParasPtr->walkingstep = step_gS;
		}
		else if((ZBandRFIDmapping[SpinStation_7] == Zigbee_Ptr->runningInfo.Req_Station) || (ZBandRFIDmapping[SpinStation_8] == Zigbee_Ptr->runningInfo.Req_Station))
		{
			ctrlParasPtr->goalRFIDnode = STATION_7AND8_RFID;
			if(ZBandRFIDmapping[SpinStation_7] == Zigbee_Ptr->runningInfo.Req_Station)
			{
				ctrlParasPtr->goalStation = SpinStation_7;
			}
			else
			{
				ctrlParasPtr->goalStation = SpinStation_8;
			}
			ctrlParasPtr->walkingstep = step_gS;
		}
		else if((ZBandRFIDmapping[SpinStation_9] == Zigbee_Ptr->runningInfo.Req_Station) || (ZBandRFIDmapping[SpinStation_10] == Zigbee_Ptr->runningInfo.Req_Station))
		{
			ctrlParasPtr->goalRFIDnode = STATION_9AND10_RFID;
			if(ZBandRFIDmapping[SpinStation_9] == Zigbee_Ptr->runningInfo.Req_Station)
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


void AGV_Walking_Opt(void)
{
	
	if(AutomaticMode == ctrlParasPtr->agvWalkingMode)
	{
		if((ctrlParasPtr->agvStatus > StatusStart) && (ctrlParasPtr->agvStatus < StatusEnd))
		{
			agv_walking[ctrlParasPtr->agvStatus](ctrlParasPtr->gear);
		}
		
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
	
	ctrlParasPtr->settedSpeed 			= AgvGear[15];
	ctrlParasPtr->leftMotorSettedSpeed 	= ctrlParasPtr->settedSpeed + 2;
	ctrlParasPtr->rightMotorSettedSpeed = ctrlParasPtr->settedSpeed;
	
	MOTOR_RIGHT_DUTY_SET(ctrlParasPtr->rightMotorSettedSpeed);
	MOTOR_LEFT_DUTY_SET(ctrlParasPtr->leftMotorSettedSpeed);
	#endif
	//CHANGE_TO_BACK_MODE();
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

#if USE_MPU6050
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



#endif


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


u8 Origin_PatCtrl(u8 duty)
{
	u8 status = 0;
	static u32 timRec = 0;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1))
	{
		CHANGE_TO_CIR_RIGHT_MODE();
		timRec = SystemRunningTime;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		CHANGE_TO_CIR_LEFT_MODE();
		//ctrlParasPtr->cirDuty = 7;
		timRec = SystemRunningTime;
	}
	else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
	{
		CHANGE_TO_STOP_MODE();
		CleanAllSpeed();
		
		if(SystemRunningTime - timRec >= 10000)
		{
			status = 1;
		}
		
	}

	return status;
}


u8 Origin_PatCtrl2(u8 duty)
{
	u8 status = 0;
	
	if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Left_1))
	{
		CHANGE_TO_CIR_RIGHT_MODE();
		//timRec = SystemRunningTime;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Right_1) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		CHANGE_TO_CIR_LEFT_MODE();
		//ctrlParasPtr->cirDuty = 7;
		//timRec = SystemRunningTime;
	}
	else if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
	{
		CHANGE_TO_STOP_MODE();
		CleanAllSpeed();
		
		status = 1;
		
	}

	return status;
}


void CrossRoad_Hall_Count_Start(void)
{
	
	ctrlParasPtr->CrossRoadHallCountFlag = 1;
	
}

void CrossRoad_Hall_Count_Stop(void)
{
	ctrlParasPtr->CrossRoadHallCountFlag = 0;
	//printf("CrossRoadHallCountL = %d, CrossRoadHallCountR = %d\r\n", ctrlParasPtr->CrossRoadHallCountL, ctrlParasPtr->CrossRoadHallCountR);
	ctrlParasPtr->CrossRoadHallCountL = 0;
	ctrlParasPtr->CrossRoadHallCountR = 0;
}


void CrossRoad_Count(void)
{
	static Agv_MS_Location mslRecF = AgvInits, mslRecR = AgvInits;

	if(0 == ctrlParasPtr->rifdAdaptFlag)
	{
		if(((goStraightStatus == ctrlParasPtr->agvStatus) || (gSslow == ctrlParasPtr->agvStatus)) && (step_entry != ctrlParasPtr->walkingstep) && (step_exit != ctrlParasPtr->walkingstep))
		{
			if(mslRecF != FMSDS_Ptr->AgvMSLocation)
			{
				mslRecF = FMSDS_Ptr->AgvMSLocation;
				
				if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountF < STATION_9AND10_RFID + EXTRA_CROSS_ROAD_R)
					{
						ctrlParasPtr->rightHallCounter = 0;
						ctrlParasPtr->leftHallCounter = 0;
						ctrlParasPtr->crossRoadCountF++;
						ctrlParasPtr->crossRoadUpdateF = 1;
						//printf("1Hclean\r\n");
						//printf("MAIN GcrossRoadCountF = %d, GcrossRoadCountR = %d\r\n", ctrlParasPtr->crossRoadCountF, ctrlParasPtr->crossRoadCountR);
					}
					else
					{
						//printf("crossRoadCountF error\r\n");
						//ctrlParasPtr->crossRoadCountF = 5;
					}
				}
				
				
			}

			if(mslRecR != RMSDS_Ptr->AgvMSLocation)
			{
				mslRecR = RMSDS_Ptr->AgvMSLocation;
				
				if(Agv_MS_CrossRoad == RMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountR < STATION_9AND10_RFID + EXTRA_CROSS_ROAD_R - 1)
					{
						ctrlParasPtr->crossRoadCountR++;
						ctrlParasPtr->crossRoadUpdateR = 1;
						//printf("1Hclean\r\n");
						//printf("MAIN GcrossRoadCountR = %d, GcrossRoadCountF = %d\r\n", ctrlParasPtr->crossRoadCountR, ctrlParasPtr->crossRoadCountF);
					}
					else
					{
						//printf("crossRoadCountR error\r\n");
					}
				}	
			}
			
		}
		else if(((backStatus == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus)) && (step_entry != ctrlParasPtr->walkingstep) && (step_exit != ctrlParasPtr->walkingstep))
		{
			if(mslRecF != FMSDS_Ptr->AgvMSLocation)
			{
				mslRecF = FMSDS_Ptr->AgvMSLocation;

				if(Agv_MS_CrossRoad == FMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountR > 0)
					{
						ctrlParasPtr->rightHallCounter = 0;
						ctrlParasPtr->leftHallCounter = 0;
						ctrlParasPtr->crossRoadCountR--;
						ctrlParasPtr->crossRoadUpdateF = 1;
						//printf("2Hclean\r\n");
						//printf("MAIN BcrossRoadCountR = %d, BcrossRoadCountF = %d\r\n", ctrlParasPtr->crossRoadCountR, ctrlParasPtr->crossRoadCountF);
					}
					else
					{
						//printf("crossRoadCountF error\r\n");
					}
				}
				
			}

			if(mslRecR != RMSDS_Ptr->AgvMSLocation)
			{
				mslRecR = RMSDS_Ptr->AgvMSLocation;

				if(Agv_MS_CrossRoad == RMSDS_Ptr->AgvMSLocation)
				{
					if(ctrlParasPtr->crossRoadCountF > 0)
					{
						ctrlParasPtr->crossRoadCountF--;
						ctrlParasPtr->crossRoadUpdateR = 1;
						//printf("2Hclean\r\n");
						//printf("MAIN BcrossRoadCountF = %d, BcrossRoadCountR = %d\r\n", ctrlParasPtr->crossRoadCountF, ctrlParasPtr->crossRoadCountR);
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

void step_gS_Func(void)
{
	static u8 flag = 0, step = 0;
	
	#if USE_R_DEC_SPEED
	static u32 timRec = 0;
	#endif
	
	//printf("rfidData = %02x, goalRFIDnode = %d\r\n", RFID_Info_Ptr->rfidData, ctrlParasPtr->goalRFIDnode);
	if((1 == ctrlParasPtr->originFlag) && (ORIGIN_STATION_NODE == ctrlParasPtr->goalRFIDnode))
	{
		if(0 == step)
		{
			CircleInfoStrPtr->CircleTime.Go2RFIDTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			step = 1;
		}
		else if(1 == step)
		{
			#if 0
			
			if(TypeManuReq == Zigbee_Ptr->runningInfo.Req_Type)
			{
				ctrlParasPtr->originFlag = 0;
				ctrlParasPtr->walkingstep = step_gVeer;
				step = 0;
				#if USE_CIRCLE_INFO_RECODER
				CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
				#endif
				
			}
			else if(TypeAutoReq == Zigbee_Ptr->runningInfo.Req_Type)
			{
				
				if(Return_SW_Respond)
				{
					ctrlParasPtr->originFlag = 0;
					ctrlParasPtr->walkingstep = step_gVeer;
					step = 0;
					#if USE_CIRCLE_INFO_RECODER
					CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
					#endif
				}
			}
			else
			{
				printf("req type error!\r\n");
			}
			
			#else

			if((TypeManuReq == Zigbee_Ptr->runningInfo.Req_Type) ||\
			   ((TypeAutoReq == Zigbee_Ptr->runningInfo.Req_Type) && (Return_SW_Respond)))
			{
				ctrlParasPtr->originFlag = 0;
				ctrlParasPtr->walkingstep = step_gVeer;
				ctrlParasPtr->AutoCancel_Respond = 0;
				step = 0;
				#if USE_CIRCLE_INFO_RECODER
				CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
				#endif
				
			}
			
			#endif
		}		
		
	}
	else
	{
		if(0 == ctrlParasPtr->rifdAdaptFlag)
		{
			if(GoodReqCancel == CMD_Flag_Ptr->Cancel_Flag)
			{			
				if(ctrlParasPtr->goalRFIDnode > RFID_Info_Ptr->rfidData)		// 目标在现在之前
				{
					if(ctrlParasPtr->crossRoadCountF >= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R)
					{
						
						CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
						ctrlParasPtr->gear = 3;
						ctrlParasPtr->rifdAdaptFlag = 1;
						CrossRoad_Hall_Count_Start();
						//printf("ctrlParasPtr->gear = 3\r\n");
					}				
					else
					{
						
						CHANGE_TO_GO_STRAIGHT_MODE();
						ctrlParasPtr->gear = 17;
					}
				}
				else if(ctrlParasPtr->goalRFIDnode <= RFID_Info_Ptr->rfidData)	// 目标在现在之后
				{
					if(ctrlParasPtr->crossRoadCountR <= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R - 1)
					{
						CHANGE_TO_BACK_SLOW_MODE();
						ctrlParasPtr->gear = 3;
						ctrlParasPtr->rifdAdaptFlag = 1;
						CrossRoad_Hall_Count_Start();
						//printf("ctrlParasPtr->gear = 3\r\n");
					}
					else
					{
						CHANGE_TO_BACK_MODE();
						ctrlParasPtr->gear = 15;
					}
				}
				
			}
			else
			{
				if(ctrlParasPtr->goalRFIDnode > ORIGIN_STATION_NODE)
				{
					
					#if USE_R_DEC_SPEED
					if(ctrlParasPtr->crossRoadCountR >= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R - 1)
					{
						if(Delay_Func(&timRec, decTim))
						{
							CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
							ctrlParasPtr->gear = 3;
						}
						
					}
					#else
					if(ctrlParasPtr->crossRoadCountF >= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R)
					{
						
						CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
						ctrlParasPtr->gear = 3;
						ctrlParasPtr->rifdAdaptFlag = 1;
						CrossRoad_Hall_Count_Start();
						//printf("ctrlParasPtr->gear = 3\r\n");
					}
					#endif
					else
					{
						CHANGE_TO_GO_STRAIGHT_MODE();
						ctrlParasPtr->gear = 17;
					}
				}
				else if(ctrlParasPtr->goalRFIDnode < ORIGIN_STATION_NODE)
				{
					#if USE_R_DEC_SPEED
					if(ctrlParasPtr->crossRoadCountF <= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R)
					{
						if(Delay_Func(&timRec, decTim))
						{
							CHANGE_TO_BACK_SLOW_MODE();
							ctrlParasPtr->gear = 3;
						}
						
					}
					#else
					if(ctrlParasPtr->crossRoadCountR <= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R - 1)
					{
						
						CHANGE_TO_BACK_SLOW_MODE();
						ctrlParasPtr->gear = 3;
						ctrlParasPtr->rifdAdaptFlag = 1;
						CrossRoad_Hall_Count_Start();
						//printf("ctrlParasPtr->gear = 3\r\n");
					}
					#endif
					
					else
					{
						
						CHANGE_TO_BACK_MODE();
						ctrlParasPtr->gear = 15;
					}
				}
			}		
			
			
		}
		else if(1 == ctrlParasPtr->rifdAdaptFlag)
		{
			
			if((goStraightStatus == ctrlParasPtr->agvStatus) || (gSslow == ctrlParasPtr->agvStatus))
			{
				if(0x0000 == RMSDS_Ptr->MSD_Hex)
				{
					CHANGE_TO_BACK_SLOW_MODE();
					ctrlParasPtr->gear = 3;
				}
			}
			else if((backStatus == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
			{
				if(0x0000 == RMSDS_Ptr->MSD_Hex)
				{
					CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
					ctrlParasPtr->gear = 3;
				}
			}
		}
		
		
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			
			RFID_Info_Ptr->updateFlag = 0;
			//printf("rfidData = %08x, goalRFIDnode = %d\r\n", RFID_Info_Ptr->rfidData, ctrlParasPtr->goalRFIDnode);
			//printf("1LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
			if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
			{
				CrossRoad_Hall_Count_Stop();
				CHANGE_TO_STOP_MODE();
				//Delay_ms(500);
				flag = 1;

				#if USE_CIRCLE_INFO_RECODER
				CircleInfoStrPtr->CircleTime.Go2RFIDTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
				#endif
			}
			else
			{
				ctrlParasPtr->rifdAdaptFlag = 0;
			}
		}

		if(1 == flag)
		{
			if(FLMT_SW_UNRESPOND)		// 检测到托盘的极限开关没有响应
			{
				ctrlParasPtr->Machine_ARM_Toggle_Flag = 0;
				MACHINE_ARM_TOGGLE_SET();
				flag = 2;
			}
			else
			{
				flag = 3;
			}
			
		}
		else if(2 == flag)
		{
			if(1 == ctrlParasPtr->Machine_ARM_Toggle_Flag)
			{
				
				if(FLMT_SW_UNRESPOND)
				{
					// 异常未解除, 报警
					
					WARNING_STATUS_FLMT_SW_ERR_SET();
				}
				else
				{
					// 异常排除, 解除异常, 恢复正常模式
					WARNING_STATUS_NORMAL_SET();
					
					ctrlParasPtr->Machine_ARM_Toggle_Flag = 0;
					flag = 3;
				}
				
			}
		}
		else if(3 == flag)
		{
			#if 0
			
			if(TypeManuReq == Zigbee_Ptr->runningInfo.Req_Type)
			{
				flag = 0;
				Zigbee_Ptr->runningInfo.Req_Type = TypeUnknow;
				CMD_Flag_Ptr->Cancel_Flag = NcNone;
				ctrlParasPtr->rifdAdaptFlag = 0;
				ctrlParasPtr->originFlag = 0;
				ctrlParasPtr->walkingstep	= step_gVeer;

				#if USE_CIRCLE_INFO_RECODER
				CircleInfoStrPtr->TimeTempRec = SystemRunningTime;	
				#endif
				
			}
			else if(TypeAutoReq == Zigbee_Ptr->runningInfo.Req_Type)
			{
				
				if(Return_SW_Respond)
				{
					flag = 0;
					Zigbee_Ptr->runningInfo.Req_Type = TypeUnknow;
					CMD_Flag_Ptr->Cancel_Flag = NcNone;
					ctrlParasPtr->rifdAdaptFlag = 0;
					ctrlParasPtr->originFlag = 0;
					ctrlParasPtr->walkingstep = step_gVeer;
					printf("AutoReq step_gVeer\r\n");
					
					#if USE_CIRCLE_INFO_RECODER
					CircleInfoStrPtr->TimeTempRec = SystemRunningTime;	
					#endif
					
				}
			}

			#else

			if((TypeManuReq == Zigbee_Ptr->runningInfo.Req_Type) ||\
			  ((TypeAutoReq == Zigbee_Ptr->runningInfo.Req_Type) && (Return_SW_Respond)))
			{
				flag = 0;
				Zigbee_Ptr->runningInfo.Req_Type = TypeUnknow;
				CMD_Flag_Ptr->Cancel_Flag = NcNone;
				ctrlParasPtr->rifdAdaptFlag = 0;
				ctrlParasPtr->originFlag = 0;
				ctrlParasPtr->walkingstep = step_gVeer;
				ctrlParasPtr->AutoCancel_Respond = 0;
				#if USE_CIRCLE_INFO_RECODER
				CircleInfoStrPtr->TimeTempRec = SystemRunningTime;	
				#endif
				
			}
			
			#endif
			
		}
		
		#if USE_HALL_GVEER
		else if((ctrlParasPtr->CrossRoadHallCountL >= 170) || (ctrlParasPtr->CrossRoadHallCountR >= 170))
		{
			CrossRoad_Hall_Count_Stop();
			CHANGE_TO_STOP_MODE();
			//Delay_ms(500);

			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->CircleTime.Go2RFIDTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			#endif
			
			ctrlParasPtr->walkingstep = step_gVeer;
			ctrlParasPtr->AutoCancel_Respond = 0;
		}
		#endif	
	}

}

void step_gS_Func2(void)
{
	#if USE_R_DEC_SPEED
	static u32 timRec = 0;
	#endif
	
	if(0 == ctrlParasPtr->rifdAdaptFlag)
	{
		if(1 == ctrlParasPtr->originFlag)
		{
			if(ORIGIN_STATION_NODE == ctrlParasPtr->goalRFIDnode)
			{
				ctrlParasPtr->originFlag = 0;
				ctrlParasPtr->walkingstep = step_gVeer;
			}
			else
			{
				if(ctrlParasPtr->goalRFIDnode > ORIGIN_STATION_NODE)
				{
					
					#if USE_R_DEC_SPEED
					if(ctrlParasPtr->crossRoadCountR >= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R - 1)
					{
						if(Delay_Func(&timRec, decTim))
						{
							CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
							ctrlParasPtr->gear = 3;
						}
						
					}
					#else
					if(ctrlParasPtr->crossRoadCountF >= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R)
					{
						
						CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
						ctrlParasPtr->gear = 3;
						ctrlParasPtr->rifdAdaptFlag = 1;
						CrossRoad_Hall_Count_Start();
						//printf("ctrlParasPtr->gear = 3\r\n");
					}
					#endif
					
					else
					{
						CHANGE_TO_GO_STRAIGHT_MODE();
						ctrlParasPtr->gear = 17;
					}
				}
				else if(ctrlParasPtr->goalRFIDnode < ORIGIN_STATION_NODE)
				{
					#if USE_R_DEC_SPEED
					if(ctrlParasPtr->crossRoadCountF <= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R)
					{
						if(Delay_Func(&timRec, decTim))
						{
							CHANGE_TO_BACK_SLOW_MODE();
							ctrlParasPtr->gear = 3;
						}
						
					}
					#else
					if(ctrlParasPtr->crossRoadCountR <= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R - 1)
					{
						
						CHANGE_TO_BACK_SLOW_MODE();
						ctrlParasPtr->gear = 3;
						ctrlParasPtr->rifdAdaptFlag = 1;
						CrossRoad_Hall_Count_Start();
						//printf("ctrlParasPtr->gear = 3\r\n");
					}
					#endif
					
					else
					{
						
						CHANGE_TO_BACK_MODE();
						ctrlParasPtr->gear = 15;
					}
				}
			}
		}
		else
		{			
			if(ctrlParasPtr->goalRFIDnode > RFID_Info_Ptr->rfidData)		// 目标在现在之前
			{
				if(ctrlParasPtr->crossRoadCountF >= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R)
				{
					
					CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
					ctrlParasPtr->gear = 3;
					ctrlParasPtr->rifdAdaptFlag = 1;
					CrossRoad_Hall_Count_Start();
					//printf("ctrlParasPtr->gear = 3\r\n");
				}				
				else
				{
					
					CHANGE_TO_GO_STRAIGHT_MODE();
					ctrlParasPtr->gear = 17;
				}
			}
			else if(ctrlParasPtr->goalRFIDnode <= RFID_Info_Ptr->rfidData)	// 目标在现在之后
			{
				if(ctrlParasPtr->crossRoadCountR <= ctrlParasPtr->goalRFIDnode + EXTRA_CROSS_ROAD_R - 1)
				{
					
					CHANGE_TO_BACK_SLOW_MODE();
					ctrlParasPtr->gear = 3;
					ctrlParasPtr->rifdAdaptFlag = 1;
					CrossRoad_Hall_Count_Start();
					//printf("ctrlParasPtr->gear = 3\r\n");
				}
				else
				{
					
					CHANGE_TO_BACK_MODE();
					ctrlParasPtr->gear = 15;
				}
			}
			
		}
		
		
	}
	else if(1 == ctrlParasPtr->rifdAdaptFlag)
	{
		
		if((goStraightStatus == ctrlParasPtr->agvStatus) || (gSslow == ctrlParasPtr->agvStatus))
		{
			if(0x0000 == RMSDS_Ptr->MSD_Hex)
			{
				CHANGE_TO_BACK_SLOW_MODE();
				ctrlParasPtr->gear = 3;
			}
		}
		else if((backStatus == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
		{
			if(0x0000 == RMSDS_Ptr->MSD_Hex)
			{
				CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
				ctrlParasPtr->gear = 3;
			}
		}
	}
	
	
	if(1 == RFID_Info_Ptr->updateFlag)
	{
		RFID_Info_Ptr->updateFlag = 0;
		printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
		//printf("1LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
		if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
		{
			CrossRoad_Hall_Count_Stop();
			CHANGE_TO_STOP_MODE();
			//Delay_ms(500);
			
			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->CircleTime.Go2RFIDTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			#endif
			ctrlParasPtr->rifdAdaptFlag = 0;
			ctrlParasPtr->walkingstep	= step_gVeer;
		}
	}
	
	#if USE_HALL_GVEER
	else if((ctrlParasPtr->CrossRoadHallCountL >= 170) || (ctrlParasPtr->CrossRoadHallCountR >= 170))
	{
		CrossRoad_Hall_Count_Stop();
		CHANGE_TO_STOP_MODE();
		//Delay_ms(500);

		#if USE_CIRCLE_INFO_RECODER
		CircleInfoStrPtr->CircleTime.Go2RFIDTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
		CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
		#endif
		
		ctrlParasPtr->walkingstep = step_gVeer;
	}
	#endif	

}


void step_gVeer_Func(void)
{
	static u8 stepFlag = 0;

	//ctrlParasPtr->rifdAdaptFlag = 0;
	
	if((SpinStation_1 	== ctrlParasPtr->goalStation) || \
		(SpinStation_3 	== ctrlParasPtr->goalStation) || \
		(SpinStation_5 	== ctrlParasPtr->goalStation) || \
		(SpinStation_7 	== ctrlParasPtr->goalStation) || \
		(SpinStation_9 	== ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_LEFT_MODE();
		//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
	}
	else if((SpinStation_2 	== ctrlParasPtr->goalStation) || \
			(SpinStation_4 	== ctrlParasPtr->goalStation) || \
			(SpinStation_6 	== ctrlParasPtr->goalStation) || \
			(SpinStation_8 	== ctrlParasPtr->goalStation) || \
			(SpinStation_10 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_RIGHT_MODE();
		//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
	}

	if(0 == stepFlag)
	{
		ctrlParasPtr->cirDuty = 13;
		stepFlag = 1;
	}
	else if(1 == stepFlag)
	{
		if((0xFFFF == FMSDS_Ptr->MSD_Hex) && (0xFFFF == RMSDS_Ptr->MSD_Hex))
		{
			ctrlParasPtr->cirDuty = 13;
			stepFlag = 2;
		}
	}
	else if(2 == stepFlag)
	{	
		if(0xFFFF != FMSDS_Ptr->MSD_Hex)
		{
			ctrlParasPtr->cirDuty = 9;
			
			if(1 == Origin_PatCtrl2(ctrlParasPtr->cirDuty))
			{
				CleanAllSpeed();
					
				CHANGE_TO_STOP_MODE();
				
				
				//if(TypeAutoReq == Zigbee_Ptr->runningInfo.Req_Type)
				if(0)
				{
					
					if(Return_SW_Respond)
					{
						stepFlag = 0;
						#if USE_CIRCLE_INFO_RECODER
						CircleInfoStrPtr->TimeTempRec 			= SystemRunningTime;
						#endif
						Zigbee_Ptr->runningInfo.Req_Type = TypeUnknow;
						printf("AutoReq step_entry\r\n");
						ctrlParasPtr->walkingstep = step_entry;
					}
					
				}
				//else if(TypeManuReq == Zigbee_Ptr->runningInfo.Req_Type)
				else
				{
					stepFlag = 0;
					Zigbee_Ptr->runningInfo.Req_Type = TypeUnknow;
					ctrlParasPtr->walkingstep = step_entry;
					
					#if USE_CIRCLE_INFO_RECODER
					CircleInfoStrPtr->CircleTime.GoCirTime 	= (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
					CircleInfoStrPtr->TimeTempRec 			= SystemRunningTime;
					#endif
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
		ctrlParasPtr->gear = 3;
	}
	else
	{
		ctrlParasPtr->gear = 2;
	}
	
	if((0 == LMT_INR) || (0 == LMT_INL))		// 前传感器感应到络纱机	
	//if((0 == LMT_INR) || (0 == LMT_INL) || (0x0000 == FMSDS_Ptr->MSD_Hex))	// 接近开关响应或者是超过磁条了
	//if(0x0000 == FMSDS_Ptr->MSD_Hex)
	{
		CHANGE_TO_STOP_MODE();
		//Delay_ns(1);
		//printf("go to step_catch*****************************\r\n");
		RFID_Info_Ptr->rfidData = 0;
		flag = 0;

		#if USE_CIRCLE_INFO_RECODER
		CircleInfoStrPtr->CircleTime.EntryTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
		CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
		#endif
		
		FECV_Str_Ptr->ECV_Clean_Use_Status();
		BECV_Str_Ptr->ECV_Clean_Use_Status();
		ctrlParasPtr->walkingstep = step_catch;
		
	}

#endif	
}

void step_catch_Func(void)
{
	
	if(0 == LMT_SW) 	// 已经抓到货物了
	{
		FECV_Str_Ptr->Dir 			= ECV_STOP;
		
		RFID_Info_Ptr->updateFlag 	= 0;

	#ifdef USE_SEND_ZIGBEE		
		Send_GettedGoods();
	#endif
		
		//printf("change to exit\r\n");
		
		//printf("change to back\r\n");
		
		CHANGE_TO_BACK_SLOW_MODE();
		
		#if USE_CIRCLE_INFO_RECODER
		CircleInfoStrPtr->CircleTime.CatchTime 	= (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
		CircleInfoStrPtr->TimeTempRec 			= SystemRunningTime;
		#endif

		FECV_Str_Ptr->ECV_Clean_Use_Status();
		BECV_Str_Ptr->ECV_Clean_Use_Status();

		ctrlParasPtr->walkingstep = step_exit;
	}
	else
	{
		
		Ecv_Para temp;
		temp.Dir 				= ECV_UP;
		temp.EcvSpeed 			= 100;
		temp.HallCountMode 		= ECV_USE_HALL_COUNT_MODE_DISABLE;
		FECV_Str_Ptr->ECV_SetPara(&temp);
		
	}
}

void step_exit_Func(void)
{
	static u8 flag = 0;
	
	ctrlParasPtr->gear = 4;
	
	if(1 == RFID_Info_Ptr->updateFlag)
	{
		RFID_Info_Ptr->updateFlag = 0;
		//printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);

		if(ctrlParasPtr->goalRFIDnode == RFID_Info_Ptr->rfidData)
		{
			CHANGE_TO_STOP_MODE();
			
			#ifdef USE_SEND_ZIGBEE		
			Send_FiberMachine();
			#endif
			
			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->CircleTime.ExitTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			#endif
			
			ctrlParasPtr->rifdAdaptFlag = 0;
			flag = 0;
			getWeightCtrl_Ptr->weightScanEnable = 1;
			MACHINE_ARM_WEIGHT_FIBER_SET();
			ctrlParasPtr->walkingstep = step_weigh;
		}
	}
	else
	{
		
		if(0 == flag)
		{
			if((backStatus == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
			{
				if(0x0000 == RMSDS_Ptr->MSD_Hex)
				{
					CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
					ctrlParasPtr->rifdAdaptFlag = 1;
					flag = 1;
					ctrlParasPtr->gear = 3;
					//printf("************* CHANGE_TO_GO_STRAIGHT_SLOW_MODE **************\r\n");
				}
			}
		}
		else
		{
			if((goStraightStatus == ctrlParasPtr->agvStatus) || (gSslow == ctrlParasPtr->agvStatus))
			{
				if(0x0000 == RMSDS_Ptr->MSD_Hex)
				{
					CHANGE_TO_BACK_SLOW_MODE();
					ctrlParasPtr->gear = 3;
					flag = 0;
					//printf("**************** CHANGE_TO_BACK_SLOW_MODE *********************\r\n");
				}
			}
		}
		
	}
}


void step_weigh_Func(void)
{
	
	if(Return_SW_Respond)
	{
		Warning_LED_NORMAL_STATUS();
		printf("step_weigh_Func Return_SW_Respond!\r\n");
		#if USE_CIRCLE_INFO_RECODER
		CircleInfoStrPtr->ManualOptTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
		CircleInfoStrPtr->TimeTempRec	= SystemRunningTime;
		#endif
		
		FECV_Str_Ptr->ECV_Clean_Use_Status();
		BECV_Str_Ptr->ECV_Clean_Use_Status();
		
		MOTOR1_HALL_COUNT_FLAG = 1;
		getWeightCtrl_Ptr->weightScanEnable = 0;
		
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
	else if((SpinStation_2 	== ctrlParasPtr->goalStation) || \
			(SpinStation_4 	== ctrlParasPtr->goalStation) || \
			(SpinStation_6 	== ctrlParasPtr->goalStation) || \
			(SpinStation_8 	== ctrlParasPtr->goalStation) || \
			(SpinStation_10 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_LEFT_MODE();
	}

	if(0 == stepFlag)
	{
		ctrlParasPtr->cirDuty = 13;
		
		stepFlag = 1;
	}
	else if(1 == stepFlag)
	{
		if( ( 0xFFFF == FMSDS_Ptr->MSD_Hex ) && ( 0xFFFF == RMSDS_Ptr->MSD_Hex ) )
		{
			ctrlParasPtr->cirDuty = 13;
			stepFlag = 2;
		}
	}
	else if(2 == stepFlag)
	{	
		if(0xFFFF != FMSDS_Ptr->MSD_Hex)
		{
			ctrlParasPtr->cirDuty = 9;
			//ctrlParasPtr->agvStatus = backStatus;
			if(1 == Origin_PatCtrl2(ctrlParasPtr->cirDuty))
			{
				CleanAllSpeed();
				
				CHANGE_TO_STOP_MODE();

				#ifdef USE_SEND_ZIGBEE
				Send_WaitForGoods();
				#endif
				
				stepFlag = 0;
				ctrlParasPtr->BSflag = 0;
				ctrlParasPtr->fgvflag = 1;
				
				ctrlParasPtr->gear = 17;

				#if USE_CIRCLE_INFO_RECODER
				CircleInfoStrPtr->CircleTime.BackCirTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
				CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
				#endif
				
				//printf("rightHall = %d, leftHall = %d\r\n", ctrlParasPtr->rightHallCounter, ctrlParasPtr->leftHallCounter);
				MOTOR1_HALL_COUNT_FLAG = 0;

				ctrlParasPtr->Use_WECV = 0;
				ctrlParasPtr->walkingstep = step_gB;
			}
			
		}
	}
	
}


void step_gB_Func(void)
{

	if(1 == RFID_Info_Ptr->updateFlag)
	{
		RFID_Info_Ptr->updateFlag = 0;

		RFID_Info_Ptr->rfidData = 0;
	}
	
	CHANGE_TO_BACK_MODE();
	
	
	if(ctrlParasPtr->crossRoadCountR <= 1)
	{
		Ecv_Para temp;
		
		CHANGE_TO_BACK_SLOW_MODE();
		ctrlParasPtr->gear = 4;
		
		temp.Dir = ECV_DOWN;
		WECV_Str_Ptr->ECV_SetPara(&temp);
		
	}
	
	if((0x0000 == FMSDS_Ptr->MSD_Hex) && (0x0000 == RMSDS_Ptr->MSD_Hex))
	{
		CHANGE_TO_STOP_MODE();
		ctrlParasPtr->gear = 17;
		RFID_Info_Ptr->updateFlag = 0;
		CMD_Flag_Ptr->cmdFlag = NcNone;
		
		#if USE_CIRCLE_INFO_RECODER
		CircleInfoStrPtr->CircleTime.BackTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
		CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
		#endif
		
		//WECV_Str_Ptr->ECV_Clean_Use_Status();
		ctrlParasPtr->originFlag = 0;

		//Clean_Weight_Func();
		
		ctrlParasPtr->walkingstep = step_wFTans;
		
	}
}

void step_wFTans_Func(void)
{
	static u8 step = 0;	
	static u32 timRec = 0;

	if(0 == step)
	{
		#ifdef USE_SEND_ZIGBEE
		Send_Arrive();
		#endif
		
		ctrlParasPtr->crossRoadCountF = ORIGIN_STATION_NODE + EXTRA_CROSS_ROAD_R;
		ctrlParasPtr->crossRoadCountR = ORIGIN_STATION_NODE + EXTRA_CROSS_ROAD_R - 1;
		
		step = 1;

		
	}
	else if(1 == step)
	{
		
		if(0 == timRec)
		{
			timRec = SystemRunningTime;
		}
		else
		{
			if(SystemRunningTime - timRec >= 50000)
			{
				timRec = 0;
				getWeightCtrl_Ptr->weightScanEnable = 1;
				getWeightCtrl_Ptr->getWeightCount = 0;
				step = 2;
			}
		}	
	}
	
	else if(2 == step)
	{
		static u16 weight = 0;
		
		if(getWeightCtrl_Ptr->getWeightCount >= 10)
		{
			if((weight / 10) <= 3)
			{
				CMD_Flag_Ptr->cmdFlag = GoodLeav;
				step = 3;
			}
			
			getWeightCtrl_Ptr->getWeightCount = 0;
			weight = 0;
		}
		else
		{
			weight += FiberglasInfo_Ptr->weight_H;
		}
	}
	
	
	if(Return_SW_Respond)
	{
		Delay_ms(200);
		if(Return_SW_Respond)
		{
			while(Return_SW_Respond);
			
			CMD_Flag_Ptr->cmdFlag = GoodLeav;
			printf("step_wFTans_Func Return_SW_Respond\r\n");
		}
	}
	
	if(GoodLeav == CMD_Flag_Ptr->cmdFlag)
	{
		step = 0;
		timRec = 0;
		//weight = 0;

		getWeightCtrl_Ptr->weightScanEnable = 0;
		CMD_Flag_Ptr->cmdFlag = NcNone;
		RFID_Info_Ptr->rfidData = 0x00;
		Zigbee_Ptr->runningInfo.Req_Station = 0x00;
		Zigbee_Ptr->runningInfo.Req_Type = TypeUnknow;
		CHANGE_TO_GO_STRAIGHT_MODE();
		ctrlParasPtr->crossRoadCountF = EXTRA_CROSS_ROAD_R - 1;
		ctrlParasPtr->crossRoadCountR = EXTRA_CROSS_ROAD_R - 2;

		#if USE_CIRCLE_INFO_RECODER
		CircleInfoStrPtr->TakeAwayTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
		CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
		#endif
				
		zigbeeRecvDataBuf_Delete();
		printf("zigbeeRecvDataBuf_Delete\r\n");
		Show_Queue_Data();
		
		if(zigbeeQueueCtrl.Total > 0)
		{
			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->CircleTime.BackToOriginTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			Save_OneCircleInfo(CircleInfoStrPtr);
			Clean_CircleInfoStr(CircleInfoStrPtr);
			#endif
		}
		
		RFID_Info_Ptr->rfidData = STATION_LM;
		ctrlParasPtr->originFlag = 0;
		ctrlParasPtr->StartupFlag = 0;
		RFID_Info_Ptr->updateFlag = 0;
		ctrlParasPtr->AutoCancel_Respond = 1;
		ctrlParasPtr->walkingstep = step_origin;
	}
	
	
}

void step_origin_Func(void)
{
	ctrlParasPtr->FSflag = 0;
	
	if(ctrlParasPtr->crossRoadCountF < 3)
	{
		CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
		ctrlParasPtr->gear = 5;
		//printf("ctrlParasPtr->gear = 3\r\n");
	}
	else
	{
		//ctrlParasPtr->gear = 10;
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

			if(0 == ctrlParasPtr->start_origin_mode)
			{
				ctrlParasPtr->start_origin_mode = 1;
			}

			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->CircleTime.BackToOriginTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			Save_OneCircleInfo(CircleInfoStrPtr);
			Clean_CircleInfoStr(CircleInfoStrPtr);
			#endif
			
		}
		else if(RFID_Info_Ptr->rfidData > 0x01)
		{
			ctrlParasPtr->BSflag = 0;
			
			CHANGE_TO_BACK_SLOW_MODE();
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

void step_origin_Func2(void)
{
	
	if(0 == ctrlParasPtr->StartupFlag)
	{
		
		if(1 == RFID_Info_Ptr->updateFlag)
		{
			//RFID_Info_Ptr->updateFlag = 0;
			ctrlParasPtr->StartupFlag = 1;
		}
		else
		{
			CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
			ctrlParasPtr->gear = 5;
		}
		
	}
	else
	{
		if(0 == ctrlParasPtr->rifdAdaptFlag)
		{
			if(RFID_Info_Ptr->rfidData < ORIGIN_STATION_NODE)	// AGV在所设原点之后
			{
				if(ctrlParasPtr->crossRoadCountF >= ORIGIN_STATION_NODE + EXTRA_CROSS_ROAD_R - 1)
				{
					CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
					ctrlParasPtr->rifdAdaptFlag = 1;
					ctrlParasPtr->gear = 3;
				}
				else
				{
					CHANGE_TO_GO_STRAIGHT_MODE();
					//printf("CHANGE_TO_GO_STRAIGHT_MODE = %d\r\n", ctrlParasPtr->gear);
					ctrlParasPtr->gear = 17;
				}
				
			}
			else if(RFID_Info_Ptr->rfidData > ORIGIN_STATION_NODE)		// AGV在原点之前
			{
				//printf("crossRoadCountR = %d\r\n", ctrlParasPtr->crossRoadCountR);
				
				if(ctrlParasPtr->crossRoadCountR <= ORIGIN_STATION_NODE + EXTRA_CROSS_ROAD_R - 1)
				{
					CHANGE_TO_BACK_SLOW_MODE();
					ctrlParasPtr->rifdAdaptFlag = 1;
					ctrlParasPtr->gear = 3;
				}
				else
				{
					CHANGE_TO_BACK_MODE();
					//printf("CHANGE_TO_BACK_MODE = %d\r\n", ctrlParasPtr->gear);
					ctrlParasPtr->gear = 15;
				}
				
			}
			
			/*
			if((0x0000 == FMSDS_Ptr->MSD_Hex) && (0x0000 == RMSDS_Ptr->MSD_Hex))
			{
				RFID_Info_Ptr->rfidData = STATION_LM;
			}
			*/
		}
		else
		{
			if((goStraightStatus == ctrlParasPtr->agvStatus) || (gSslow == ctrlParasPtr->agvStatus))
			{
				if(0x0000 == RMSDS_Ptr->MSD_Hex)
				{
					CHANGE_TO_BACK_SLOW_MODE();
					ctrlParasPtr->gear = 3;
				}
			}
			else if((backStatus == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
			{
				if(0x0000 == RMSDS_Ptr->MSD_Hex)
				{
					CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
					ctrlParasPtr->gear = 3;
				}
			}
		}
		
	}
	
	if(1 == RFID_Info_Ptr->updateFlag)
	{
		RFID_Info_Ptr->updateFlag = 0;
		
		if(ORIGIN_STATION_NODE == RFID_Info_Ptr->rfidData)	// AGV到达所设原点
		{
			CleanAllSpeed();
			CHANGE_TO_STOP_MODE();
			ctrlParasPtr->rifdAdaptFlag = 0;
			
			#if USE_CIRCLE_INFO_RECODER
			if(1 == CircleInfoStrPtr->lock)
			{
				CircleInfoStrPtr->CircleTime.BackToOriginTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
				CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
				Save_OneCircleInfo(CircleInfoStrPtr);
				Clean_CircleInfoStr(CircleInfoStrPtr);
			}
			#endif
			
			//printf("***************************!\r\n");
			ctrlParasPtr->originFlag = 1;
			ctrlParasPtr->walkingstep = step_stop;
			
		}
	}
	
	
}


void step_stop_Func(void)
{
	ctrlParasPtr->FSflag = 0;
	ctrlParasPtr->BSflag = 0;
	ctrlParasPtr->goalStation = ControlCenter;
	ctrlParasPtr->goalRFIDnode = 0x0000;
	ctrlParasPtr->walkingstep = step_stop;
	//ctrlParasPtr->crossRoadCountF = EXTRA_CROSS_ROAD_R + 1;
	//ctrlParasPtr->crossRoadCountR = EXTRA_CROSS_ROAD_R;
	//Zigbee_Ptr->runningInfo.Req_Station = 0x00;
	//Zigbee_Ptr->runningInfo.Req_Type = TypeUnknow;
}


void startup_origin_Func(void)
{

	if(1 == RFID_Info_Ptr->updateFlag)
	{
		
		RFID_Info_Ptr->updateFlag = 0;
		printf("startup_origin_Func\r\n");
		printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);
		//printf("1LHC = %d, RHC = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
		if(0x01 == RFID_Info_Ptr->rfidData)
		{
			CHANGE_TO_STOP_MODE();
			step_stop_Func();
			ctrlParasPtr->crossRoadCountF = EXTRA_CROSS_ROAD_R + 1;
			ctrlParasPtr->crossRoadCountF = EXTRA_CROSS_ROAD_R;
			ctrlParasPtr->start_origin_mode = 1;
			
		}
		else if(RFID_Info_Ptr->rfidData > 0x01)
		{
						
			CHANGE_TO_BACK_SLOW_MODE();
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




void Walking_Step_Controler(void)
{
	if(ctrlParasPtr->walkingstep < step_end)
	{
		walking_step[ctrlParasPtr->walkingstep]();
	}

	
}

void ECV_Machine_ARM_Init_Func(u8 *step, u32 *timRec)
{
	Ecv_Para temp;
	
	if(0 == *step)
	{
		temp.Dir			= ECV_DOWN;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_DISABLE;
		FECV_Str_Ptr->ECV_SetPara(&temp);

		temp.Dir			= ECV_DOWN;
		temp.EcvSpeed		= 60;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_DISABLE;
		BECV_Str_Ptr->ECV_SetPara(&temp);

		if((ECV_COMPLETE == BECV_Str_Ptr->UseStatus) && (ECV_COMPLETE == BECV_Str_Ptr->UseStatus))
		{
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
			*step = 1;
		}
		
	}
	else if(1 == *step)
	{
		temp.Dir			= ECV_UP;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
		temp.EcvHallCountCmp= HallCountCmpManager_Str_Ptr->BecvInit + 150;
		BECV_Str_Ptr->ECV_SetPara(&temp);

		if(ECV_COMPLETE == BECV_Str_Ptr->UseStatus)
		{
			BECV_Str_Ptr->ECV_Clean_Use_Status();
			*step = 2;
		}
		
	}
	else if(2 == *step)
	{
		temp.Dir			= ECV_DOWN;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
		temp.EcvHallCountCmp= 150;
		BECV_Str_Ptr->ECV_SetPara(&temp);

		if(ECV_COMPLETE == BECV_Str_Ptr->UseStatus)
		{
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
			*step = 0;
			MACHINE_ARM_INIT_CLEAN();
		}
	}
}


void ECV_Machine_ARM_WEIGHT_Small_Fiber_Func(u8 *step, u32 *timRec)
{
	Ecv_Para temp;
	
	if(0 == *step)
	{	
		#if 0
		
		temp.Dir			= ECV_UP;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_DISABLE;
		FECV_Str_Ptr->ECV_SetPara(&temp);

		temp.Dir			= ECV_UP;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
		temp.EcvHallCountCmp= 100;
		BECV_Str_Ptr->ECV_SetPara(&temp);
		
		if(FLMT_SW_UNRESPOND)
		{
			
			FECV_Str_Ptr->Dir = ECV_STOP;
			
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			*step = 1;
		}
		
		#else
		
		temp.Dir			= ECV_UP;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
		temp.EcvHallCountCmp= 600;
		FECV_Str_Ptr->ECV_SetPara(&temp);

		temp.Dir			= ECV_UP;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
		temp.EcvHallCountCmp= 100;
		BECV_Str_Ptr->ECV_SetPara(&temp);
		
		if((ECV_COMPLETE == FECV_Str_Ptr->UseStatus) && (ECV_COMPLETE == BECV_Str_Ptr->UseStatus))
		{
						
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
			*step = 2;
		}
		
		#endif
	}
	else if(1 == *step)
	{
		temp.Dir			= ECV_UP;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
		temp.EcvHallCountCmp= 200;
		FECV_Str_Ptr->ECV_SetPara(&temp);

		if((ECV_COMPLETE == FECV_Str_Ptr->UseStatus) && (ECV_COMPLETE == BECV_Str_Ptr->UseStatus))
		{
			*step = 2;
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
		}
		
	}
	else if(2 == *step)
	{
		if(0 == *timRec)
		{
			*timRec = SystemRunningTime;
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
		}
		else
		{
			
			if(RLMT_SW_RESPOND || (SystemRunningTime - *timRec >= 20000))
			{
				temp.Dir			= ECV_UP;
				temp.EcvSpeed		= 100;
				temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
				temp.EcvHallCountCmp= 200;
				FECV_Str_Ptr->ECV_SetPara(&temp);
				
				temp.Dir			= ECV_UP;
				temp.EcvSpeed		= 100;
				temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
				temp.EcvHallCountCmp= 320;
				BECV_Str_Ptr->ECV_SetPara(&temp);
				*timRec = 0;
				*step = 3;
			}
			
		}
		
	}
	else if(3 == *step)
	{
		if(ECV_COMPLETE == BECV_Str_Ptr->UseStatus)
		{
		#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->CircleTime.WeightTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
		#endif
			*step = 4;
		}			
		
	}
	else if(4 == *step)
	{
		if((ECV_COMPLETE == FECV_Str_Ptr->UseStatus) && (ECV_COMPLETE == BECV_Str_Ptr->UseStatus))
		{
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
			*step = 0;
			MACHINE_ARM_INIT_SET();
			MACHINE_ARM_WEIGHT_FIBER_CLEAN();
		}
	}
	
	if(1 == FECV_Str_Ptr->EcvHallCountTimeoutUpdate)
	{
		FECV_Str_Ptr->EcvHallCountTimeoutUpdate = 0;
		Warning_LED_ECV_TIMEOUT_STATUS();
	}
}

void ECV_Machine_ARM_Catch_Func(u8 *step, u32 *timRec)
{
	static u8 flag = 0;
	static u16 counter = 0;
	
	if(0 == *step)
	{
		
		Ecv_Para temp;
		temp.Dir				= ECV_UP;
		temp.EcvSpeed			= 100;
		temp.HallCountMode		= ECV_USE_HALL_COUNT_MODE_DISABLE;
		FECV_Str_Ptr->ECV_SetPara(&temp);
		FECV_Str_Ptr->EcvHallCountTimeoutUpdate = 0;
		*step = 1;
	}
	else if(1 == *step)
	{
		if(0 == flag)
		{
			if(LMT_SW_Respond) 	// 已经抓到货物了
			{
				FECV_Str_Ptr->Dir = ECV_STOP;

				FECV_Str_Ptr->ECV_Clean_Use_Status();

				ctrlParasPtr->walkingstep = step_exit;
				ctrlParasPtr->Catch_Goods_Flag = 1;

				MACHINE_ARM_CATCH_CLEAN();

				*step = 0;
			}
		}
		else if(1 == flag)
		{
			if((LMT_SW_Respond) || (FECV_Str_Ptr->EcvHallCount >= counter)) 	// 已经抓到货物了
			{
				FECV_Str_Ptr->Dir = ECV_STOP;

				FECV_Str_Ptr->ECV_Clean_Use_Status();

				ctrlParasPtr->walkingstep = step_exit;
				ctrlParasPtr->Catch_Goods_Flag = 1;

				MACHINE_ARM_CATCH_CLEAN();

				*step = 0;
			}
		}
		
		if(1 == FECV_Str_Ptr->EcvHallCountTimeoutUpdate)
		{
			FECV_Str_Ptr->EcvHallCountTimeoutUpdate = 0;
			counter = FECV_Str_Ptr->EcvHallCountTimeout;
			flag = 1;
		}
		
		
	}
	
}

void ECV_Machine_ARM_Toggle_Func(u8 *step, u32 *timRec)
{
	Ecv_Para temp;
	
	if(0 == *step)
	{
		temp.Dir			= ECV_UP;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
		temp.EcvHallCountCmp= 200;
		BECV_Str_Ptr->ECV_SetPara(&temp);

		if(ECV_COMPLETE == BECV_Str_Ptr->UseStatus)
		{
			BECV_Str_Ptr->ECV_Clean_Use_Status();
			*step = 1;
		}
		
	}
	else if(1 == *step)
	{
		temp.Dir			= ECV_DOWN;
		temp.EcvSpeed		= 100;
		temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_ENABLE;
		temp.EcvHallCountCmp= 200;
		BECV_Str_Ptr->ECV_SetPara(&temp);

		if(ECV_COMPLETE == BECV_Str_Ptr->UseStatus)
		{
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
			ctrlParasPtr->Machine_ARM_Toggle_Flag = 1;
			*step = 0;
			MACHINE_ARM_TOGGLE_CLEAN();
		}
	}
}

void Machinearm_Control_Handle(void)
{
	static u8 step = 0;
	static u32 timRec = 0;
	
	if(MACHINE_ARM_INIT_CHECK)
	{
		ECV_Machine_ARM_Init_Func(&step, &timRec);
	}
	else if(MACHINE_ARM_TOGGLE_CHECK)
	{
		ECV_Machine_ARM_Toggle_Func(&step, &timRec);
	}
	else if(MACHINE_ARM_CATCH_CHECK)
	{
		ECV_Machine_ARM_Catch_Func(&step, &timRec);
	}
	else if(MACHINE_ARM_WEIGHT_CHECK)
	{
		ECV_Machine_ARM_WEIGHT_Small_Fiber_Func(&step, &timRec);
	}
	
}

void ProtectFunc(void)
{
	static u32 timRec = 0;
	static u8 flag = 0;
	
	if(ProtectSW_F_RESPOND || ProtectSW_R_RESPOND)
	{
		MOTOR_POWER_OFF();
		//ECV_ALL_POWER_OFF();
		printf("Protect_Power_Off\r\n");
		flag = 1;
		
		ctrlParasPtr->FSflag = 0;
		ctrlParasPtr->BSflag = 0;
	}
	

	if(1 == flag)
	{
		if(Delay_Func(&timRec, 5000))
		{
			if(ProtectSW_F_UNRESPOND && ProtectSW_R_UNRESPOND)
			{
				MOTOR_POWER_ON();
			}
			
			flag = 0;
			
		}
	}
	else
	{
		timRec = 0;

		if((cirLeft != ctrlParasPtr->agvStatus) && (cirRight != ctrlParasPtr->agvStatus))
		{
			if((0xFFFF == FMSDS_Ptr->MSD_Hex) && (0xFFFF == RMSDS_Ptr->MSD_Hex))
			{
				MOTOR_POWER_OFF();
			}
			else
			{
				MOTOR_POWER_ON();
			}
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


void Recv_RFID_CrossRoad(u8 recvD)
{
	if((recvD >= 0x01) && (recvD <= 0x05))
	{
		//if(RFID_Info_Ptr->lock != recvD)
		if(0 == RFID_Info_Ptr->lock)
		{
			//RFID_Info_Ptr->lock = recvD;
			RFID_Info_Ptr->updateFlag = 1;
			//printf("recvD = %04x\r\n", recvD);
			RFID_Info_Ptr->rfidData = recvD;
			WarningLedCtrlPtr->twinkleFlag = 1;
			//BuzzerCtrlPtr->buzzerFlag = 1;
			
			ctrlParasPtr->crossRoadCountF = recvD + EXTRA_CROSS_ROAD_R;
			ctrlParasPtr->crossRoadCountR = recvD + EXTRA_CROSS_ROAD_R - 1;				
			//printf("\r\nIT GcrossRoadCountF = %d, crossRoadCountR = %d\r\n", ctrlParasPtr->crossRoadCountF, ctrlParasPtr->crossRoadCountR);

			printf("rfidData = %04x\r\n", RFID_Info_Ptr->rfidData);
		}		
		
	}
	else
	{
		RFID_Info_Ptr->noValide = 1;
	}
}

void ZigbeeRecv_Simu2(u8 *flag)
{
	
	if(1 == *flag)
	{	
		#if 1
		
		static u8 cir = 7;
		
		if((cir > 0) && (cir <= 10))
		{
			Zigbee_Ptr->ZigbeeRecvCmdUpdate = 1;
			
			Zigbee_Ptr->ZigbeeRecvCmdData[6] = 0x7f;
			Zigbee_Ptr->ZigbeeRecvCmdData[7] = ((cir - 1) << 4) | 0x03;
			printf("############ cir = %d, ZigbeeRecvCmdData[6] = %02x, ZigbeeRecvCmdData[7] = %02x\r\n", cir, Zigbee_Ptr->ZigbeeRecvCmdData[6], Zigbee_Ptr->ZigbeeRecvCmdData[7]);		
			cir += 6;
		}

		/*
		if(0x04 == RFID_Info_Ptr->rfidData)
		{
			Zigbee_Ptr->ZigbeeRecvCmdUpdate = 1;
				
			Zigbee_Ptr->ZigbeeRecvCmdData[6] = 0x7f;
			Zigbee_Ptr->ZigbeeRecvCmdData[7] = ((9 - 1) << 4) | 0x02;
			printf("########### ZigbeeRecvCmdData[6] = %02x, ZigbeeRecvCmdData[7] = %02x\r\n", Zigbee_Ptr->ZigbeeRecvCmdData[6], Zigbee_Ptr->ZigbeeRecvCmdData[7]);
			*flag = 2;
		}
		*/

		#else

		static u8 cir = 5;

		if((cir > 0) && (cir <= 10))
		{
			Zigbee_Ptr->ZigbeeRecvCmdUpdate = 1;
			
			Zigbee_Ptr->ZigbeeRecvCmdData[6] = 0x7f;
			Zigbee_Ptr->ZigbeeRecvCmdData[7] = ((cir - 1) << 4) | 0x01;
			printf("############ cir = %d, ZigbeeRecvCmdData[6] = %02x, ZigbeeRecvCmdData[7] = %02x\r\n", cir, Zigbee_Ptr->ZigbeeRecvCmdData[6], Zigbee_Ptr->ZigbeeRecvCmdData[7]);		
			cir += 5;
		}
		
		if(0x02 == RFID_Info_Ptr->rfidData)
		{
			Zigbee_Ptr->ZigbeeRecvCmdUpdate = 1;
			
			Zigbee_Ptr->ZigbeeRecvCmdData[6] = 0x7f;
			Zigbee_Ptr->ZigbeeRecvCmdData[7] = ((5 - 1) << 4) | 0x02;
			printf("########### ZigbeeRecvCmdData[6] = %02x, ZigbeeRecvCmdData[7] = %02x\r\n", Zigbee_Ptr->ZigbeeRecvCmdData[6], Zigbee_Ptr->ZigbeeRecvCmdData[7]);
			*flag = 2;
		}

		#endif
	}
}


void ZigbeeRecv_Simu(void)
{
#if 1

	//static u8 flag = 0;
	
	if(ctrlParasPtr->walkingstep >= step_origin)
	{
		
		if(Return_SW_LF_Respond && Return_SW_LR_Respond && Return_SW_RF_UnRespond && Return_SW_RR_UnRespond)
		{
			Delay_ms(20);
			if(Return_SW_LF_Respond && Return_SW_LR_Respond && Return_SW_RF_UnRespond && Return_SW_RR_UnRespond)
			{
				while(Return_SW_LF_Respond && Return_SW_LR_Respond && Return_SW_RF_UnRespond && Return_SW_RR_UnRespond);
				ctrlParasPtr->agvWalkingMode = ManualMode;
			}
		}
		/*
		else if(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_UnRespond && Return_SW_RR_Respond)
		{
			Delay_ms(20);
			if(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_UnRespond && Return_SW_RR_Respond)
			{
				while(Return_SW_RR_Respond);
				//flag = 1;
				Zigbee_Ptr->runningInfo.Req_Station = 0x000a;
				Zigbee_Ptr->recvValidDataFlag = 1;
			}
		}
		*/
	}


	//ZigbeeRecv_Simu2(&flag);
	
	
#else
	
	if(step_stop == ctrlParasPtr->walkingstep)
	{
		ManualModeEcvCtrlFunc();
	}
	
#endif
	
}


void SIMU_PWM_BreathWarningLED_Ctrl(void)
{
	static u32 timRec = 0;
	static u8 step = 0, flag = 0, counter = 0;
	static u16 dutyTime = 0;
	
	if(0 == timRec)
	{
		timRec = SystemRunningTime;
		Warning_LED_GREEN = 0;
		Warning_LED_ORANGE = 1;
		step = 0;
	}
	else
	{
		
		if(0 == step)
		{
			if(SystemRunningTime - timRec >= dutyTime)
			{
				Warning_LED_GREEN = 1;
				Warning_LED_ORANGE = 0;
				step = 1;
				timRec = SystemRunningTime;
			}
		}
		else if(1 == step)
		{
			if(SystemRunningTime - timRec >= (100 - dutyTime))
			{
				timRec = 0;
				step = 0;

				if(counter < 1)
				{
					counter++;
				}
				else
				{
					counter = 0;
					if(0 == dutyTime)
					{
						flag = 0;
					}
					else if(dutyTime >= 100)
					{
						flag = 1;
					}

					if(0 == flag)
					{
						dutyTime++;
					}
					else if(1 == flag)
					{
						dutyTime--;
					}
				}
				
				
			}
		}
		
	}
	
}


u8 BCD_Change2_Hex_u8(u8 u8_data)
{
	u8 hex = 0;

	hex = ((u8_data >> 4) * 10);
	hex += (u8_data & 0x0F);

	return hex;
}

u8 HEX_Change2_BCD_u8(u8 u8_data)
{
	u8 hex = 0;

	hex = ((u8_data / 10) << 4);
	hex |= (u8_data % 10);

	return hex;
}



void ProtectSW_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_Init(GPIOD, &GPIO_InitStructure);
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
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);

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



void SW_Gpio_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	/*打开APB2总线上的GPIOA时钟*/
	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

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
	//u8 cir = 0;
	
	//Trigger_Gpio_Init();
	//Delay_ns(1);
	Motion_Ctrl_GPIO_Init();
	
	PG_EXTI_CFG();
	
	CHANGE_TO_STOP_MODE();
	
	SW_Gpio_Init();

	//Trigger_Gpio_Init();
	
	ctrlParasPtr->agvStatus 				= StatusStart;
	ctrlParasPtr->settedSpeed 				= 0;
	ctrlParasPtr->rightMotorRealSpeed 		= 0;
	ctrlParasPtr->rightMotorSettedSpeed 	= 0;
	ctrlParasPtr->leftMotorRealSpeed 		= 0;
	ctrlParasPtr->leftMotorSettedSpeed 		= 0;
	ctrlParasPtr->speedMode 				= PWM_MODE;
	ctrlParasPtr->agvWalkingMode 			= AutomaticMode;
	ctrlParasPtr->leftMotorSpeedOffset 		= 0;
	ctrlParasPtr->rightMotorSpeedOffset 	= 0;
	ctrlParasPtr->leftHallIntervalTime 		= 0x00;
	ctrlParasPtr->rightHallIntervalTime 	= 0x00;
	ctrlParasPtr->HLavg 					= 0x00;
	ctrlParasPtr->HRavg 					= 0x00;
	ctrlParasPtr->gear 						= 0;
	ctrlParasPtr->FSflag 					= 0;
	ctrlParasPtr->BSflag 					= 0;
	ctrlParasPtr->goalRFIDnode 				= 0;
	ctrlParasPtr->goalStation 				= ControlCenter;
	ctrlParasPtr->walkingstep 				= step_origin;
	ctrlParasPtr->crossRoadCountF 			= 0;
	ctrlParasPtr->crossRoadUpdateF 			= 0;
	ctrlParasPtr->crossRoadCountR 			= 0;
	ctrlParasPtr->crossRoadUpdateR 			= 0;
	ctrlParasPtr->LP_duty 					= 0;
	ctrlParasPtr->RP_duty 					= 0;
	ctrlParasPtr->LD_duty 					= 0;
	ctrlParasPtr->RD_duty 					= 0;
	ctrlParasPtr->originFlag 				= 0;
	ctrlParasPtr->rifdAdaptFlag 			= 0;
	ctrlParasPtr->manualCtrl 				= Man_Stop;
	ctrlParasPtr->fgvflag 					= 0;
	ctrlParasPtr->CrossRoadHallCountFlag 	= 0;
	ctrlParasPtr->CrossRoadHallCountL 		= 0;
	ctrlParasPtr->CrossRoadHallCountR 		= 0;
	ctrlParasPtr->HallCounterFlag 			= 0;
	ctrlParasPtr->rightHallCounterCMP		= CIR_HALL;
	ctrlParasPtr->leftHallCounterCMP		= CIR_HALL;
	ctrlParasPtr->LED_Warning				= 0x00;
	ctrlParasPtr->Machine_ARM_Toggle_Flag	= 0x00;
	ctrlParasPtr->Catch_Goods_Flag			= 0x00;
	ctrlParasPtr->ECV_StepFlag				= 0x00;
	ctrlParasPtr->AutoCancel_Respond		= 0x01;
	ctrlParasPtr->AccCtrl.AccCtrlFunc		= AcceCtrl_Func;
	
	agv_walking[StatusStart] 		= NullFunc;
	agv_walking[stopStatus] 		= walking_stopStatus;
	agv_walking[goStraightStatus] 	= AGV_Correct_gS_8ug;
	agv_walking[backStatus] 		= AGV_Correct_back_ug;
	agv_walking[gSslow] 			= gS_slow2;
	agv_walking[bSslow] 			= back_slow2;

	#if USE_HALL_CTRL
	agv_walking[cirLeft] 			= walking_cir_hall;
	agv_walking[cirRight] 			= walking_cir_hall;
	#else
	agv_walking[cirLeft] 			= walking_cir;
	agv_walking[cirRight] 			= walking_cir;
	#endif

	#if 1
	walking_step[step_gS]			= step_gS_Func;
	#else
	walking_step[step_gS]			= step_gS_Func2;
	#endif
	walking_step[step_gVeer]		= step_gVeer_Func;
	walking_step[step_entry]		= step_entry_Func;
	walking_step[step_catch]		= step_catch_Func;
	walking_step[step_exit]			= step_exit_Func;
	walking_step[step_weigh]		= step_weigh_Func;
	walking_step[step_bVeer]		= step_bVeer_Func;
	walking_step[step_gB]			= step_gB_Func;
	walking_step[step_wFTans]		= step_wFTans_Func;
	walking_step[step_origin]		= step_origin_Func2;
	walking_step[step_stop]			= step_stop_Func;
	

	ZBandRFIDmapping[ControlCenter] 	= 0x0000;
	ZBandRFIDmapping[SpinStation_1] 	= 0x0001;
	ZBandRFIDmapping[SpinStation_2] 	= 0x0002;
	ZBandRFIDmapping[SpinStation_3] 	= 0x0003;
	ZBandRFIDmapping[SpinStation_4] 	= 0x0004;
	ZBandRFIDmapping[SpinStation_5] 	= 0x0005;
	ZBandRFIDmapping[SpinStation_6] 	= 0x0006;
	ZBandRFIDmapping[SpinStation_7] 	= 0x0007;
	ZBandRFIDmapping[SpinStation_8] 	= 0x0008;
	ZBandRFIDmapping[SpinStation_9] 	= 0x0009;
	ZBandRFIDmapping[SpinStation_10] 	= 0x000A;
	
	#if USE_MPU6050
	for(cir = 0; cir < MAX_DAMP_ADAPT_NUM; cir++)
	{
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
	}
	#endif
	
	BuzzerCtrlPtr->buzzerFlag = 0;
	BuzzerCtrlPtr->buzzerTime_ms = 150;
	BuzzerCtrlPtr->buzzerNum = 2;
	BuzzerCtrlPtr->buzzerCtrlFunc = BuzzerCtrlFunc;
}



