#include "cfg_gpio.h"
#include "timer_opts.h"
#include "pwm_opts.h"
#include "magn_sensor.h"
#include "zigbee.h"
#include "mpu6050.h"
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

TimRec rec[30];
u8 recH = 0;

HallCount HallCountArr[6];
HallCount CrossRoadHallCountArrGS[6];
HallCount CrossRoadHallCountArrGB[6];


u16 ZBandRFIDmapping[11];

u8 	AgvGear[MAX_GEAR_NUM] = {0, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
u8 	AgvGearCompDutyLF[MAX_GEAR_NUM] = {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
u8 	AgvGearCompDutyRF[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 	AgvGearCompDutyLB[MAX_GEAR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 	AgvGearCompDutyRB[MAX_GEAR_NUM] = {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
u8 	AgvGearK[MAX_GEAR_NUM] = {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 16, 17, 18, 19, 20, 21, 22};
u8 	AgvGear7CDLF[MAX_GEAR_OFFSET] = {0, 2, 7, 8, 10, 12, 14, 16, 18, 20, 20};
GearShiftCtrl 		gearCtrl;
GearShiftCtrl_P 	gearCtrlPtr = &gearCtrl;

MPU6050_Para 		mpu6050DS;
MPU6050_Para_P 		mpu6050DS_ptr = &mpu6050DS;


u8 LTM[9] = {181, 104, 73, 55, 45, 37, 32, 29, 26};
u8 RTM[9] = {132, 86, 63, 49, 41, 34, 30, 27, 24};

u8 DutyTableLow[10] = {2, 4, 8, 10, 12, 16, 16, 16, 16, 16};
u8 DutyTable[10] = {2, 8, 15, 20, 20, 20, 20, 20, 20, 20};

//u8 DutyTable[10] = {2, 4, 6, 8, 8, 8, 8, 8, 8, 8};

u8 DutyTable_Duty10[10] = {0, 0, 2, 3, 4, 5, 6, 7, 8, 9};

LED_Twinkle 	WarningLedCtrl;
LED_Twinkle_P 	WarningLedCtrlPtr = &WarningLedCtrl;

Buzzer_Ctrl 	BuzzerCtrl;
Buzzer_Ctrl_P 	BuzzerCtrlPtr = &BuzzerCtrl;

ControlerParaStruct 	ctrlParas;
ControlerParaStruct_P 	ctrlParasPtr = &ctrlParas;
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


void walking_cir(u8 duty)
{
	static u8 lmSpeed = 0, rmSpeed = 0;

	lmSpeed = duty + AgvGearCompDutyLF[2];
	rmSpeed = duty + AgvGearCompDutyLF[2];
	
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
			
			if(centCount > ShiftTrigTime)
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
		
		if(centCount >= ShiftTrigTime)
		{
			ctrlParasPtr->FSflag = 1;
			ctrlParasPtr->comflag = 6331;
			
			startCount = 0;
			
		}

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


void gS_startup_mode3(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	static Agv_MS_Location mslRec = AgvInits;
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

	if(mslRec == FMSDS_Ptr->AgvMSLocation)
	{
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
			
		}
	}
	else
	{
		startCount = 0;
		mslRec = FMSDS_Ptr->AgvMSLocation;
	}

	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	set_duty(lmSpeed, rmSpeed);
	
}


void gS_startup_mode4(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	static Agv_MS_Location mslRec = AgvInits;
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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
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

	
	if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
	{
		if(mslRec == FMSDS_Ptr->AgvMSLocation)
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
				ctrlParasPtr->FSflag = 1;
				ctrlParasPtr->comflag = 6331;
				
				startCount = 0;
				
			}
		}
		else
		{
			startCount = 0;
			mslRec = FMSDS_Ptr->AgvMSLocation;
		}
	}
	else
	{
		startCount = 0;
	}
	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	set_duty(lmSpeed, rmSpeed);
	
}

void gS_startup_mode5(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
		lmSpeedP = 0;
		rmSpeedP = gainDuty[Agv_MS_Left_0_5 - FMSDS_Ptr->AgvMSLocation];
		
		startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
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
		printf("centCount = %d\r\n", centCount);
		if(centCount >= ShiftTrigTime)
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

	set_duty(lmSpeed, rmSpeed);
	
}


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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
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

void gS_startup_mode7(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
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
		if((RMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_0_5) && (RMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_0_5))
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
				ctrlParasPtr->FSflag = 1;
				ctrlParasPtr->gear = 7;
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
	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

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
	rmSpeed = AgvGear[gearRecod] - rmSpeedP + 2;

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
		
		if(centCount >= ShiftTrigTime)
		{
			ctrlParasPtr->BSflag = 1;
			ctrlParasPtr->comflag = 6331;
			
			startCount = 0;
			
		}

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

void bS_startup_mode3(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	static Agv_MS_Location mslRec = AgvInits;
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
		
		//startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		rmSpeedP = 0;
		
		//startCount = 0;
	}
	else if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeedP = 0;
		rmSpeedP = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}

	if(mslRec == FMSDS_Ptr->AgvMSLocation)
	{
		if(0 == startCount)
		{
			startCount = SystemRunningTime;
		}
		else
		{
			centCount = SystemRunningTime - startCount;
			//printf("centCount = %d, mslRec = %d, AgvMSLocation = %d\r\n", centCount, mslRec, FMSDS_Ptr->AgvMSLocation);
		}
		
		if(centCount > ShiftTrigTime)
		{
			ctrlParasPtr->BSflag = 1;
			ctrlParasPtr->comflag = 6331;
			
			startCount = 0;
			
		}
	}
	else
	{
		startCount = 0;
		mslRec = FMSDS_Ptr->AgvMSLocation;
	}

	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP + 1;

	set_duty(rmSpeed, lmSpeed);
	
}

void bS_startup_mode4(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	static Agv_MS_Location mslRec = AgvInits;
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
		
		//startCount = 0;
	}
	else if((FMSDS_Ptr->AgvMSLocation > Agv_MS_Center) && (FMSDS_Ptr->AgvMSLocation < Agv_MS_Right_End))
	{		
		ctrlParasPtr->comflag = 632;
		
		lmSpeedP = gainDuty[FMSDS_Ptr->AgvMSLocation - Agv_MS_Right_0_5];
		rmSpeedP = 0;
		
		//startCount = 0;
	}
	else if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeedP = 0;
		rmSpeedP = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}


	if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
	{
		if(mslRec == FMSDS_Ptr->AgvMSLocation)
		{
			if(0 == startCount)
			{
				startCount = SystemRunningTime;
			}
			else
			{
				centCount = SystemRunningTime - startCount;
				//printf("centCount = %d, mslRec = %d, AgvMSLocation = %d\r\n", centCount, mslRec, FMSDS_Ptr->AgvMSLocation);
			}
			
			if(centCount >= ShiftTrigTime)
			{
				ctrlParasPtr->BSflag = 1;
				ctrlParasPtr->comflag = 6331;
				
				startCount = 0;
				
			}
		}
		else
		{
			startCount = 0;
			mslRec = FMSDS_Ptr->AgvMSLocation;
		}
	}
	else
	{
		startCount = 0;
	}
	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP + 1;

	set_duty(rmSpeed, lmSpeed);
	
}

void bS_startup_mode5(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	//static Agv_MS_Location mslRec = AgvInits;
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
		rmSpeedP = 0;
		
		startCount = 0;
	}
	else if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
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
		
		//printf("centCount = %d\r\n", centCount);
		if(centCount >= ShiftTrigTime)
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
	rmSpeed = AgvGear[gearRecod] - rmSpeedP + 1;

	set_duty(rmSpeed, lmSpeed);
	
}

void bS_startup_mode6(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
	u32 centCount = 0;
	static u32 startCount = 0;
	//static Agv_MS_Location mslRec = AgvInits;
	u32 tm = 0;
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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
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
	else if(Agv_MS_Center == FMSDS_Ptr->AgvMSLocation)
	{
		ctrlParasPtr->comflag = 634;

		lmSpeedP = 0;
		rmSpeedP = 0;
		
		FMSDS_Ptr->MaxRecoder = Agv_MS_Center;
		
	}


	if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_1) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_1))
	{
		if(0 == startCount)
		{
			startCount = SystemRunningTime;
		}
		else
		{
			centCount = SystemRunningTime - startCount;
			//printf("centCount = %d, mslRec = %d, AgvMSLocation = %d\r\n", centCount, mslRec, FMSDS_Ptr->AgvMSLocation);
		}

		if(1 == ctrlParasPtr->fgvflag)
		{
			tm = 20000;
			
		}
		else
		{
			tm = 0;
		}
		
		if(centCount >= ShiftTrigTime + tm)
		{
			//ctrlParasPtr->BSflag = 1;
			ctrlParasPtr->fgvflag = 0;
			ctrlParasPtr->gear = 7;
			ctrlParasPtr->comflag = 6331;
			
			startCount = 0;
			
		}
	}
	else
	{
		startCount = 0;
	}
	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP + 1;

	set_duty(rmSpeed, lmSpeed);
	
}

void bS_startup_mode7(u8 gear)
{
	u8 lmSpeed = 0, rmSpeed = 0, gearRecod = 0, lmSpeedP = 0, rmSpeedP = 0;
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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
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
			ctrlParasPtr->BSflag = 1;
			ctrlParasPtr->gear = 10;
			ctrlParasPtr->comflag = 6331;
			
			startCount = 0;
			
		}
	}
	else
	{
		startCount = 0;
	}
	//lmSpeed = AgvGear[gearRecod] + AgvGearCompDutyLF[gearRecod] - lmSpeedP;
	//rmSpeed = AgvGear[gearRecod] + AgvGearCompDutyRF[gearRecod] - rmSpeedP;
	lmSpeed = AgvGear[gearRecod] - lmSpeedP;
	rmSpeed = AgvGear[gearRecod] - rmSpeedP;

	set_duty(rmSpeed, lmSpeed);
	
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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
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

		//lmSpeed = AgvGear[2] + DutyTable[Agv_MS_Left_1 - FMSDS_Ptr->AgvMSLocation] + FLG[0][2];
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

void scale_1_mode18_back(u8 gear)
{
	//u8 AgvGearS1CDLF[20] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10};
	u8 gearRecod = 0;
	u8 lmSpeedSet = 0, rmSpeedSet = 0, lmSpeed = 0, rmSpeed = 0, lmSpeedPat = 0, rmSpeedPat = 0;
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
	
		
	Pattern_ctrl4(&lmSpeedPat, &rmSpeedPat);
		

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
	
	
	set_duty2(lmSpeedSet, rmSpeedSet);
	
}


void so_This_is_P(u8 *lmSpeedPat_PP, u8 *rmSpeedPat_PP)
{	
	//u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16};
	//u8 AgvPatAngOut[MAX_OUT] = {1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10};
	//u8 AgvPatAngOut[21] = {1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
	u8 AgvPatAngOut[21] = {1, 1, 1, 2, 2, 3, 3, 5, 5, 7, 7, 9, 9, 11, 11, 13, 13, 15, 15, 15, 15};
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
	
	so_This_is_D5(&lmSpeedPat_D, &rmSpeedPat_D);
	
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
		
		rmSpeedSet = AgvGear[gearRecod] - rmSpeedPat_P - rmSpeedPat_D - 1;
		
	}

	set_duty_Com(lmSpeedSet, rmSpeedSet);
}

void HighSpeed_Protect(void)
{
	static u8 count = 0;
	static Agv_MS_Location maxRec = AgvInits;

	if((AgvCent2Right == FMSDS_Ptr->agvDirection) || (AgvCent2Left == FMSDS_Ptr->agvDirection))
	{
		maxRec = FMSDS_Ptr->AgvMSLocation;
	}
	else if((AgvRight2Cent == FMSDS_Ptr->agvDirection) || (AgvLeft2Cent == FMSDS_Ptr->agvDirection))
	{
		if((maxRec < Agv_MS_Left_5) || (maxRec > Agv_MS_Right_5))
		{
			count++;
		}
		else
		{
			count = 0;
		}
	}

	if(count >= 2)
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
	

	ECV_Ctrl_Func(FECV_Str_Ptr);
	ECV_Ctrl_Func_SW(BECV_Str_Ptr);
	
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
			bS_startup_mode8(4);
		}
		else if(1 == ctrlParasPtr->BSflag)
		{
			
			//scale_1_mode18_back(gearRecod);
			//printf("gearRecod = %d\r\n", gearRecod);
			scale_1_mode20_back(gearRecod);
			HighSpeed_Protect2();
		}
		else if(2 == ctrlParasPtr->BSflag)
		{
			// 紧急模式
			gS_urgency_mode();
			
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
		u16 data = 0;
		
		get_zigbeeData(&data);

		printf("get_data = %d\r\n", data);

		if(0 == data)
		{
			printf("recvId error!\r\n");
		}
		else
		{
			Zigbee_Ptr->recvValidDataFlag = 1;
			Zigbee_Ptr->recvId = data;
			
			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->Station = data;
			CircleInfoStrPtr->RESPOND_TIME = BackgroudRTC_Rec;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			#endif
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
		Warning_LED_RED = 0;
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

	//if((goStraightStatus == ctrlParasPtr->agvStatus) || (backStatus == ctrlParasPtr->agvStatus) ||\
		//(gSslow == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
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
						printf("MAIN GcrossRoadCountF = %d, GcrossRoadCountR = %d\r\n", ctrlParasPtr->crossRoadCountF, ctrlParasPtr->crossRoadCountR);
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
						printf("MAIN GcrossRoadCountR = %d, GcrossRoadCountF = %d\r\n", ctrlParasPtr->crossRoadCountR, ctrlParasPtr->crossRoadCountF);
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
						printf("MAIN BcrossRoadCountR = %d, BcrossRoadCountF = %d\r\n", ctrlParasPtr->crossRoadCountR, ctrlParasPtr->crossRoadCountF);
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
						printf("MAIN BcrossRoadCountF = %d, BcrossRoadCountR = %d\r\n", ctrlParasPtr->crossRoadCountF, ctrlParasPtr->crossRoadCountR);
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
	static u8 flag = 0;
	
	#if USE_R_DEC_SPEED
	static u32 timRec = 0;
	#endif
	
	//printf("rfidData = %02x, goalRFIDnode = %d\r\n", RFID_Info_Ptr->rfidData, ctrlParasPtr->goalRFIDnode);
	if((1 == ctrlParasPtr->originFlag) && (ORIGIN_STATION_NODE == ctrlParasPtr->goalRFIDnode))
	{
		if(AutoReq == CMD_Flag_Ptr->Req_Flag)
		{
			
			if(Return_SW_Respond)
			{
				ctrlParasPtr->originFlag = 0;
				ctrlParasPtr->walkingstep = step_gVeer;
			}
		}
		else
		{
			ctrlParasPtr->originFlag = 0;
			ctrlParasPtr->walkingstep = step_gVeer;
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
			}
		}

		if(1 == flag)
		{
			if(GoodReq == CMD_Flag_Ptr->Req_Flag)
			{
				flag = 0;
				#if USE_CIRCLE_INFO_RECODER
				CircleInfoStrPtr->CircleTime.Go2RFIDTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
				CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
				#endif
				CMD_Flag_Ptr->Req_Flag = NcNone;
				CMD_Flag_Ptr->Cancel_Flag = NcNone;
				ctrlParasPtr->rifdAdaptFlag = 0;
				ctrlParasPtr->walkingstep	= step_gVeer;
			}
			else
			{
				
				if(Return_SW_Respond)
				//if(Return_SW_LF_Respond)
				{
					flag = 0;
					CircleInfoStrPtr->TimeTempRec 			= SystemRunningTime;
					CMD_Flag_Ptr->Req_Flag = NcNone;
					CMD_Flag_Ptr->Cancel_Flag = NcNone;
					ctrlParasPtr->rifdAdaptFlag = 0;
					printf("AutoReq step_gVeer\r\n");
					ctrlParasPtr->walkingstep = step_gVeer;
					
				}
				/*
				else if(Return_SW_LR_Respond)
				{
					Delay_ms(20);
					if(Return_SW_LR_Respond)
					{
						while(Return_SW_LR_Respond);

						receive_state = 1;
			
						nc_receive[6] = 0x7f;
						nc_receive[7] = ((7 - 1) << 4) | 0x04;
						printf("########### nc_receive[6] = %02x, nc_receive[7] = %02x\r\n", nc_receive[6], nc_receive[7]);
					}
				}
				*/
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
	
	if((SpinStation_1 == ctrlParasPtr->goalStation) || \
		(SpinStation_3 == ctrlParasPtr->goalStation) || \
		(SpinStation_5 == ctrlParasPtr->goalStation) || \
		(SpinStation_7 == ctrlParasPtr->goalStation) || \
		(SpinStation_9 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_LEFT_MODE();
		//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
	}
	else if((SpinStation_2 == ctrlParasPtr->goalStation) || \
			(SpinStation_4 == ctrlParasPtr->goalStation) || \
			(SpinStation_6 == ctrlParasPtr->goalStation) || \
			(SpinStation_8 == ctrlParasPtr->goalStation) || \
			(SpinStation_10 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_RIGHT_MODE();
		//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
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
					Delay_ns(1);
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
					Delay_ns(1);
					stepFlag = 0;
					ctrlParasPtr->walkingstep = step_entry;
				}
				
			}
		
			
		}
	}
	
}

void step_gVeer_Func2(void)
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
				
				
				//if(AutoReq == CMD_Flag_Ptr->Req_Flag)
				if(0)
				{
					
					if(Return_SW_Respond)
					{
						stepFlag = 0;
						CircleInfoStrPtr->TimeTempRec 			= SystemRunningTime;
						CMD_Flag_Ptr->Req_Flag = NcNone;
						printf("AutoReq step_entry\r\n");
						ctrlParasPtr->walkingstep = step_entry;
					}
					
				}
				//else if(GoodReq == CMD_Flag_Ptr->Req_Flag)
				else
				{
					stepFlag = 0;
					CMD_Flag_Ptr->Req_Flag = NcNone;
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

void step_gVeer_Func3(void)
{
	static u8 stepFlag = 0;

	//ctrlParasPtr->rifdAdaptFlag = 0;

	//if(1 == ctrlParasPtr->goalStation % 2)
	if((SpinStation_1 == ctrlParasPtr->goalStation) || \
	   (SpinStation_3 == ctrlParasPtr->goalStation) || \
	   (SpinStation_5 == ctrlParasPtr->goalStation) || \
	   (SpinStation_7 == ctrlParasPtr->goalStation) || \
	   (SpinStation_9 == ctrlParasPtr->goalStation))
	{
		CHANGE_TO_CIR_LEFT_MODE();
		//printf("goalStation = %d\r\n", ctrlParasPtr->goalStation);
	}
	//else if(0 == ctrlParasPtr->goalStation % 2)
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
		if((ctrlParasPtr->leftHallCounter >= 100) && (ctrlParasPtr->rightHallCounter >= 100))
		{
			CleanAllSpeed();
					
			CHANGE_TO_STOP_MODE();
			//Delay_ns(1);
			stepFlag = 0;

			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->CircleTime.GoCirTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			#endif
			
			ctrlParasPtr->walkingstep = step_entry;
		}
		else
		{
			if(ctrlParasPtr->leftHallCounter >= 100)
			{
				MOTOR_LEFT_STOP_PIN_SET();
			}
			else
			{
				
			}
			
			if(ctrlParasPtr->rightHallCounter >= 100)
			{
				MOTOR_RIGHT_STOP_PIN_SET();
			}
			else
			{
				
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
		printf("go to step_catch*****************************\r\n");
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
		Send_GettedGoods3();
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
	static u8 step = 0;
	static u32 timRec = 0;
	Ecv_Para temp;

#if 1

	#if 0
	
	if(0 == step)
	{
		temp.Dir 			= ECV_UP;
		temp.EcvSpeed 		= 100;
		temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_DISABLE;
		FECV_Str_Ptr->ECV_SetPara(&temp);

		/*
		if(FECV_Str_Ptr->EcvHallCount >= 350)
		{
			temp.Dir 			= ECV_UP;
			temp.EcvSpeed 		= 100;
			temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
			temp.EcvHallCountCmp= 60;
			BECV_Str_Ptr->ECV_SetPara(&temp);
		}
		*/
		
		if(FLMT_SW_UNRESPOND)
		{
			
			FECV_Str_Ptr->Dir = ECV_STOP;
			if(BECV_Str_Ptr->Dir != ECV_STOP)
			{
				BECV_Str_Ptr->Dir = ECV_STOP;
			}
			
			step = 1;
		}
	}
	else if(1 == step)
	{

		if(0 == timRec)
		{
			timRec = SystemRunningTime;
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
		}
		else
		{
			if((SystemRunningTime - timRec >= 10000) || RLMT_SW_RESPOND)
			{
				temp.Dir 			= ECV_UP;
				temp.EcvSpeed 		= 100;
				temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_DISABLE;
				temp.EcvHallCountCmp= 800;
				FECV_Str_Ptr->ECV_SetPara(&temp);
				
				temp.Dir 			= ECV_UP;
				temp.EcvSpeed 		= 100;
				temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
				temp.EcvHallCountCmp= 200;
				BECV_Str_Ptr->ECV_SetPara(&temp);
				
				step = 2;
			}
		}
		
	}

	#else

	if(0 == step)
	{
		temp.Dir 			= ECV_UP;
		temp.EcvSpeed 		= 100;
		temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_DISABLE;
		FECV_Str_Ptr->ECV_SetPara(&temp);

		/*
		if(FECV_Str_Ptr->EcvHallCount >= 350)
		{
			temp.Dir 			= ECV_UP;
			temp.EcvSpeed 		= 100;
			temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
			temp.EcvHallCountCmp= 60;
			BECV_Str_Ptr->ECV_SetPara(&temp);
		}
		*/
		
		if(FLMT_SW_UNRESPOND)
		{
			
			FECV_Str_Ptr->Dir = ECV_STOP;
			if(BECV_Str_Ptr->Dir != ECV_STOP)
			{
				BECV_Str_Ptr->Dir = ECV_STOP;
			}
			
			step = 1;
		}
	}
	else if(1 == step)
	{

		if(0 == timRec)
		{
			timRec = SystemRunningTime;
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
		}
		else
		{
			if((SystemRunningTime - timRec >= 20000) || RLMT_SW_RESPOND)
			{
				temp.Dir 			= ECV_UP;
				temp.EcvSpeed 		= 100;
				temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_DISABLE;
				temp.EcvHallCountCmp= 800;
				FECV_Str_Ptr->ECV_SetPara(&temp);
				
				temp.Dir 			= ECV_UP;
				temp.EcvSpeed 		= 100;
				temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
				temp.EcvHallCountCmp= 200;
				BECV_Str_Ptr->ECV_SetPara(&temp);
				
				step = 2;
			}
		}
		
	}

	#endif

#else

	if(0 == step)
	{
		temp.Dir 			= ECV_UP;
		temp.EcvSpeed 		= 100;
		temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_DISABLE;
		FECV_Str_Ptr->ECV_SetPara(&temp);

		/*
		if(FECV_Str_Ptr->EcvHallCount >= 350)
		{
			temp.Dir 			= ECV_UP;
			temp.EcvSpeed 		= 100;
			temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
			temp.EcvHallCountCmp= 60;
			BECV_Str_Ptr->ECV_SetPara(&temp);
		}
		*/
		
		if(FLMT_SW_UNRESPOND)
		{
			
			FECV_Str_Ptr->Dir = ECV_STOP;
			if(BECV_Str_Ptr->Dir != ECV_STOP)
			{
				BECV_Str_Ptr->Dir = ECV_STOP;
			}
			
			step = 1;
		}
		
	}
	else if(1 == step)
	{
		if(0 == timRec)
		{
			timRec = SystemRunningTime;
			FECV_Str_Ptr->ECV_Clean_Use_Status();
			BECV_Str_Ptr->ECV_Clean_Use_Status();
		}
		else
		{
			
			if(RLMT_SW_RESPOND || (SystemRunningTime - timRec >= 20000))
			{
				temp.Dir 			= ECV_UP;
				temp.EcvSpeed 		= 100;
				temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_DISABLE;
				temp.EcvHallCountCmp= 800;
				//FECV_Str_Ptr->ECV_SetPara(&temp);
				
				temp.Dir 			= ECV_UP;
				temp.EcvSpeed 		= 100;
				temp.HallCountMode 	= ECV_USE_HALL_COUNT_MODE_ENABLE;
				temp.EcvHallCountCmp= 420;
				BECV_Str_Ptr->ECV_SetPara(&temp);
				
				step = 2;
			}
			
		}
		
	}
	
#endif
	
	
	if(Return_SW_Respond)
	//if(Return_SW_LR_Respond)
	{
		#if 1
		#if USE_CIRCLE_INFO_RECODER
		CircleInfoStrPtr->ManualOptTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
		CircleInfoStrPtr->TimeTempRec 	= SystemRunningTime;
		#endif
		
		FECV_Str_Ptr->ECV_Clean_Use_Status();
		BECV_Str_Ptr->ECV_Clean_Use_Status();
		//Get_Weight_Data();
		step = 0;
		timRec = 0;
		MOTOR1_HALL_COUNT_FLAG = 1;
		ctrlParasPtr->walkingstep = step_bVeer;

		#else
		#ifdef USE_SEND_ZIGBEE		
		Send_FiberMachine();
		#endif
		#endif
	}
	#if 0
	else if(Return_SW_RR_Respond)
	{
		Delay_ms(20);
		if(Return_SW_RR_Respond)
		{
			while(Return_SW_RR_Respond){}

			#ifdef USE_SEND_ZIGBEE		
			Send_FiberMachine();
			#endif
			
		}
			
	}
	#endif
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
				#ifdef USE_SEND_ZIGBEE
					Send_WaitForGoods();
				#endif
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


void step_bVeer_Func2(void)
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
				Ecv_Para ctr;
				CleanAllSpeed();
				
				CHANGE_TO_STOP_MODE();
				//Delay_ns(1);
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
				
				printf("rightHall = %d, leftHall = %d\r\n", ctrlParasPtr->rightHallCounter, ctrlParasPtr->leftHallCounter);
				MOTOR1_HALL_COUNT_FLAG = 0;

				ctr.Dir 			= ECV_DOWN;
				ctr.EcvSpeed		= 100;
				ctr.HallCountMode	= ECV_USE_HALL_COUNT_MODE_DISABLE;
				FECV_Str_Ptr->ECV_SetPara(&ctr);
				
				ctr.Dir 			= ECV_DOWN;
				ctr.EcvSpeed		= 80;
				ctr.HallCountMode	= ECV_USE_HALL_COUNT_MODE_DISABLE;
				BECV_Str_Ptr->ECV_SetPara(&ctr);
				ctrlParasPtr->Use_WECV = 0;
				ctrlParasPtr->walkingstep = step_gB;
			}
			
		}
	}
	
}


void step_gB_Func(void)
{
	Ecv_Para temp;

	if(1 == RFID_Info_Ptr->updateFlag)
	{
		RFID_Info_Ptr->updateFlag = 0;

		RFID_Info_Ptr->rfidData = 0;
	}
	
	CHANGE_TO_BACK_MODE();

	temp.Dir 			= ECV_DOWN;
	temp.EcvSpeed 		= 100;
	temp.HallCountMode	= ECV_USE_HALL_COUNT_MODE_DISABLE;
	FECV_Str_Ptr->ECV_SetPara(&temp);

	temp.EcvSpeed 		= 80;
	BECV_Str_Ptr->ECV_SetPara(&temp);
	
	
	//if((ctrlParasPtr->crossRoadCountR <= 1) && (ctrlParasPtr->crossRoadCountF <= 2))
	if(ctrlParasPtr->crossRoadCountR <= 1)
	{
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
		
		FECV_Str_Ptr->Dir = ECV_STOP;
		FECV_Str_Ptr->ECV_Clean_Use_Status();
		BECV_Str_Ptr->ECV_Clean_Use_Status();
		WECV_Str_Ptr->ECV_Clean_Use_Status();
		ctrlParasPtr->walkingstep = step_wFTans;
		
	}
}

void step_wFTans_Func(void)
{
	static u32 timRec = 0;
	Ecv_Para temp;
	
	if(0 == timRec)
	{
	#ifdef USE_SEND_ZIGBEE
		Send_Arrive();
	#endif
		ctrlParasPtr->crossRoadCountF = ORIGIN_STATION_NODE + EXTRA_CROSS_ROAD_R;
		ctrlParasPtr->crossRoadCountR = ORIGIN_STATION_NODE + EXTRA_CROSS_ROAD_R - 1;
		
		timRec = SystemRunningTime;
		temp.Dir 				= ECV_UP;
		temp.EcvHallCountCmp 	= HallCountCmpManager_Str_Ptr->BecvInit;
		temp.EcvSpeed			= 100;
		temp.HallCountMode		= ECV_USE_HALL_COUNT_MODE_ENABLE;
		BECV_Str_Ptr->ECV_SetPara(&temp);
		//printf("timRec = %d\r\n", timRec);
	}
	else
	{
		
		if(Return_SW_Respond)
		{
			CMD_Flag_Ptr->cmdFlag = GoodLeav;
		}
		
		if(GoodLeav == CMD_Flag_Ptr->cmdFlag)
		{
			CMD_Flag_Ptr->cmdFlag = NcNone;
			timRec = 0;
			RFID_Info_Ptr->rfidData = 0x00;
			Zigbee_Ptr->recvId = 0x00;
			CHANGE_TO_GO_STRAIGHT_MODE();
			ctrlParasPtr->crossRoadCountF = EXTRA_CROSS_ROAD_R;
			ctrlParasPtr->crossRoadCountR = EXTRA_CROSS_ROAD_R - 1;

			#if USE_CIRCLE_INFO_RECODER
			CircleInfoStrPtr->TakeAwayTime = (SystemRunningTime - CircleInfoStrPtr->TimeTempRec) / 1000;
			CircleInfoStrPtr->TimeTempRec = SystemRunningTime;
			#endif
			
			BECV_Str_Ptr->ECV_Clean_Use_Status();
			
			zigbeeRecvDataBuf_Delete();
			ctrlParasPtr->walkingstep = step_origin;
		}
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
				if(ctrlParasPtr->crossRoadCountF >= ORIGIN_STATION_NODE + EXTRA_CROSS_ROAD_R)
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

			if((0x0000 == FMSDS_Ptr->MSD_Hex) && (0x0000 == RMSDS_Ptr->MSD_Hex))
			{
				RFID_Info_Ptr->rfidData = STATION_LM;
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
	//Zigbee_Ptr->recvId = 0x00;
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


void AutoRunningFunc(void)
{
	static u8 cir = 5;
	
	receive_state = 1;
	nc_receive[6] = 0x7f;
	
	nc_receive[7] = (cir << 4) | 0x01;

	if(cir < 9)
	{
		cir++;
	}
	else
	{
		cir = 5;
	}
	
}

void Walking_Step_Controler(void)
{
	static WalkStep stepRec = step_stop;
	
	if(step_gS == ctrlParasPtr->walkingstep)
	{
		#if 1
		step_gS_Func();
		#else
		step_gS_Func2();
		#endif
	}
	else if(step_gVeer == ctrlParasPtr->walkingstep)
	{
		step_gVeer_Func2();
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
		step_bVeer_Func2();
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
		//step_origin_Func();
		step_origin_Func2();
	}
	else if(step_stop == ctrlParasPtr->walkingstep)
	{
		step_stop_Func();
	}

	if(stepRec != ctrlParasPtr->walkingstep)
	{
		stepRec = ctrlParasPtr->walkingstep;
		printf("\r\n   ******** walkingstep = %d ********\r\n", ctrlParasPtr->walkingstep);
		if(step_gS == ctrlParasPtr->walkingstep)
		{
			printf("  step_gS\t");
			printf("goalRFIDnode = %d\t", ctrlParasPtr->goalRFIDnode);
			printf("originFlag = %d\t", ctrlParasPtr->originFlag);
			printf("rifdAdaptFlag = %d\t", ctrlParasPtr->rifdAdaptFlag);
			printf("Cancel_Flag = %d\t", CMD_Flag_Ptr->Cancel_Flag);
			printf("goalRFIDnode = %d\t", ctrlParasPtr->goalRFIDnode);
			printf("rfidData = %d\t", RFID_Info_Ptr->rfidData);
			printf("gear = %d\t", ctrlParasPtr->gear);
			printf("agvStatus %d\t", ctrlParasPtr->agvStatus);
			
			printf("\r\n\r\n");
		}
		else if(step_origin == ctrlParasPtr->walkingstep)
		{
			printf("step_origin\t");
			printf("StartupFlag = %d\rt", ctrlParasPtr->StartupFlag);
			printf("rfidData = %d\t", RFID_Info_Ptr->rfidData);
			printf("crossRoadCountF = %d\t", ctrlParasPtr->crossRoadCountF);
			printf("crossRoadCountR = %d\t", ctrlParasPtr->crossRoadCountR);
			printf("gear = %d\t", ctrlParasPtr->gear);
			printf("agvStatus %d\t", ctrlParasPtr->agvStatus);
			printf("\r\n");
		}
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
			Warning_LED_RED = 1;
			step = 0;
		}
		else
		{
			
			if(0 == step)
			{
				if(SystemRunningTime - timRec >= WarningLedCtrlPtr->intervalTime_ms * 10)
				{
					Warning_LED_RED = 0;
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


void Recv_RFID_CrossRoad(u8 recvD)
{
	if((recvD >= 0x01) && (recvD <= 0x05))
	{
		//if(RFID_Info_Ptr->lock != recvD)
		if(0 == RFID_Info_Ptr->lock)
		{
			//RFID_Info_Ptr->lock = recvD;
			RFID_Info_Ptr->updateFlag = 1;
			printf("recvD = %04x\r\n", recvD);
			RFID_Info_Ptr->rfidData = recvD;
			WarningLedCtrlPtr->twinkleFlag = 1;
			//BuzzerCtrlPtr->buzzerFlag = 1;
			
			ctrlParasPtr->crossRoadCountF = recvD + EXTRA_CROSS_ROAD_R;
			ctrlParasPtr->crossRoadCountR = recvD + EXTRA_CROSS_ROAD_R - 1;				
			printf("\r\nIT GcrossRoadCountF = %d, crossRoadCountR = %d\r\n", ctrlParasPtr->crossRoadCountF, ctrlParasPtr->crossRoadCountR);

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
			receive_state = 1;
			
			nc_receive[6] = 0x7f;
			nc_receive[7] = ((cir - 1) << 4) | 0x03;
			printf("############ cir = %d, nc_receive[6] = %02x, nc_receive[7] = %02x\r\n", cir, nc_receive[6], nc_receive[7]);		
			cir += 6;
		}

		/*
		if(0x04 == RFID_Info_Ptr->rfidData)
		{
			receive_state = 1;
				
			nc_receive[6] = 0x7f;
			nc_receive[7] = ((9 - 1) << 4) | 0x02;
			printf("########### nc_receive[6] = %02x, nc_receive[7] = %02x\r\n", nc_receive[6], nc_receive[7]);
			*flag = 2;
		}
		*/

		#else

		static u8 cir = 5;

		if((cir > 0) && (cir <= 10))
		{
			receive_state = 1;
			
			nc_receive[6] = 0x7f;
			nc_receive[7] = ((cir - 1) << 4) | 0x01;
			printf("############ cir = %d, nc_receive[6] = %02x, nc_receive[7] = %02x\r\n", cir, nc_receive[6], nc_receive[7]);		
			cir += 5;
		}
		
		if(0x02 == RFID_Info_Ptr->rfidData)
		{
			receive_state = 1;
			
			nc_receive[6] = 0x7f;
			nc_receive[7] = ((5 - 1) << 4) | 0x02;
			printf("########### nc_receive[6] = %02x, nc_receive[7] = %02x\r\n", nc_receive[6], nc_receive[7]);
			*flag = 2;
		}

		#endif
	}
}


void ZigbeeRecv_Simu(void)
{
	#if 0
	
	if(step_stop == ctrlParasPtr->walkingstep)
	{
		Zigbee_Ptr->recvValidDataFlag = 1;
		
		if(Zigbee_Ptr->recvId < 10)
		{
			Zigbee_Ptr->recvId++;
		}
		else
		{
			Zigbee_Ptr->recvId = 1;
		}
		
	}
	
	#else
	
	#if 1

	//static u8 flag = 0;
	
	if(ctrlParasPtr->walkingstep >= step_origin)
	{
		
		if(Return_SW_LF_Respond && Return_SW_LR_Respond && Return_SW_RF_UnRespond && Return_SW_RR_UnRespond)
		{
			printf("in2\r\n");
			Delay_ms(20);
			if(Return_SW_LF_Respond && Return_SW_LR_Respond && Return_SW_RF_UnRespond && Return_SW_RR_UnRespond)
			{
				while(Return_SW_LF_Respond && Return_SW_LR_Respond && Return_SW_RF_UnRespond && Return_SW_RR_UnRespond);
				ctrlParasPtr->agvWalkingMode = ManualMode;
			}
		}
		else if(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_UnRespond && Return_SW_RR_Respond)
		{
			Delay_ms(20);
			if(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_UnRespond && Return_SW_RR_Respond)
			{
				while(Return_SW_RR_Respond);
				//flag = 1;
				Zigbee_Ptr->recvId = 0x000a;
				Zigbee_Ptr->recvValidDataFlag = 1;
			}
		}
		
	}


	//ZigbeeRecv_Simu2(&flag);
	
	
	#else
	
	if(step_stop == ctrlParasPtr->walkingstep)
	{
		ManualModeEcvCtrlFunc();
	}
	
	#endif
	
	
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


void SIMU_PWM_BreathBoardLED_Ctrl(void)
{
	static u32 timRec = 0;
	static u8 step = 0, flag = 0, counter = 0;
	static u16 dutyTime = 0;
	
	if(0 == timRec)
	{
		timRec = SystemRunningTime;
		PCout(5) = 0;
		step = 0;
	}
	else
	{
		
		if(0 == step)
		{
			if(SystemRunningTime - timRec >= dutyTime)
			{
				PCout(5) = 1;
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
	u8 cir = 0;
	
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
	
	agv_walking_func[StatusStart] 		= NullFunc;
	agv_walking_func[stopStatus] 		= walking_stopStatus;
	agv_walking_func[goStraightStatus] 	= AGV_Correct_gS_8ug;
	agv_walking_func[backStatus] 		= AGV_Correct_back_ug;
	agv_walking_func[cirLeft] 			= walking_cir;
	agv_walking_func[cirRight] 			= walking_cir;
	agv_walking_func[gSslow] 			= gS_slow2;
	agv_walking_func[bSslow] 			= back_slow2;
	

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

	CrossRoadHallCountArrGS[0].HallCountLeft 	= 0;
	CrossRoadHallCountArrGS[1].HallCountLeft 	= 29;
	CrossRoadHallCountArrGS[2].HallCountLeft 	= 29;
	CrossRoadHallCountArrGS[3].HallCountLeft 	= 97;
	CrossRoadHallCountArrGS[4].HallCountLeft 	= 120;
	CrossRoadHallCountArrGS[5].HallCountLeft 	= 120;

	CrossRoadHallCountArrGS[0].HallCountRight 	= 0;
	CrossRoadHallCountArrGS[1].HallCountRight 	= 29;
	CrossRoadHallCountArrGS[2].HallCountRight 	= 29;
	CrossRoadHallCountArrGS[3].HallCountRight 	= 97;
	CrossRoadHallCountArrGS[4].HallCountRight 	= 120;
	CrossRoadHallCountArrGS[5].HallCountRight 	= 120;

	CrossRoadHallCountArrGB[0].HallCountLeft 	= 0;
	CrossRoadHallCountArrGB[1].HallCountLeft 	= 29;
	CrossRoadHallCountArrGB[2].HallCountLeft 	= 29;
	CrossRoadHallCountArrGB[3].HallCountLeft 	= 29;
	CrossRoadHallCountArrGB[4].HallCountLeft 	= 29;
	CrossRoadHallCountArrGB[5].HallCountLeft 	= 29;

	CrossRoadHallCountArrGB[0].HallCountRight 	= 0;
	CrossRoadHallCountArrGB[1].HallCountRight 	= 29;
	CrossRoadHallCountArrGB[2].HallCountRight 	= 29;
	CrossRoadHallCountArrGB[3].HallCountRight 	= 29;
	CrossRoadHallCountArrGB[4].HallCountRight 	= 29;
	CrossRoadHallCountArrGB[5].HallCountRight 	= 29;


	
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

		WarningLedCtrlPtr->twinkleFlag = 0;
		WarningLedCtrlPtr->intervalTime_ms = 500;
		WarningLedCtrlPtr->twinkleNum = 2;
		WarningLedCtrlPtr->twinkleCtrlFunc = WarningLedTwinkleCtrl;

		BuzzerCtrlPtr->buzzerFlag = 0;
		BuzzerCtrlPtr->buzzerTime_ms = 150;
		BuzzerCtrlPtr->buzzerNum = 2;
		BuzzerCtrlPtr->buzzerCtrlFunc = BuzzerCtrlFunc;

		
	}
	
}



