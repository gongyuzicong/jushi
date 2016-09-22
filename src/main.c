#include "main_include.h"
#include "cfg_rcc.h"
#include "cfg_gpio.h"
#include "cfg_usart.h"
#include "timer_opts.h"
#include "key_opts.h"
#include "buffer.h"
//#include "dht11_opts.h"
#include "pwm_opts.h"
#include "motion_control.h"
#include "exti_opts.h"
#include "nrf24l01_opts.h"
#include "spi_opts.h"
#include "magn_sensor.h"
#include "zigbee.h"
#include "rfid.h"
#include "mpu6050.h"
#include "i2c_opts.h"
#include "buffer.h"
#include "eeprom.h" 
#include "ads1256.h"
#include "fiberglas.h"
#include "lcd.h"
#include "watch_dog.h"
#include "circle_recoder.h"
#include "ecv_control.h"

void SystemInit(void)
{	
	//keyConfig();
	BufferOpts_Init();
	Delay_Init(72);
	//Timer2_Init(10000, 7199);	// 1s
	TIM3_Init(65535, 35999);		
	//Timer4_Init(9, 719);		// 0.1ms
	Timer6_Init(9, 719);
	//Timer5_Init(9, 719);
	Motion_Ctrl_Init();
	Pwm_Init();
	ECV_Init();
	SPI_Initial();
	RFID_Usart_Init();
	//NFR24L01_Init();
	//ExtiInit();
	Magn_Sensor_Init();
	Zigbee_Init();
	My_I2C_Init();
	ProtectSW_GPIO_Config();
	//MPU6050_init();
	
	
	ADS1256_Init();
	LCD_INIT();

	#if USE_CIRCLE_INFO_RECODER
	circle_recoder_init();
	#endif
	
	Get_Weight_Offset_Data();
	
	errorInfo.errorType = errorTypeBegin;
	errorInfo.errorData = 0;

	
}



int main(void)
{
	//TIMx_PwmOpts_Struct TIM3_PWM;
	//int cir = 0, ayDec = 0;
	
	static u16 hexF = 0, hexR = 0;
	//static Agv_MS_Location mslRecF = AgvInits, mslRecR = AgvInits;
	
	CB_RCC_Config();	/*配置系统时钟*/
	CB_GPIO_Config();	/*配置GPIO口*/
	CB_USART_Config();	/*配置USART*/
	SystemInit();

	//mslRecF = FMSDS_Ptr->AgvMSLocation;
	//mslRecR = FMSDS_Ptr->AgvMSLocation;
	Warning_LED_RED = 0;
	
	//MPU6050_Data_init3();
	ECV_POWER_ON();
	#if 0
	FECV_DOWN();
	BECV_DOWN();
	//WECV_DOWN();
	Delay_ns(3);
	FECV_STOP();
	BECV_UP();
	//Delay_ns(1);
	//Delay_ms(600);
	Delay_ns(2);
	Delay_ms(500);
	BECV_DOWN();
	Delay_ns(1);
	Delay_ms(900);
	BECV_STOP();
	#else
	//while(ARM_Reset() != 1);
	ctrlParasPtr->armResetFlag = 1;
	#endif
	
	MOTOR_POWER_ON();
	//MOTOR_POWER_OFF();
	//AGV_Walking_Test();
	Get_Weight_Offset_Data_One();

	#if USE_IWDG
	IWatch_Dog_Init();
	#endif
	
	
	printf("Start\r\n");
	
	ctrlParasPtr->gear = 10;
	//CHANGE_TO_GO_STRAIGHT_MODE();
	//Zigbee_Ptr->recvValidDataFlag = 1;
	//Zigbee_Ptr->recvId = 0x0008;
	//ctrlParasPtr->FSflag = 1;
	CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
	
	while(1)
	{		
		
		#if 1
		
		#if USE_IWDG
		u32 timRec = 0;
		if(Delay_Func(&timRec, 500))
		{
			IWDG_RELOAD();
		}
		#endif
		
		//ProtectFunc();
		Read_RTC_Data();	// 年月日
		Lcd_Handle();		// 小屏幕操作函数
		//SIMU_PWM_BreathBoardLED_Ctrl();
		Scan_Weight_Func();	// 扫描称重模块数据
		WarningLedCtrlPtr->twinkleCtrlFunc();
		ARM_Reset2();		// 机械手臂复位操作


		/****控制逻辑部分 start****/
		if(TestMode == ctrlParasPtr->agvWalkingMode)
		{
			//printf("l=%d,r=%d\r\n", ctrlParasPtr->leftHallIntervalTime, ctrlParasPtr->rightHallIntervalTime);
			//AVG_Calu_Program();
			AGV_Correct_2();
		}
		else if(ManualMode == ctrlParasPtr->agvWalkingMode)
		{
			//MOTOR_POWER_ON();
			ManualModeFunc(ctrlParasPtr->manualCtrl);
			#if USE_ECV
			if((Man_CirL == ctrlParasPtr->manualCtrl) || (Man_CirR == ctrlParasPtr->manualCtrl))
			{
				ctrlParasPtr->manualCtrl = Man_Stop;
			}
			#endif
		}
		else if(RFID_Setting_Mode == ctrlParasPtr->agvWalkingMode)
		{
			
			MOTOR_POWER_ON();
			
			if(1 == RFID_Info_Ptr->noValide)
			{
				
			}
			
		}
		else if(AutomaticMode == ctrlParasPtr->agvWalkingMode)
		{
			Magn_Sensor_Scan();		// 磁传感器数据处理
			Receive_handle2();		// ZigBee数据接收处理函数

			
			if(0 == ctrlParasPtr->start_origin_mode)
			//if(0)
			{
				
				if(gSslow == ctrlParasPtr->agvStatus)
				{
					gS_slow2(5);
				}
				else if(bSslow == ctrlParasPtr->agvStatus)
				{
					back_slow2(5);
				}
				
				startup_origin_Func();
				
			}
			else
			{
				//originP();
				
				CrossRoad_Count2();				// 磁条十字交叉路口的计算
				
				Get_Zigbee_Info_From_Buf();		// 从队列当中取出接收到的ZigBee信息
				
				RFID_Goal_Node_Analy();			// 分析哪个RFID点转
				
				Walking_Step_Controler();		// 整个大逻辑的控制


				/****小车驱动轮控制****/
				if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_End))
				{
					
				#if 0
					AGV_Walking();
				#else
					
					if(goStraightStatus == ctrlParasPtr->agvStatus)		// 高速直行状态
					{
						
						AGV_Correct_gS_8ug(ctrlParasPtr->gear);
					}
					else if(backStatus == ctrlParasPtr->agvStatus)		// 高速后退状态
					{
						
						AGV_Correct_back_ug(ctrlParasPtr->gear);
					}
					else if(cirLeft == ctrlParasPtr->agvStatus)			// 左转状态
					{
						//walking_cirLeft(ctrlParasPtr->gear);
						walking_cir(ctrlParasPtr->cirDuty);
					}
					else if(cirRight == ctrlParasPtr->agvStatus)		// 右转状态
					{
						//walking_cirRight(ctrlParasPtr->gear);
						walking_cir(ctrlParasPtr->cirDuty);
					}
					else if(gSslow == ctrlParasPtr->agvStatus)			// 低速直行
					{
						gS_slow2(ctrlParasPtr->gear);
					}
					else if(bSslow == ctrlParasPtr->agvStatus)			// 低速后退
					{
						back_slow2(ctrlParasPtr->gear);
					}
											
				#endif
					
				}
				
				//AGV_Change_Mode();
				
				//AGV_Proc();
				// 在原点待机时自动回正
				if(ControlCenter == ctrlParasPtr->goalStation)
				{
					//SIMU_PWM_BreathWarningLED_Ctrl();
					
					ctrlParasPtr->cirDuty = 8;
					Origin_PatCtrl(ctrlParasPtr->cirDuty);
					//AutoRunningFunc();
				}
				
				//LeftOrRight_Counter();
				
			}
			
			
			ZigbeeResendInfo_Ptr->resendCtrlFunc();
			BuzzerCtrlPtr->buzzerCtrlFunc();

			// 小车调试信息的打印
			if((hexF != FMSDS_Ptr->MSD_Hex) || (hexR != RMSDS_Ptr->MSD_Hex))
			{
				hexF = FMSDS_Ptr->MSD_Hex;
				hexR = RMSDS_Ptr->MSD_Hex;
				
			#if 0
				
				if((goStraightStatus == ctrlParasPtr->agvStatus) && (0 != ctrlParasPtr->FSflag))
				{
					Show_Infomation();
					//show_Excel_Analysis_Info();
				}
				else if((0 != ctrlParasPtr->BSflag) && (backStatus == ctrlParasPtr->agvStatus))
				{
					Show_Infomation();
					//show_Excel_Analysis_Info();
				}
				
			#else
				
				//Show_Infomation();
				
			#endif
				
			}
		}

		#else

		/*
		Receive_handle();

		Get_Zigbee_Info_From_Buf();

		if(1 == Zigbee_Ptr->recvValidDataFlag)
		{
			 Zigbee_Ptr->recvValidDataFlag = 0;
			 
			if(0x0001 == Zigbee_Ptr->recvId)
			{
				Delay_ns(3);
				Send_GettedGoods(1);
				Delay_ns(3);
				Send_WaitForGoods();
				Delay_ns(10);
				Send_Arrive();
			}
			else if(0x0000 == Zigbee_Ptr->recvId)
			{
				
			}
			
		}
		*/
		//BUZZER_1 = 1;
		//BUZZER_2 = 1;

		//Delay_ns(1);

		//BUZZER_1 = 0;
		//BUZZER_2 = 0;

		//Delay_ns(1);

		//Read_RTC_Data();
		
		/*
		u8 data1[9] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
		u8 data2[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		u8 data3[9] = {0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01};
		u8 cir = 9;
		static u8 flag = 0;

		if(0 == flag)
		{
			flag = 1;
			EEPROM_Write_Data(1, data1, 9);
		}
		else if(1 == flag)
		{
			flag = 0;
			EEPROM_Write_Data(1, data3, 9);
		}
		
		Delay_ms(10);
		EEPROM_Read_Data(1, data2, 9);
		for(cir = 0; cir < 9; cir++)
		{
			printf("%02x ", data2[cir]);
		}
		printf("\r\n");
		
		Delay_ns(1);
		*/
		//Lcd_Handle();
		//Get_Weight_Data();
		//MA_TEST();
				
		//Lcd_Handle();

		/*
		static u32 timRec = 0;
		static u8 step = 0;

		if(0 == step)
		{
			FECV_UP();
			BECV_UP();
			if(Delay_Func(&timRec, 4000))
			{
				step = 1;
				BECV_STOP();
				timRec = 0;
				//printf("BECV_STOP\r\n");
				WarningLedCtrlPtr->twinkleFlag = 1;
			}
		}
		else if(1 == step)
		{
			if(Delay_Func(&timRec, 7000))
			{
				FECV_STOP();
				step = 2;
				//printf("FECV_STOP\r\n");
				WarningLedCtrlPtr->twinkleFlag = 1;
			}
		}

		WarningLedCtrlPtr->twinkleCtrlFunc();
		*/
		/*
		static u8 step = 0;
		static u32 timRec = 0;
		
				
		if(0 == step)
		{
			if(0 == Return_SW)
			{
				Delay_ms(20);
				if(0 == Return_SW)
				{
					while(0 == Return_SW);
					step = 1;
					BECV_UP();
					FECV_UP();
					timRec = 0;
					printf("step = 1\r\n");
				}
				
			}
		}
		else if(1 == step)
		{
			if(Delay_Func(&timRec, 1200))
			{
				BECV_STOP();
				FECV_STOP();
			}

			if(0 == Return_SW)
			{
				Delay_ms(20);
				if(0 == Return_SW)
				{
					while(0 == Return_SW);
					step = 2;
					BECV_UP();
					FECV_UP();
					timRec = 0;
					printf("step = 2\r\n");
				}
			}
		}
		else if(2 == step)
		{
			if(Delay_Func(&timRec, 2000))
			{
				BECV_STOP();
				FECV_STOP();
			}

			if(0 == Return_SW)
			{
				Delay_ms(20);
				if(0 == Return_SW)
				{
					while(0 == Return_SW);
					step = 3;
					BECV_DOWN();
					timRec = 0;
					printf("step = 3\r\n");
				}
			}
		}
		else if(3 == step)
		{
			if(Delay_Func(&timRec, 1000))
			{
				BECV_STOP();
			}

			if(0 == Return_SW)
			{
				Delay_ms(20);
				if(0 == Return_SW)
				{
					while(0 == Return_SW);
					step = 4;
					BECV_DOWN();
					FECV_DOWN();
					timRec = 0;
					printf("step = 4\r\n");
				}
			}
		}
		else if(4 == step)
		{
			if(0 == Return_SW)
			{
				Delay_ms(20);
				if(0 == Return_SW)
				{
					while(0 == Return_SW);
					step = 5;
					BECV_UP();
					FECV_STOP();
					timRec = 0;
					printf("step = 5\r\n");
				}
			}
		}
		else if(5 == step)
		{
			if(Delay_Func(&timRec, 2500))
			{
				BECV_DOWN();
				step = 6;
				timRec = 0;
				printf("step = 6\r\n");
			}
		}
		else if(6 == step)
		{
			if(Delay_Func(&timRec, 1900))
			{
				BECV_STOP();
				timRec = 0;
				step = 0;
			}
		}
		*/
		
		
		#endif
		
	}

	
}





