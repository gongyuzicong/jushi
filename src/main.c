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
	
	SPI_Initial();
	RFID_Usart_Init();
	//NFR24L01_Init();
	//ExtiInit();
	Magn_Sensor_Init();
	Zigbee_Init();
	My_I2C_Init();
	ProtectSW_GPIO_Config();
	//MPU6050_init();
	ECV_Init();
	
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
	Warning_LED_RED 	= 1;
	Warning_LED_GREEN 	= 0;
	Warning_LED_ORANGE 	= 0;
	//MPU6050_Data_init3();
	
	FECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
	
	BECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
	
	WECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
	WECV_Str_Ptr->Dir = ECV_DOWN;
	
	
	M_A_Init2();
	
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
	CHANGE_TO_GO_STRAIGHT_SLOW_MODE();
	
	while(1)
	{
		
		#if 1
		
		#if USE_IWDG
		u32 timRec = 0;
		if(Delay_Func(&timRec, 500))
		{
			IWDG_RELOAD();
			timRec = 0;
		}
		#endif
		
		//ProtectFunc();
		Read_RTC_Data();						// 年月日
		Lcd_Handle();							// 小屏幕操作函数
		SIMU_PWM_BreathBoardLED_Ctrl();			// 模拟PWM控制主控板LED呼吸灯
		Scan_Weight_Func();						// 扫描称重模块数据
		WarningLedCtrlPtr->twinkleCtrlFunc();	// 警告灯闪烁控制
		ECV_Ctrl_Func(FECV_Str_Ptr);			// 前电缸控制
		ECV_Ctrl_Func(BECV_Str_Ptr);			// 后电缸控制
		ECV_Ctrl_Func_W(WECV_Str_Ptr);			// 直行辅助轮电缸控制
		
		
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

			if(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_Respond && Return_SW_RR_Respond)
			{
				Delay_ms(20);
				if(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_Respond && Return_SW_RR_Respond)
				{
					while(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_Respond && Return_SW_RR_Respond);
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}
				
			}
			else
			{
				ManualModeEcvCtrlFunc();
			}
			
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
			
			ZigbeeRecv_Simu();
			
			
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
				
				
				/************小车驱动轮控制************/
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
						#if USE_HALL_CTRL
						walking_cir_hall(ctrlParasPtr->cirDuty);
						#else
						walking_cir(ctrlParasPtr->cirDuty);
						#endif
						
					}
					else if(cirRight == ctrlParasPtr->agvStatus)		// 右转状态
					{
						#if USE_HALL_CTRL
						walking_cir_hall(ctrlParasPtr->cirDuty);
						#else
						walking_cir(ctrlParasPtr->cirDuty);
						#endif
						
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
					SIMU_PWM_BreathWarningLED_Ctrl();
					
					ctrlParasPtr->cirDuty = 8;
					Origin_PatCtrl(ctrlParasPtr->cirDuty);
					//AutoRunningFunc();
				}
				else
				{
					SIMU_PWM_BreathWarningLED_Ctrl();
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
				
				if(((goStraightStatus == ctrlParasPtr->agvStatus) && (0 != ctrlParasPtr->FSflag)) || (gSslow == ctrlParasPtr->agvStatus))
				{
					//Show_Infomation();
					//show_Excel_Analysis_Info();
				}
				//else if((0 != ctrlParasPtr->BSflag) && (backStatus == ctrlParasPtr->agvStatus))
				else if((backStatus == ctrlParasPtr->agvStatus) || (bSslow == ctrlParasPtr->agvStatus))
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
				
		//Lcd_Handle();

		/*
		static u32 timRec = 0;
		
		if(0 == timRec)
		{
			ctrlParasPtr->rightHallCounter = 0;
			ctrlParasPtr->leftHallCounter = 0;
			timRec = SystemRunningTime;
			set_duty_Com(100, 100);
		}
		else
		{
			if(SystemRunningTime - timRec >= 10000 * 60)
			{
				printf("leftH = %d, rightH = %d\r\n", ctrlParasPtr->leftHallCounter, ctrlParasPtr->rightHallCounter);
				timRec = 0;
			}
			
		}
		*/

		
	#if 1
		/*
		static s8 flag = 0;
		
		ECV_Ctrl_Func_W(WECV_Str_Ptr);
		WECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);

		if(0 == Return_SW_RF)
		{
			Delay_ms(20);
			
			if(0 == Return_SW_RF)
			{
				while(0 == Return_SW_RF);

				if(0 == flag)
				{
					WECV_Str_Ptr->Dir = ECV_DOWN;
					printf("ECV_DOWN\r\n");
					flag = 1;
				}
				else if(1 == flag)
				{
					WECV_Str_Ptr->Dir = ECV_UP;
					printf("ECV_UP\r\n");
					flag = 0;
				}
			}
			
		}

		
		*/
	#else
		
		static s8 	flag 	= 0;
		static u32 	timRec 	= 0;
		
		ECV_Ctrl_Func_F(FECV_Str_Ptr);
		FECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
		
		if(0 == Return_SW_RF)
		{
			Delay_ms(20);
			
			if(0 == Return_SW_RF)
			{
				while(0 == Return_SW_RF);

				if(0 == flag)
				{
					FECV_Str_Ptr->Dir = ECV_UP;
					
					flag = 1;
				}
				else if(1 == flag)
				{
					FECV_Str_Ptr->Dir = ECV_DOWN;
					
					flag = 0;
				}
			}
			
		}

		if(ECV_STOP != FECV_Str_Ptr->Dir)
		{
			if(0 == timRec)
			{
				timRec = SystemRunningTime;
			}
			else
			{
				if(SystemRunningTime - timRec >= 30000)
				{
					FECV_Str_Ptr->Dir = ECV_STOP;
					timRec = 0;
				}
			}
		}
		
	#endif
		/*
		static u8 flag = 0;
		Ecv_Para temp;

		ECV_Ctrl_Func(BECV_Str_Ptr);
		
		BECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
		
		if(0 == Return_SW_RF)
		{
			Delay_ms(20);
			
			if(0 == Return_SW_RF)
			{
				while(0 == Return_SW_RF);
				
				if(0 == flag)
				{
					temp.Dir 				= ECV_DOWN;
					temp.EcvHallCountCmp 	= 100;
					temp.EcvSpeed			= 100;
					temp.HallCountMode		= ECV_USE_HALL_COUNT_MODE_ENABLE;

					BECV_Str_Ptr->ECV_SetPara(&temp);
					
					flag = 1;
				}
				else if(1 == flag)
				{
					temp.Dir 				= ECV_UP;
					temp.EcvHallCountCmp 	= 100;
					temp.EcvSpeed			= 100;
					temp.HallCountMode		= ECV_USE_HALL_COUNT_MODE_ENABLE;
					
					BECV_Str_Ptr->ECV_SetPara(&temp);
					
					flag = 0;
				}
			}
		}
		
		//BECV_Str_Ptr->ECV_UpDownFunc(ECV_DOWN);
		//BECV_Str_Ptr->ECV_SetSpeedFunc(100);

		if(ECV_COMPLETE == BECV_Str_Ptr->UseStatus)
		{
			BECV_Str_Ptr->UseStatus = ECV_UNUSED;
		}
		*/

		ManualModeEcvCtrlFunc();
		
		#endif
		
	}
	
	
}





