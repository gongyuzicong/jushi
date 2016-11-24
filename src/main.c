#include "main_include.h"
#include "cfg_rcc.h"
#include "cfg_gpio.h"
#include "cfg_usart.h"
#include "timer_opts.h"
#include "key_opts.h"
#include "buffer.h"
#include "pwm_opts.h"
#include "motion_control.h"
#include "exti_opts.h"
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
#include "battery.h"
#include "agv_debug.h"
#include "buffer.h"

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
	
	Battery_Init();

	LCD_INIT();
	
	LED_Init();
	
	#if USE_CIRCLE_INFO_RECODER
	circle_recoder_init();
	#endif

	#if USE_NEW_RTC
	RTC_PCF8563_Init();
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
		
	
	FECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
	
	//BECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
	
	WECV_Str_Ptr->ECV_PowerOnOffFunc(ECV_POWER_ON);
	
	
	MOTOR_POWER_ON();
	//MOTOR_POWER_OFF();
	
	Machine_Arm_Init3();

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
		IWDG_RELOAD();
		#endif
		
		ProtectFunc();							// 行人保护

		ctrlParasPtr->AccCtrl.CtrlFunc(&(ctrlParasPtr->AccCtrl));
		
		Read_RTC_Data();						// 年月日
		
		Lcd_Handle();							// 小屏幕接收操作函数
		
		//SIMU_PWM_BreathBoardLED_Ctrl();		// 模拟PWM控制主控板LED呼吸灯
		
		Scan_Weight_Func();						// 扫描称重模块数据
		
		LCD_Page_Report();						// 小屏幕各个页面数据显示控制
		
		Machinearm_Control_Handle();			// 取纱臂动作控制函数

		LED_Status_Handle();

		Motor_Count_Cmp_Func();					// 

		BatteryInfoPtr->Scan_Battery(BatteryInfoPtr);
		
		WarningLedCtrlPtr->twinkleCtrlFunc(WarningLedCtrlPtr);	// 警告灯闪烁控制
		FECV_Str_Ptr->ECV_Ctrl_Function(FECV_Str_Ptr);			// 前电缸控制
		BECV_Str_Ptr->ECV_Ctrl_Function(BECV_Str_Ptr);			// 后电缸控制
		//WECV_Str_Ptr->ECV_Ctrl_Function(WECV_Str_Ptr);		// 直行辅助轮电缸控制
		
		
		/****控制逻辑部分 start****/
		if(TestMode == ctrlParasPtr->agvWalkingMode)
		{
			//printf("l=%d,r=%d\r\n", ctrlParasPtr->leftHallIntervalTime, ctrlParasPtr->rightHallIntervalTime);
			//AVG_Calu_Program();
			AGV_Correct_2();
		}
		else if(ManualMode == ctrlParasPtr->agvWalkingMode)
		{
			
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
		else if(AutomaticMode == ctrlParasPtr->agvWalkingMode)
		{
			Magn_Sensor_Scan();				// 磁传感器数据处理

			ZB_Data_Receive_handle();		// ZigBee数据接收函数

			ZigbeeRecv_Simu();				// 模拟
			
			ZB_Data_Analysis();				// ZigBee数据接收处理函数
			
			CrossRoad_Count();				// 磁条十字交叉路口的计算
			
			Get_Zigbee_Info_From_Buf();		// 从队列当中取出接收到的ZigBee信息
			
			RFID_Goal_Node_Analy();			// 分析哪个RFID点转
			
			Walking_Step_Controler();		// 整个大逻辑的控制
			
			/************小车驱动轮控制************/
			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_End))
			{
				AGV_Walking_Opt();
			}
			
			#if 0
			// 在原点待机时自动回正
			//if(ControlCenter == ctrlParasPtr->goalStation)
			if((1 == ctrlParasPtr->originFlag) && (ORIGIN_STATION_NODE != STATION_LM))
			{
				//SIMU_PWM_BreathWarningLED_Ctrl();
				
				ctrlParasPtr->cirDuty = 8;
				Origin_PatCtrl(ctrlParasPtr->cirDuty);
			}
			#endif
			
			//SIMU_PWM_BreathWarningLED_Ctrl();
			//LeftOrRight_Counter();
			ZigbeeResendInfo_Ptr->resendCtrlFunc();
			BuzzerCtrlPtr->buzzerCtrlFunc();
			Clean_Motor_Hall_Counter();
			//AGV_Debug_Func();
			
			// 小车调试信息的打印
			if((hexF != FMSDS_Ptr->MSD_Hex) || (hexR != RMSDS_Ptr->MSD_Hex))
			{
				hexF = FMSDS_Ptr->MSD_Hex;
				hexR = RMSDS_Ptr->MSD_Hex;
				
			#if 0
				
				//if(((goStraightStatus == ctrlParasPtr->agvStatus) && (0 != ctrlParasPtr->FSflag)) || (gSslow == ctrlParasPtr->agvStatus))
				if((goStraightStatus == ctrlParasPtr->agvStatus) || (gSslow == ctrlParasPtr->agvStatus))
				{
					Show_Infomation();
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

			//show_info();
		}

		#else

		//ZB_Data_Receive_handle();				// 
		//set_duty(100, 100);
		//Get_Voltage();
		if(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_UnRespond && Return_SW_RR_Respond)
		{
			
			Delay_ms(20);			
			if(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_UnRespond && Return_SW_RR_Respond)
			{
				while(Return_SW_RR_Respond);
				//ctrlParasPtr->cirDuty = 13;
				ctrlParasPtr->walkingstep = step_bVeer;
				CHANGE_TO_CIR_LEFT_MODE();
				//CHANGE_TO_CIR_RIGHT_MODE();
			}
			
		}
		
		Walking_Step_Controler();		// 整个大逻辑的控制
		AGV_Walking_Opt();
		Clean_Motor_Hall_Counter();
		#endif
		
	}
	
	
}





