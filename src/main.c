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
	ADS1256_Init();
	LCD_INIT();
	
	Get_Weight_Offset_Data();
	
	errorInfo.errorType = errorTypeBegin;
	errorInfo.errorData = 0;

	
}



int main(void)
{
	//TIMx_PwmOpts_Struct TIM3_PWM;
	//int cir = 0, ayDec = 0;
	static u16 hexF = 0, hexR = 0;
	static Agv_MS_Location mslRecF = AgvInits, mslRecR = AgvInits;
	
	CB_RCC_Config();	/*ÅäÖÃÏµÍ³Ê±ÖÓ*/
	CB_GPIO_Config();	/*ÅäÖÃGPIO¿Ú*/
	CB_USART_Config();	/*ÅäÖÃUSART*/
	SystemInit();

	mslRecF = FMSDS_Ptr->AgvMSLocation;
	mslRecR = FMSDS_Ptr->AgvMSLocation;
	Warning_LED = 0;
	
	//MPU6050_Data_init3();
	ECV_POWER_ON();
	FECV_DOWN();
	BECV_DOWN();
	//WECV_DOWN();
	Delay_ns(3);
	FECV_STOP();
	BECV_UP();
	//Delay_ns(1);
	Delay_ms(700);
	BECV_STOP();
	MOTOR_POWER_ON();
	//MOTOR_POWER_OFF();
	//AGV_Walking_Test();
	Get_Weight_Offset_Data_One();
	
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

		ProtectFunc();
		Read_RTC_Data();
		Lcd_Handle();
		//SIMU_PWM_BreathBoardLED_Ctrl();
		Scan_Weight_Func();
		
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
		}
		else if(RFID_Setting_Mode == ctrlParasPtr->agvWalkingMode)
		{
			static u8 count = 0x00;
			
			MOTOR_POWER_ON();
			
			if(1 == RFID_Info_Ptr->noValide)
			{
				
			}
			
		}
		else if(AutomaticMode == ctrlParasPtr->agvWalkingMode)
		{
			Magn_Sensor_Scan();
			Receive_handle2();
			
			if(0 == ctrlParasPtr->start_origin_mode)
			//if(0)
			{
				
				if(gSslow == ctrlParasPtr->agvStatus)
				{
					gS_slow2(3);
				}
				else if(bSslow == ctrlParasPtr->agvStatus)
				{
					back_slow2(3);
				}
				
				startup_origin_Func();
				
			}
			else
			{
				//originP();
				
				CrossRoad_Count2();
				
				Get_Zigbee_Info_From_Buf();
				
				RFID_Goal_Node_Analy();
				
				Walking_Step_Controler();
				
				if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_End))
				{
					
				#if 0
					AGV_Walking();
				#else
					
					if(goStraightStatus == ctrlParasPtr->agvStatus)
					{
						
						AGV_Correct_gS_8ug(ctrlParasPtr->gear);
					}
					else if(backStatus == ctrlParasPtr->agvStatus)
					{
						
						AGV_Correct_back_ug(ctrlParasPtr->gear);
					}
					else if(cirLeft == ctrlParasPtr->agvStatus)
					{
						//walking_cirLeft(ctrlParasPtr->gear);
						walking_cir(ctrlParasPtr->cirDuty);
					}
					else if(cirRight == ctrlParasPtr->agvStatus)
					{
						//walking_cirRight(ctrlParasPtr->gear);
						walking_cir(ctrlParasPtr->cirDuty);
					}
					else if(gSslow == ctrlParasPtr->agvStatus)
					{
						gS_slow2(ctrlParasPtr->gear);
					}
					else if(bSslow == ctrlParasPtr->agvStatus)
					{
						back_slow2(ctrlParasPtr->gear);
					}
											
				#endif
					
				}
				
				//AGV_Change_Mode();
				
				//AGV_Proc();
				
				if(ControlCenter == ctrlParasPtr->goalStation)
				{
					//SIMU_PWM_BreathWarningLED_Ctrl();
					
					ctrlParasPtr->cirDuty = 8;
					Origin_PatCtrl(ctrlParasPtr->cirDuty);
					//AutoRunningFunc();
				}
				
				//LeftOrRight_Counter();
				
			}
			
			WarningLedCtrlPtr->twinkleCtrlFunc();
			ZigbeeResendInfo_Ptr->resendCtrlFunc();
			BuzzerCtrlPtr->buzzerCtrlFunc();
			
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
		MA_TEST();
				
		//Lcd_Handle();
		
		#endif
		
	}

	
}





