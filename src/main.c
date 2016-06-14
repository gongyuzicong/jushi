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

void SystemInit(void)
{	
	//keyConfig();
	BufferOpts_Init();
	Delay_Init(72);
	Timer2_Init(10000, 7199);	// 1s
	TIM3_Init(65535, 35999);		
	Timer4_Init(9, 719);		// 1ms
	Motion_Ctrl_Init();
	Pwm_Init();
	SPI_Initial();
	//NFR24L01_Init();
	//ExtiInit();
	Magn_Sensor_Init();
	Zigbee_Init();
	
	errorInfo.errorType = errorTypeBegin;
	errorInfo.errorData = 0;
}



int main(void)
{
	//TIMx_PwmOpts_Struct TIM3_PWM;
	//int cir = 1, cir2 = 0;
	static u8 addGearFlag = 0;
	
	CB_RCC_Config();	/*ÅäÖÃÏµÍ³Ê±ÖÓ*/
	CB_GPIO_Config();	/*ÅäÖÃGPIO¿Ú*/
	CB_USART_Config();	/*ÅäÖÃUSART*/
	SystemInit();
	
	printf("Start\r\n");
	
	AGV_Walking_Test();
	
	ctrlParasPtr->gear = 5;
	
	
	while(1)
	{
		
		Zigbee_Data_Scan();
		
		if(1 == Zigbee_Ptr->recvValidData)
		{
			Zigbee_Ptr->recvValidData = 0;

			if((0x0001 == Zigbee_Ptr->recvValidData) || (0x0002 == Zigbee_Ptr->recvValidData))
			{
				ctrlParasPtr->goalStation = STATION_1AND2_RFID;
			}
			else if((0x0003 == Zigbee_Ptr->recvValidData) || (0x0004 == Zigbee_Ptr->recvValidData))
			{
				ctrlParasPtr->goalStation = STATION_3AND4_RFID;
			}
			else if((0x0005 == Zigbee_Ptr->recvValidData) || (0x0006 == Zigbee_Ptr->recvValidData))
			{
				ctrlParasPtr->goalStation = STATION_5AND6_RFID;
			}
			else if((0x0007 == Zigbee_Ptr->recvValidData) || (0x0008 == Zigbee_Ptr->recvValidData))
			{
				ctrlParasPtr->goalStation = STATION_7AND8_RFID;
			}
			else if((0x0009 == Zigbee_Ptr->recvValidData) || (0x000a == Zigbee_Ptr->recvValidData))
			{
				ctrlParasPtr->goalStation = STATION_9AND10_RFID;
			}
			else
			{
				ctrlParasPtr->goalStation = 0x0000;
			}
			
		}

		if(1 == RFID_Info_Ptr->updateFlag)
		{
			RFID_Info_Ptr->updateFlag = 0;
			printf("data = %08x\r\n", RFID_Info_Ptr->rfidData);

			if(ctrlParasPtr->goalStation == RFID_Info_Ptr->rfidData)
			{
				
				
				
			}
		}

		//Walking_Mode_Control();
		
		#if 1
		
		if(testStatus == ctrlParasPtr->agvStatus)
		{
			//printf("l=%d,r=%d\r\n", ctrlParasPtr->leftHallIntervalTime, ctrlParasPtr->rightHallIntervalTime);
			AVG_Calu_Program();
			AGV_Correct_1();
		}
		else
		{
			
			Magn_Sensor_Scan();
			
			//AGV_Walking();

			
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				#if 0
				if(1 == addGearFlag)
				{
					addGearFlag = 0;
					ctrlParasPtr->gear++;
				}

				if(ctrlParasPtr->gear > 10)
				{
					CleanAllSpeed();
					CHANGE_TO_STOP_MODE();
				}
				#endif
				
				AGV_Correct_gS_5(ctrlParasPtr->gear);
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				//addGearFlag = 1;
				AGV_Correct_back_4(ctrlParasPtr->gear);
			}
			else if(cirLeft == ctrlParasPtr->agvStatus)
			{
				walking_cirLeft();
			}
			else if(cirRight == ctrlParasPtr->agvStatus)
			{
				
			}
			
			AGV_Change_Mode();
			
			//LeftOrRight_Counter();

			if(FMSDS_Pre_Ptr->MSD_Hex != FMSDS_Ptr->MSD_Hex)
			{
				Show_Infomation();
			}
		}
		
		#endif
		
	}

	
}





