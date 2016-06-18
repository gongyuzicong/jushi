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

void SystemInit(void)
{	
	//keyConfig();
	BufferOpts_Init();
	Delay_Init(72);
	Timer2_Init(10000, 7199);	// 1s
	TIM3_Init(65535, 35999);		
	Timer4_Init(9, 719);		// 0.1ms
	//Timer5_Init(9, 719);
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
	int cir = 1, cir2 = 0;
	//static u8 addGearFlag = 0;
	
	CB_RCC_Config();	/*ÅäÖÃÏµÍ³Ê±ÖÓ*/
	CB_GPIO_Config();	/*ÅäÖÃGPIO¿Ú*/
	CB_USART_Config();	/*ÅäÖÃUSART*/
	SystemInit();
	
	printf("Start\r\n");
	
	//ECV_POWER_ON();
	//FECV_DOWN();
	//BECV_DOWN();
	//Delay_ns(3);
	ECV_POWER_OFF();
	MOTOR_POWER_ON();
	//AGV_Walking_Test();
	
	
	while(1)
	{
		
		#if 1
		
		if(testStatus == ctrlParasPtr->agvStatus)
		{
			//printf("l=%d,r=%d\r\n", ctrlParasPtr->leftHallIntervalTime, ctrlParasPtr->rightHallIntervalTime);
			AVG_Calu_Program();
			AGV_Correct_1();
		}
		else
		{
			
			#if 1
			
			if(step_stop == ctrlParasPtr->walkingstep)
			{
				if(Zigbee_Ptr->recvId < 0x0a)
				{
					Zigbee_Ptr->recvValidDataFlag = 1;
					Zigbee_Ptr->recvId++;

					if(0x02 == Zigbee_Ptr->recvId)
					{
						Zigbee_Ptr->recvId = 0x03;
					}
					ctrlParasPtr->walkingstep = step_gS;
					CHANGE_TO_GO_STRAIGHT_MODE();
				}
				else
				{
					Zigbee_Ptr->recvId = 0x03;
				}
			}

			#else
			
			if(1 == Zigbee_Ptr->recvValidDataFlag)
			{
				Zigbee_Ptr->recvValidDataFlag = 0;
				
				ctrlParasPtr->gear = 5;
				ctrlParasPtr->walkingstep = step_gS;
				CHANGE_TO_GO_STRAIGHT_MODE();
				
			}
			
			#endif
			
			
			Magn_Sensor_Scan();
			
			//Zigbee_Data_Scan();
			
			RFID_Node_Analy();
			
			Walking_Step_Controler();
			
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
				
				AGV_Correct_gS_7(ctrlParasPtr->gear);
				
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				//addGearFlag = 1;
				AGV_Correct_back_5(ctrlParasPtr->gear);
			}
			else if(cirLeft == ctrlParasPtr->agvStatus)
			{
				walking_cirLeft(ctrlParasPtr->gear);
			}
			else if(cirRight == ctrlParasPtr->agvStatus)
			{
				walking_cirRight(ctrlParasPtr->gear);
			}
			
			//AGV_Change_Mode();
			
			//LeftOrRight_Counter();

			if(FMSDS_Pre_Ptr->MSD_Hex != FMSDS_Ptr->MSD_Hex)
			{
				Show_Infomation();
			}

			
			
		}
		
		#endif
		
	}

	
}





