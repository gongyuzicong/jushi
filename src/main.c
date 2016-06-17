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
	
	errorInfo.errorType = errorTypeBegin;
	errorInfo.errorData = 0;
}



int main(void)
{
	//TIMx_PwmOpts_Struct TIM3_PWM;
	//int cir = 1, cir2 = 0;
	static u8 addGearFlag = 0;
	
	CB_RCC_Config();	/*����ϵͳʱ��*/
	CB_GPIO_Config();	/*����GPIO��*/
	CB_USART_Config();	/*����USART*/
	SystemInit();
	
	printf("Start\r\n");
	MOTOR_POWER_ON();
	//ECV_POWER_ON();
	//FECV_DOWN();
	//BECV_DOWN();
	//Delay_ms(5000);
	//ECV_POWER_OFF();
	
	AGV_Walking_Test();
	
	ctrlParasPtr->gear = 7;

	if(1 == Zigbee_Ptr->recvValidDataFlag)
	{
		Zigbee_Ptr->recvValidDataFlag = 0;
		
		ctrlParasPtr->gear = 7;
		ctrlParasPtr->walkingstep = step_gS;
		CHANGE_TO_GO_STRAIGHT_MODE();
		
	}
	
	while(1)
	{
		
		if(testStatus == ctrlParasPtr->agvStatus)
		{
			//printf("l=%d,r=%d\r\n", ctrlParasPtr->leftHallIntervalTime, ctrlParasPtr->rightHallIntervalTime);
			AVG_Calu_Program();
			AGV_Correct_2(ctrlParasPtr->gear);
		}
		else
		{
			
			Magn_Sensor_Scan();
			
			//AGV_Walking();

			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{	
				
				AGV_Correct_gS_7(ctrlParasPtr->gear);
				//AGV_Correct_gS_test(ctrlParasPtr->gear);
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				
				AGV_Correct_back_5(ctrlParasPtr->gear);
			}
			
			//AGV_Change_Mode();
			
			//LeftOrRight_Counter();

			if(FMSDS_Pre_Ptr->MSD_Hex != FMSDS_Ptr->MSD_Hex)
			{
				Show_Infomation();
			}
		}
		
		
	}

	
}





