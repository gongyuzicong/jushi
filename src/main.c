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
	My_I2C_Init();
	//MPU6050_init();
	
	errorInfo.errorType = errorTypeBegin;
	errorInfo.errorData = 0;
}



int main(void)
{
	//TIMx_PwmOpts_Struct TIM3_PWM;
	//int cir = 0, ayDec = 0;
	static u16 hexF = 0, hexR = 0;	
	
	CB_RCC_Config();	/*ÅäÖÃÏµÍ³Ê±ÖÓ*/
	CB_GPIO_Config();	/*ÅäÖÃGPIO¿Ú*/
	CB_USART_Config();	/*ÅäÖÃUSART*/
	SystemInit();
	
	//MPU6050_Data_init3();
	//ECV_POWER_ON();
	FECV_DOWN();
	BECV_DOWN();
	//WECV_DOWN();
	Delay_ns(3);
	//MOTOR_POWER_OFF();
	//FECV_UP();
	//FECV_DOWN();
	//BECV_UP();
	//Delay_ns(5);
	//BECV_DOWN();
	
	//FECV_DOWN();
	//Delay_ns(20);
	//FECV_DOWN();
	//Delay_ns(20);
	//Delay_ns(3);
	//WECV_UP();
	//Delay_ns(7);
	//WECV_DOWN();
	//Delay_ns(5);
	ECV_POWER_OFF();
	MOTOR_POWER_ON();
	//MOTOR_POWER_OFF();
	//AGV_Walking_Test();

	printf("Start\r\n");
	
	ctrlParasPtr->gear = 10;
	CHANGE_TO_GO_STRAIGHT_MODE();
	Zigbee_Ptr->recvValidDataFlag = 1;
	Zigbee_Ptr->recvId = 0x0007;
	
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
			
			Magn_Sensor_Scan();

			Receive_handle();
			//Zigbee_Data_Scan();
			
			if(CMD_Flag_Ptr->cmdFlag == GoodReq)
			{
				CMD_Flag_Ptr->cmdFlag = NcNone;
				CHANGE_TO_GO_STRAIGHT_MODE();
				Zigbee_Ptr->recvValidDataFlag = 1;
				Zigbee_Ptr->recvId = 0x0007;
			}
			
			CrossRoad_Count();
			
			//Hall_Count();
			
			RFID_Goal_Node_Analy();
			
			Walking_Step_Controler();
			
			//AGV_Walking();
			
			
			if((FMSDS_Ptr->AgvMSLocation >= Agv_MS_Left_End) && (FMSDS_Ptr->AgvMSLocation <= Agv_MS_Right_End))
			{
				
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
					walking_cirLeft(ctrlParasPtr->gear);
				}
				else if(cirRight == ctrlParasPtr->agvStatus)
				{
					walking_cirRight(ctrlParasPtr->gear);
				}
				else if(gSslow == ctrlParasPtr->agvStatus)
				{
					gS_slow(ctrlParasPtr->gear);
				}
				else if(bSslow == ctrlParasPtr->agvStatus)
				{
					back_slow(ctrlParasPtr->gear);
				}
				
			}
			
			//AGV_Change_Mode();
			AGV_Proc();
			
			
			//LeftOrRight_Counter();

			if((hexF != FMSDS_Ptr->MSD_Hex) || (hexR != RMSDS_Ptr->MSD_Hex))
			//if(0)
			{
				hexF = FMSDS_Ptr->MSD_Hex;
				hexR = RMSDS_Ptr->MSD_Hex;
				
				if((goStraightStatus == ctrlParasPtr->agvStatus) && (0 != ctrlParasPtr->FSflag))
				{
					//Show_Infomation();
					show_Excel_Analysis_Info();
				}
				else if((0 != ctrlParasPtr->BSflag) && (backStatus == ctrlParasPtr->agvStatus))
				{
					//Show_Infomation();
					show_Excel_Analysis_Info();
				}
				
			}
			
			
		}
		
		#endif
		
	}

	
}





