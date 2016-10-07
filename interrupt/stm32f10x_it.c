/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
//#include "common_include.h"
//#include "recv_from_fyt_opts.h"
#include "buffer.h"
#include "cfg_usart.h"
//#include "cfg_can.h"
#include "cfg_gpio.h"
#include "key_opts.h"
#include "pwm_opts.h"
#include "stm32f10x_exti.h"
#include "nrf24l01_opts.h"
#include "motion_control.h"
#include "magn_sensor.h"
#include "zigbee.h"
#include "lcd.h"
#include "circle_recoder.h"
#include "ecv_control.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : NMIException
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMIException(void)
{
}

/*******************************************************************************
* Function Name  : HardFaultException
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFaultException(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManageException
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManageException(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFaultException
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFaultException(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFaultException
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFaultException(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : DebugMonitor
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMonitor(void)
{
}

/*******************************************************************************
* Function Name  : SVCHandler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVCHandler(void)
{
}

/*******************************************************************************
* Function Name  : PendSVC
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSVC(void)
{
}

/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTickHandler(void)
{
}

/*******************************************************************************
* Function Name  : WWDG_IRQHandler
* Description    : This function handles WWDG interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : PVD_IRQHandler
* Description    : This function handles PVD interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PVD_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TAMPER_IRQHandler
* Description    : This function handles Tamper interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TAMPER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RTC_IRQHandler
* Description    : This function handles RTC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : FLASH_IRQHandler
* Description    : This function handles Flash interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RCC_IRQHandler
* Description    : This function handles RCC interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : This function handles External interrupt Line 0 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI0_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI1_IRQHandler
* Description    : This function handles External interrupt Line 1 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI1_IRQHandler(void)
{
	printf("pc1exti\r\n");
	//EXTI->PR = ((u32)0x00004);
	EXTI_ClearITPendingBit(EXTI_Line1);
}

/*******************************************************************************
* Function Name  : EXTI2_IRQHandler
* Description    : This function handles External interrupt Line 2 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		//printf("pd2exti\r\n");
		//EXTI->PR = ((u32)0x00004);
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

/*******************************************************************************
* Function Name  : EXTI3_IRQHandler
* Description    : This function handles External interrupt Line 3 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI4_IRQHandler
* Description    : This function handles External interrupt Line 4 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI4_IRQHandler(void)
{
	
#if 1
	// E1-HALL
	if (((EXTI->PR & ((u32)0x00010)) != 0) && ((EXTI->IMR & ((u32)0x00010)) != 0))
	{
		if(1 == ECV1_HALL_FLAG)
		{
			ECV1_HALL++;
			//printf("FECV_HALL = %d\r\n", ECV1_HALL);
		}
		else
		{
			ECV1_HALL = 0;
		}

		EXTI->PR = ((u32)0x00010);
	}


#else

	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line8);
	}

#endif
	
}

/*******************************************************************************
* Function Name  : DMA1_Channel1_IRQHandler
* Description    : This function handles DMA1 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel2_IRQHandler
* Description    : This function handles DMA1 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel3_IRQHandler
* Description    : This function handles DMA1 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel4_IRQHandler
* Description    : This function handles DMA1 Channel 4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel5_IRQHandler
* Description    : This function handles DMA1 Channel 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel6_IRQHandler
* Description    : This function handles DMA1 Channel 6 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel7_IRQHandler
* Description    : This function handles DMA1 Channel 7 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : This function handles ADC1 and ADC2 global interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC1_2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USB_HP_CAN_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{
	
	if(CAN->TSR & (0x01 << 0))
	{
		
		xBitOn(CAN->TSR, 0);
	}
	else if(CAN->TSR & (0x01 << 8))
	{
		
		xBitOn(CAN->TSR, 8);
	}
	else if(CAN->TSR & (0x01 << 16))
	{
		
		xBitOn(CAN->TSR, 16);
	}

	//清除中断标志位
	
}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
	CAN->RF0R = ((u32)0x00000020);
}

/*******************************************************************************
* Function Name  : CAN_RX1_IRQHandler
* Description    : This function handles CAN RX1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_RX1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : CAN_SCE_IRQHandler
* Description    : This function handles CAN SCE interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_SCE_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{

	#if 1
	// E2-HALL
	if (((EXTI->PR & ((u32)0x00100)) != 0) && ((EXTI->IMR & ((u32)0x00100)) != 0))
	{
		
		if(1 == ECV2_HALL_FLAG)
		{
			ECV2_HALL++;
			//printf("BECV_HALL = %d\r\n", ECV2_HALL);
		}
		else
		{
			ECV2_HALL = 0;
		}
		
		EXTI->PR = ((u32)0x00100);
	}
	// E3-HALL
	if (((EXTI->PR & ((u32)0x00200)) != 0) && ((EXTI->IMR & ((u32)0x00200)) != 0))
	{
		
		if(1 == ECV3_HALL_FLAG)
		{
			ECV3_HALL++;
			//printf("WECV_HALL = %d\r\n", ECV3_HALL);
		}
		else
		{
			ECV3_HALL = 0;
		}
		
		EXTI->PR = ((u32)0x00200);
	}

	#else

	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
	
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line9);
	}

	#endif
}

/*******************************************************************************
* Function Name  : TIM1_BRK_IRQHandler
* Description    : This function handles TIM1 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_UP_IRQHandler
* Description    : This function handles TIM1 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_UP_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_TRG_COM_IRQHandler
* Description    : This function handles TIM1 Trigger and commutation interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_CC_IRQHandler
* Description    : This function handles TIM1 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
	
	if(TIM2->SR & 0x0001)	// 溢出中断
	{
		//PCout(5) = ~(PCin(5));
		
	}
	
	TIM2->SR &= ~(1 << 0);	// 清除中断标志位

}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles TIM3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{
	
	vu16  capture = 0;			/* 当前捕获计数值局部变量 */
	printf("SR = %x\r\n", TIM3->SR);
	/* 
	*	TIM2 时钟 = 72 MHz, 分频数 = 7299 + 1, TIM2 counter clock = 10KHz
	*	CC1 更新率 = TIM2 counter clock / CCRx_Val
	*/
	
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		//GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4))); 	
		//PA_PIN_NOT(4);
		/* 读出当前计数值 */
		capture = TIM_GetCapture1(TIM3);
		//printf("capture1 = %d\r\n", capture);
		/* 根据当前计数值更新输出捕获寄存器 */
		TIM_SetCompare1(TIM3, capture + 40000);

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
		//GPIO_WriteBit(GPIOA, GPIO_Pin_5, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)));
		//PA_PIN_NOT(5);
		capture = TIM_GetCapture2(TIM3);
		//printf("capture2 = %d\r\n", capture);
		TIM_SetCompare2(TIM3, capture + 20000);

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
	{
		//GPIO_WriteBit(GPIOA, GPIO_Pin_6, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6)));
		//PA_PIN_NOT(6);
		capture = TIM_GetCapture3(TIM3);
		//printf("capture3 = %d\r\n", capture);
		TIM_SetCompare3(TIM3, capture + 10000);

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
	{
		//GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7)));
		//PA_PIN_NOT(7);
		capture = TIM_GetCapture4(TIM3);
		//printf("capture4 = %d\r\n", capture);
		TIM_SetCompare4(TIM3, capture + 5000);

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
	}
	else if(TIM3->SR & 0x0001)
	{
		//capture = TIM_GetCapture4(TIM3);
		//printf("capture5 = %d\r\n", capture);
		//printf("SR = %x\r\n", TIM3->SR);
		TIM3->SR &= ~(1 << 0);	// 清除中断标志位
	}

	
	
}

/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
	
	// 1ms
	if(TIM4->SR & 0x0001)	// 溢出中断
	{
		//SystemRunningTime++;
		
	}
	//printf("%d\r\n", SystemRunningTime);
	TIM4->SR &= ~(1 << 0);	// 清除中断标志位
}

/*******************************************************************************
* Function Name  : I2C1_EV_IRQHandler
* Description    : This function handles I2C1 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_EV_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C1_ER_IRQHandler
* Description    : This function handles I2C1 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_ER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C2_EV_IRQHandler
* Description    : This function handles I2C2 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_EV_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C2_ER_IRQHandler
* Description    : This function handles I2C2 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_ER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI1_IRQHandler
* Description    : This function handles SPI1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI2_IRQHandler
* Description    : This function handles SPI2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
	/* 判断中断类型 */
	if(0 != (USART1->SR & (0x01 << 6)))	
	{
		xBitOff(USART1->SR, 6);
		
	}	
	else if(0 != (USART1->SR & (0x01 << 5)))	// 判断是否为RXNE中断
	{
		u8 recvD = USART1_RECV_DATA;
		//printf("U1D = %x\r\n", recvD);
		
		UART1_REC(recvD);
	}
	
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
	

	if(0 != (USART2->SR & (0x01 << 6)))	
	{
		xBitOff(USART2->SR, 6);
		
	}	
	else if(0 != (USART2->SR & (0x01 << 5)))	// 判断是否为RXNE中断
	{
		u8 recvD = USART2_RECV_DATA;
		//printf("U2D = %x\r\n", recvD);
		
		//Protocol_analysis(recvD);
		UART2_REC_IRQ(recvD);
	}
	
}

/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
	if(0 != (USART3->SR & (0x01 << 6)))	
	{
		xBitOff(USART3->SR, 6);
		
	}	
	else if(0 != (USART3->SR & (0x01 << 5)))	// 判断是否为RXNE中断
	{
		u8 recvD = USART3_RECV_DATA;
		//printf("3recvD = %x\r\n", recvD);
		
		Recv_RFID_CrossRoad(recvD);
		
	}
}

/*******************************************************************************
* Function Name  : EXTI15_10_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	//static u8 lMotorCount = 0x00, rMotorCount = 0x00;
	static u32 lTimeRecode = 0x00, rTimeRecode = 0x00;
	
	#if 1
	if (((EXTI->PR & ((u32)0x04000)) != 0) && ((EXTI->IMR & ((u32)0x04000)) != 0))
	{
		ctrlParasPtr->rightHallIntervalTime = SystemRunningTime - rTimeRecode;

		if(1 == MOTOR1_HALL_COUNT_FLAG)		// right
		{
			if(0xFFFFFFFF == ctrlParasPtr->rightHallCounter)
			{
				ctrlParasPtr->rightHallCounter = 0;
			}
			else
			{
				ctrlParasPtr->rightHallCounter++;
			}
		}
		else
		{
			ctrlParasPtr->rightHallCounter = 0;
		}
		

		if(1 == ctrlParasPtr->CrossRoadHallCountFlag)
		{
			ctrlParasPtr->CrossRoadHallCountR++;
		}
		
		/*
		if(0 == ctrlParasPtr->rightHallIntervalTime)
		{
			printf("rtt = %d, rtr = %d\r\n", rtempTime, rTimeRecode);
		}
		*/
		rTimeRecode = SystemRunningTime;

		EXTI->PR = ((u32)0x04000);
	}	

	if (((EXTI->PR & ((u32)0x01000)) != 0) && ((EXTI->IMR & ((u32)0x01000)) != 0))
	{
		ctrlParasPtr->leftHallIntervalTime = SystemRunningTime - lTimeRecode;

		if(1 == MOTOR2_HALL_COUNT_FLAG)		// left
		{
			if(0xFFFFFFFF == ctrlParasPtr->leftHallCounter)
			{
				ctrlParasPtr->leftHallCounter = 0;
			}
			else
			{
				ctrlParasPtr->leftHallCounter++;
			}
		}
		else
		{
			ctrlParasPtr->rightHallCounter = 0;
		}
		

		if(1 == ctrlParasPtr->CrossRoadHallCountFlag)
		{
			ctrlParasPtr->CrossRoadHallCountL++;
		}
		/*
		if(0 == ctrlParasPtr->leftHallIntervalTime)
		{
			printf("ltt = %d, ltr = %d\r\n", ltempTime, lTimeRecode);
		}
		*/
		lTimeRecode = SystemRunningTime;

		EXTI->PR = ((u32)0x01000);
	}
	
	#else
	
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
	
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line12);
	}

	#endif
}

/*******************************************************************************
* Function Name  : RTCAlarm_IRQHandler
* Description    : This function handles RTC Alarm interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTCAlarm_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USBWakeUp_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBWakeUp_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_BRK_IRQHandler
* Description    : This function handles TIM8 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_BRK_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_UP_IRQHandler
* Description    : This function handles TIM8 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_UP_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_TRG_COM_IRQHandler
* Description    : This function handles TIM8 Trigger and commutation interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_CC_IRQHandler
* Description    : This function handles TIM8 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_CC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : ADC3_IRQHandler
* Description    : This function handles ADC3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : FSMC_IRQHandler
* Description    : This function handles FSMC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM5_IRQHandler
* Description    : This function handles TIM5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM5_IRQHandler(void)
{
	
	// 1ms
	if(TIM5->SR & 0x0001)	// 溢出中断
	{
		SystemRunningTime++;
		
		
	}
	//printf("%d\r\n", SystemRunningTime);
	TIM5->SR &= ~(1 << 0);	// 清除中断标志位
}


/*******************************************************************************
* Function Name  : SPI3_IRQHandler
* Description    : This function handles SPI3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : UART4_IRQHandler
* Description    : This function handles UART4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART4_IRQHandler(void)
{
	if(0 != (UART4->SR & (0x01 << 6)))	
	{
		xBitOff(UART4->SR, 6);
		
	}	
	else if(0 != (UART4->SR & (0x01 << 5)))	// 判断是否为RXNE中断
	{
		u8 recvD = UART4_RECV_DATA;
		//printf("U4D = %x\r\n", recvD);
		
		//UART1_REC(recvD);
		#if USE_CIRCLE_INFO_RECODER
		CircleInfo_CMD_Handle(recvD);
		#endif
	}
	
}

/*******************************************************************************
* Function Name  : UART5_IRQHandler
* Description    : This function handles UART5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART5_IRQHandler(void)
{
	if(0 != (UART5->SR & (0x01 << 6)))	
	{
		xBitOff(UART5->SR, 6);
		
	}	
	else if(0 != (UART5->SR & (0x01 << 5))) // 判断是否为RXNE中断
	{
		u8 recvD = UART5_RECV_DATA;
		//printf("U4D = %x\r\n", recvD);
		
		//UART1_REC(recvD);
	}
	
}


/*******************************************************************************
* Function Name  : TIM6_IRQHandler
* Description    : This function handles TIM6 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM6_IRQHandler(void)
{
	
	// 0.1ms
	if(TIM6->SR & 0x0001)	// 溢出中断
	{
		SystemRunningTime++;
		//PCout(5) = ~(PCin(5));
		
	}
	//printf("%d\r\n", SystemRunningTime);
	TIM6->SR &= ~(1 << 0);	// 清除中断标志位
}


/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : This function handles TIM7 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM7_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel1_IRQHandler
* Description    : This function handles DMA2 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel2_IRQHandler
* Description    : This function handles DMA2 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel3_IRQHandler
* Description    : This function handles DMA2 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel4_5_IRQHandler
* Description    : This function handles DMA2 Channel 4 and DMA2 Channel 5
*                  interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel4_5_IRQHandler(void)
{
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
