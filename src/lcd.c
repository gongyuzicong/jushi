#include "lcd.h"
#include "timer_opts.h"
#include "motion_control.h"
#include "fiberglas.h"
#include "cfg_gpio.h"
#include "fiberglas.h"
#include "ecv_control.h"
#include "fiberglas.h"
#include "battery.h"

//��ӵ�ȫ�ֱ���//
u8 receive_buf[150];
u8 lcd_receive_count=0;
u8 flag_recok=0;

u16 task_state[10]={11,22,33,44,55,66,77,88,99,100};

LCD_Info_Str LCD_Info;
LCD_Info_Str_P LCD_Info_Ptr = &LCD_Info;


char itoc(int in)
{
	char str[10]="0123456789";	
	return str[in];	
}



/*
mian_state���ʮ�����ݷֱ�Ϊ
Ӧ��ɣ��ܸ������ϸ񣬳��أ�����
�������������ʣ������ʣ��ϸ��ʣ�Ч��
*/


void Uart_Send_Char(u8 data)
{
	//u16 timeout = U16_MAX;
	u32 timeout = 0;
	USART_SendData(USART1, data);
	
#if 0
	
	//while (!(USART1->SR & USART_FLAG_TXE));
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//�ȴ����ڷ������
	
#else
	
	timeout = SystemRunningTime;
	while(!(USART1->SR & USART_FLAG_TXE))
	{
		if(SystemRunningTime - timeout > 100)
		{
			printf("timeout\r\n");
			break;
		}
	}

#endif
	
}


void Uart_Send_Str(u8* str)
{
	for(;*str!='\0';str++)
	{
		Uart_Send_Char(*str);
	}
}

//��������//
void Set_date(u8 y,u8 m,u8 d)
{
	u8 temp1,temp2;
	Uart_Send_Str("main_date.date=\"20");
	temp1=(y/10)+0x30;
	temp2=(y%10)+0x30;
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char('-');
	temp1=(m/10)+0x30;
	temp2=(m%10)+0x30;
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);	
	Uart_Send_Char('-');
	temp1=(d/10)+0x30;
	temp2=(d%10)+0x30;
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char('\"');
	Uart_Send_Char(0x0d);
}

//����ʱ��//
void Set_time(u8 h,u8 m,u8 s)
{
	u8 temp1,temp2;
	Uart_Send_Str("main_time.time=\"");
	temp1=(h/10)+0x30;
	temp2=(h%10)+0x30;
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char(':');
	temp1=(m/10)+0x30;
	temp2=(m%10)+0x30;
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);	
	Uart_Send_Char(':');
	temp1=(s/10)+0x30;
	temp2=(s%10)+0x30;
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char('\"');
	Uart_Send_Char(0x0d);	
}

//���õ���//
void LCD_Report_Battery(u8* str)
{
	
	Uart_Send_Str(str);
	Uart_Send_Char(BatteryInfoPtr->Battery_H);
	Uart_Send_Char(BatteryInfoPtr->Battery_L);
	Uart_Send_Char(0x0d);
		
}

void LCD_Report_MainPage_Battery(void)
{
	LCD_Report_Battery("main_battery.value=");
	
}

void LCD_Report_WeightPage_Battery(void)
{
	LCD_Report_Battery("scale_battery.value=");
	
}

void LCD_Report_SystemPage_Battery(void)
{
	LCD_Report_Battery("systemSetup_battery.value=");
	
}

void LCD_Report_ManualPage_Battery(void)
{
	LCD_Report_Battery("manualControl_battery.value=");
	
}

void LCD_Report_HelpPage_Battery(void)
{
	LCD_Report_Battery("help_battery.value=");
	
}

void LCD_Report_AuthorityPage_Battery(void)
{
	LCD_Report_Battery("authority_battery.value=");
	
}

void LCD_Report_RfidPage_Battery(void)
{
	LCD_Report_Battery("rfidSetup_battery.value=");
	
}

void LCD_Report_TaskPage_Battery(void)
{
	LCD_Report_Battery("task_battery.value=");
	
}
//������ҳ����״̬//
void Set_task_state(u16 *buf)
{
	u8 i,temp=0;
	u8 temp1=0,temp2=0,temp3=0,temp4=0;
	
	for(i=0;i<9;i++)
	{
		temp=0x30+i+1;
		buf[i]=buf[i]%10000;
		temp1=buf[i]/1000+0x30;
		temp2=(buf[i]%1000)/100+0x30;
		temp3=(buf[i]%100)/10+0x30;
		temp4=buf[i]%10+0x30;
		Uart_Send_Str("task_L");
		Uart_Send_Char(temp);
		Uart_Send_Str("a.text=\"");
		Uart_Send_Char(temp1);
		Uart_Send_Char(temp2);
		Uart_Send_Char(temp3);
		Uart_Send_Char(temp4);
		Uart_Send_Char('\"');
		Uart_Send_Char(0x0d);		
	}
	
	buf[i]=buf[i]%10000;
	temp1=buf[i]/1000+0x30;
	temp2=(buf[i]%1000)/100+0x30;
	temp3=(buf[i]%100)/10+0x30;
	temp4=buf[i]%10+0x30;
	Uart_Send_Str("task_L");
	Uart_Send_Char('1');
	Uart_Send_Char('0');	
	Uart_Send_Str("a.text=\"");
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char(temp3);
	Uart_Send_Char(temp4);
	Uart_Send_Char('\"');
	Uart_Send_Char(0x0d);	
}

//���³���ҳ��//zΪ��������,xΪС������//
void Set_scale_weight(u8 z,u8 x)
{
	u8 temp1=0,temp2=0;
	temp1=0x30+(z/10);
	temp2=0x30+(z%10);
	Uart_Send_Str("scale_weight.text=\"");
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char('.');
	temp1=0x30+(x/10);
	temp2=0x30+(x%10);
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char('\"');
	Uart_Send_Char(0x0d);		
}

//�����LCD���յ�����//
void Lcd_Handle(void)
{
	if(1 == flag_recok)
	{
		/////////���������ֽ�����/////////
		if(2 == lcd_receive_count)
		{


			/////////LCD������Ϣ////////////
			//printf("If flag_recok=1: receive_buf=%c%c\r\n",receive_buf[0],receive_buf[1]);
			
			if('C' == receive_buf[0])
			{
				if('-' == receive_buf[1])
				{
					//printf("C-\r\n");
					//ָ�����//
				}
				if('+' == receive_buf[1])
				{
					//printf("C+\r\n");
					//ָ����ȷ//
				}								
			}
/*
1=PA	//��ҳ��
2=PB	//����
3=PC	//ϵͳ����
4=PD	//�ֶ�����
5=PE	//����˵��
6=PF	//Ȩ������
7=PG	//���ݵ���
8=PH	//RFID����
9=PK	//����ͳ��ҳ��
*/			
			
			///////ҳ���л�/////////
			if('P' == receive_buf[0])
			{
				LCD_Info_Ptr->LCD_Ready = 1;
				
				if('A' == receive_buf[1])
				{
					//printf("PA\r\n");
					LCD_Info_Ptr->ViewPage = LCD_MainPage;
					LCD_Info_Ptr->mainPage.battery_report();
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
					WarningLedCtrlPtr->twinkleNum = 2;
				}
				else if('B' == receive_buf[1])//����ҳ��
				{
					LCD_Info_Ptr->weightPage.battery_report();
					//printf("PB\r\n");
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
					LCD_Info_Ptr->ViewPage = LCD_WeightPage;
					WarningLedCtrlPtr->twinkleNum = 2;					
				}
				else if('C' == receive_buf[1])
				{
					//printf("PC\r\n");
					LCD_Info_Ptr->ViewPage = LCD_SystemPage;
					LCD_Info_Ptr->systemPage.battery_report();
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}	
				else if('D' == receive_buf[1])//�ֶ�����ҳ��
				{
					////�����ֶ����ƽ���////
					//printf("PD\r\n");
					LCD_Info_Ptr->ViewPage = LCD_ManualPage;
					LCD_Info_Ptr->manualPage.battery_report();
					ctrlParasPtr->agvWalkingMode = ManualMode;
					ctrlParasPtr->manualCtrl = Man_Stop;
					#if USE_ECV
					
					#else
					MOTOR_POWER_ON();
					#endif
					printf("ManualMode\r\n");
				}
				else if('E' == receive_buf[1])
				{	
					//printf("PE\r\n");					
					LCD_Info_Ptr->ViewPage = LCD_HelpPage;
					LCD_Info_Ptr->helpPage.battery_report();
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}
				else if('F' == receive_buf[1])
				{	
					//printf("PF\r\n");					
					LCD_Info_Ptr->ViewPage = LCD_AuthorityPage;
					LCD_Info_Ptr->authorityPage.battery_report();
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}
				else if('G' == receive_buf[1])
				{
					/////////���ݵ���////////////
					//printf("PG\r\n");
					LCD_Info_Ptr->ViewPage = LCD_DataoutPage;
					ctrlParasPtr->agvWalkingMode = AutomaticMode;					
					
				}
				else if('H' == receive_buf[1])
				{	
					//printf("PH\r\n");
					
					LCD_Info_Ptr->ViewPage = LCD_RfidPage;
					LCD_Info_Ptr->rfidPage.battery_report();
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}
				else if('K' == receive_buf[1])//����ͳ��ҳ��
				{		
					//printf("PK\r\n");
					LCD_Info_Ptr->ViewPage = LCD_TaskPage;
					LCD_Info_Ptr->taskPage.battery_report();
					//Set_task_state(task_state);	
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}
				else
				{
					printf("page error!!!");
				}
			}
			//////ɾ������///////
			else if('D' == receive_buf[0])
			{
				LCD_Info_Ptr->LCD_Ready = 1;
				if('D' == receive_buf[1])
				{
					//////ɾ������///////
					
				}
			}
			/////�ֶ�����//////
			else if('V' == receive_buf[0])
			{
				LCD_Info_Ptr->LCD_Ready = 1;
				if('F' == receive_buf[1])
				{
					////ǰ��//////
					ctrlParasPtr->manualCtrl = Man_Forward;
					//printf("Man_Forward\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if('B' == receive_buf[1])
				{
					///����/////
					ctrlParasPtr->manualCtrl = Man_Backward;
					//printf("Man_Backward\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if('L' == receive_buf[1])
				{
					///��ת/////
					ctrlParasPtr->manualCtrl = Man_CirL;
					//printf("Man_CirL\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if('R' == receive_buf[1])
				{
					///��ת/////
					ctrlParasPtr->manualCtrl = Man_CirR;
					//printf("Man_CirR\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if('S' == receive_buf[1])
				{
					///ֹͣ/////
					ctrlParasPtr->manualCtrl = Man_Stop;
					//printf("Man_Stop\r\n");
					//ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}
			}
		}

		if (('E' == receive_buf[0]) && ('+' == receive_buf[1]))
		{
			// �յ���Ļ��ȫ�����ź�
			LCD_Info_Ptr->LCD_Ready = 1;
			Set_date(BackgroudRTC_Rec_Hex.year, BackgroudRTC_Rec_Hex.month, BackgroudRTC_Rec_Hex.day);
			Set_time(BackgroudRTC_Rec_Hex.hour, BackgroudRTC_Rec_Hex.minute, BackgroudRTC_Rec_Hex.second);
		}
				
		//////���flag�����ռ���//////
		flag_recok = 0;
		lcd_receive_count = 0;
	}
}

void LCD_Page_Report(void)
{
	if(1 == LCD_Info_Ptr->LCD_Ready)
	{
		if(LCD_MainPage == LCD_Info_Ptr->ViewPage)
		{
			if(1 == BatteryInfoPtr->BatteryUpdate)
			{
				BatteryInfoPtr->BatteryUpdate = 0;

				//LCD_Report_Battery("main_battery.value=");
				LCD_Info_Ptr->mainPage.battery_report();
			}
		}
		else if(LCD_WeightPage == LCD_Info_Ptr->ViewPage)
		{
			if(1 == getWeightCtrl_Ptr->weightUpdate)
			{
				getWeightCtrl_Ptr->weightUpdate = 0;
				Report_Weight_Data();
			}

			if(1 == BatteryInfoPtr->BatteryUpdate)
			{
				BatteryInfoPtr->BatteryUpdate = 0;

				//LCD_Report_Battery("scale_battery.value=");
				LCD_Info_Ptr->weightPage.battery_report();
			}
			
		}
		else if(LCD_SystemPage == LCD_Info_Ptr->ViewPage)
		{
			if(1 == BatteryInfoPtr->BatteryUpdate)
			{
				BatteryInfoPtr->BatteryUpdate = 0;

				//LCD_Report_Battery("systemSetup_battery.value=");
				LCD_Info_Ptr->systemPage.battery_report();
			}
		}
		else if(LCD_ManualPage == LCD_Info_Ptr->ViewPage)
		{
			if(1 == BatteryInfoPtr->BatteryUpdate)
			{
				BatteryInfoPtr->BatteryUpdate = 0;

				//LCD_Report_Battery("manualControl_battery.value=");
				LCD_Info_Ptr->manualPage.battery_report();
			}
		}
		else if(LCD_HelpPage == LCD_Info_Ptr->ViewPage)
		{
			if(1 == BatteryInfoPtr->BatteryUpdate)
			{
				BatteryInfoPtr->BatteryUpdate = 0;

				//LCD_Report_Battery("help_battery.value=");
				LCD_Info_Ptr->helpPage.battery_report();
			}
		}
		else if(LCD_AuthorityPage == LCD_Info_Ptr->ViewPage)
		{
			if(1 == BatteryInfoPtr->BatteryUpdate)
			{
				BatteryInfoPtr->BatteryUpdate = 0;

				//LCD_Report_Battery("authority_battery.value=");
				LCD_Info_Ptr->authorityPage.battery_report();
			}
		}
		else if(LCD_DataoutPage == LCD_Info_Ptr->ViewPage)
		{
			
		}
		else if(LCD_RfidPage == LCD_Info_Ptr->ViewPage)
		{
			if(1 == BatteryInfoPtr->BatteryUpdate)
			{
				BatteryInfoPtr->BatteryUpdate = 0;

				//LCD_Report_Battery("rfidSetup_battery.value=");
				LCD_Info_Ptr->rfidPage.battery_report();
			}
		}
		else if(LCD_TaskPage == LCD_Info_Ptr->ViewPage)
		{
			if(1 == BatteryInfoPtr->BatteryUpdate)
			{
				BatteryInfoPtr->BatteryUpdate = 0;

				//LCD_Report_Battery("task_battery.value=");
				LCD_Info_Ptr->taskPage.battery_report();
			}
		}
		
	}
	
}


//���ڽ����жϺ���//
void UART1_REC(u8 data)
{
	receive_buf[lcd_receive_count] = data;
	
	if(receive_buf[lcd_receive_count] != 0x0D)
	{
		lcd_receive_count++;
	}
	else
	{
		flag_recok=1;
	}
}


void Weight_Screen_Show(void)
{
	Uart_Send_Str("scale.show()\r");
}

void Main_Screen_Show(void)
{
	Uart_Send_Str("main.show()\r");
}

void Task_Screen_Show(void)
{
	Uart_Send_Str("task.show()\r");
	
}

void RFID_Screen_Show(void)
{
	Uart_Send_Str("rfidSetup.show()\r");
	
}
void System_Screen_Show(void)
{
	Uart_Send_Str("systemSetup.show()\r");
	
}
void Help_Screen_Show(void)
{
	Uart_Send_Str("help.show()\r");
	
}
void Dataout_Screen_Show(void)
{
	Uart_Send_Str("dataout.show()\r");
	
}
void Manual_Screen_Show(void)
{
	Uart_Send_Str("manualControl.show()\r");
	
}

void Authority_Screen_Show(void)
{
	Uart_Send_Str("authority.show()\r");
	
}


void Screen_Save_Power_Mode(void)
{
	Uart_Send_Str("sysTouch.enabled=1\r");
	Uart_Send_Str("sysTouch.idleTimeout=10000\r");
}

void Light_Up_Screen(void)
{
	Uart_Send_Str("sysTouch.enabled=1\r");
}

#if 0
void LCD_Usart_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ����GPIOʱ�Ӻ͸��ù���ʱ��RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* ���� USART Tx ����������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ���� USART Rx �������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
}


void LCD_UART_INIT(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	LCD_Usart_GPIOInit();
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);								//��ʼ��

	/*
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(UART4, &USART_ClockInitStructure);
	*/
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_Init(UART4, &USART_InitStructure);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART4, ENABLE);
	USART_ClearFlag(UART4, USART_FLAG_TC);
}
#endif


void LCD_Usart_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ����GPIOʱ�Ӻ͸��ù���ʱ��RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* ���� USART Tx ����������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ���� USART Rx �������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
}

void LCD_UART_INIT(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	
	LCD_Usart_GPIOInit();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 			//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel; 	//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure); 							//��ʼ��

	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure);
	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
}

void LCD_Struct_Init(void)
{
	LCD_Info_Ptr->ViewPage = LCD_MainPage;

	LCD_Info_Ptr->mainPage.battery_report = LCD_Report_MainPage_Battery;
	LCD_Info_Ptr->weightPage.battery_report = LCD_Report_WeightPage_Battery;
	LCD_Info_Ptr->systemPage.battery_report = LCD_Report_SystemPage_Battery;
	LCD_Info_Ptr->manualPage.battery_report = LCD_Report_ManualPage_Battery;
	LCD_Info_Ptr->helpPage.battery_report = LCD_Report_HelpPage_Battery;
	LCD_Info_Ptr->authorityPage.battery_report = LCD_Report_AuthorityPage_Battery;
	LCD_Info_Ptr->rfidPage.battery_report = LCD_Report_RfidPage_Battery;
	LCD_Info_Ptr->taskPage.battery_report = LCD_Report_TaskPage_Battery;

	LCD_Info_Ptr->mainPage.show = Main_Screen_Show;
	LCD_Info_Ptr->weightPage.show = Weight_Screen_Show;
	LCD_Info_Ptr->taskPage.show = Task_Screen_Show;
	LCD_Info_Ptr->rfidPage.show = RFID_Screen_Show;
	LCD_Info_Ptr->systemPage.show = System_Screen_Show;
	LCD_Info_Ptr->helpPage.show = Help_Screen_Show;
	LCD_Info_Ptr->dataOutPage.show = Dataout_Screen_Show;
	LCD_Info_Ptr->manualPage.show = Manual_Screen_Show;
	LCD_Info_Ptr->authorityPage.show = Authority_Screen_Show;
}


void LCD_INIT(void)
{
	LCD_UART_INIT();

	LCD_Struct_Init();

	
}






	
