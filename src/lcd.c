#include "lcd.h"
#include "timer_opts.h"
#include "motion_control.h"
#include "fiberglas.h"
#include "cfg_gpio.h"
#include "fiberglas.h"


//��ӵ�ȫ�ֱ���//
u8 receive_buf[150];
u8 lcd_receive_count=0;
u8 flag_recok=0;

u16 main_state[10]={11,22,33,44,55,66,77,88,99,100};
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
void Set_batter(u8 batter)
{
	u8 temp1,temp2;
	if(batter>=100)
	{
	Uart_Send_Str("main_battery.vakue=100");
	}
	else
	{
	temp1=(batter/10)+0x30;
	temp2=(batter%10)+0x30;
	
	Uart_Send_Str("main_battery.value=");
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char(0x0d);
	
	Uart_Send_Str("scale_battery.value=");
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char(0x0d);
	
	Uart_Send_Str("systemSetup_battery.value=");
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);	
	Uart_Send_Char(0x0d);
	
	Uart_Send_Str("manualControl_battery.value=");
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);	
	Uart_Send_Char(0x0d);

	Uart_Send_Str("help_battery.value=");
	Uart_Send_Char(temp1);
	Uart_Send_Char(temp2);
	Uart_Send_Char(0x0d);
	}	
}
//������ҳ����״̬//
void Set_main_state(u16 *buf)
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
		Uart_Send_Str("main_L");
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
	Uart_Send_Str("main_L");
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
	if(flag_recok==1)
	{
/////////���������ֽ�����/////////
		if(lcd_receive_count==2)
		{

/////////LCD������Ϣ////////////
			if(receive_buf[0]==0x43)
			{
				if(receive_buf[1]==0x2D)
				{
					//ָ�����//
				}
				if(receive_buf[1]==0x2B)
				{
					//ָ����ȷ//
				}								
			}
///////ҳ���л�/////////
			if(receive_buf[0]==0x50)
			{
				if(receive_buf[1]==0x41)//����״̬ҳ��
				{
					Set_batter(98);
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
					Set_main_state(main_state);
					getWeightCtrl_Ptr->weightReportFlag = 0;
					WarningLedCtrlPtr->twinkleNum = 2;
				}
				if(receive_buf[1]==0x42)//����ҳ��
				{
					Set_batter(98);
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
					getWeightCtrl_Ptr->weightReportFlag = 1;
					//Set_scale_weight(00, 00);
					WarningLedCtrlPtr->twinkleNum = 2;
				}
				if(receive_buf[1]==0x44)//�ֶ�����ҳ��
				{
					Set_batter(98);
					////�����ֶ����ƽ���////
					ctrlParasPtr->agvWalkingMode = ManualMode;
					ctrlParasPtr->manualCtrl = Man_Stop;
					#if USE_ECV
					
					#else
					MOTOR_POWER_ON();
					#endif
					getWeightCtrl_Ptr->weightReportFlag = 0;
					printf("ManualMode\r\n");
				}
			}
		//////ɾ������///////
			if(receive_buf[0]==0x44)
			{
				if(receive_buf[1]==0x44)
				{
					//////ɾ������///////
					
				}
			}
/////�ֶ�����//////
			if(receive_buf[0]==0x56)
			{
				if(receive_buf[1]==0x46)
				{
					////ǰ��//////
					ctrlParasPtr->manualCtrl = Man_Forward;
					//printf("Man_Forward\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if(receive_buf[1]==0x42)
				{
					///����/////
					ctrlParasPtr->manualCtrl = Man_Backward;
					//printf("Man_Backward\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if(receive_buf[1]==0x4C)
				{
					///��ת/////
					ctrlParasPtr->manualCtrl = Man_CirL;
					//printf("Man_CirL\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if(receive_buf[1]==0x52)
				{
					///��ת/////
					ctrlParasPtr->manualCtrl = Man_CirR;
					//printf("Man_CirR\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if(receive_buf[1]==0x53)
				{
					///ֹͣ/////
					ctrlParasPtr->manualCtrl = Man_Stop;
					//printf("Man_Stop\r\n");
					//ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}
			}
		}
		if(lcd_receive_count==4)
		{
			if((receive_buf[1]==0x47)&&(receive_buf[3]==0x41))
			{
				/////////���ݵ���////////////
				
			}
		}
/////////����ʮ�˸��ֽ�����/////////

		if(lcd_receive_count>10)
		{
			/*
			Set_date(16,8,7);
			Set_time(12,13,14);
			Set_batter(100);
			Delay_ms(500);
			Set_date(16,8,7);
			Set_time(12,13,14);
			Set_batter(100);
			Set_main_state(main_state);
			*/
			Set_date(BackgroudRTC_Rec_Hex.year, BackgroudRTC_Rec_Hex.month, BackgroudRTC_Rec_Hex.day);
			Set_time(BackgroudRTC_Rec_Hex.hour, BackgroudRTC_Rec_Hex.minute, BackgroudRTC_Rec_Hex.second);
			Set_batter(98);
			Delay_ms(500);
			Set_date(BackgroudRTC_Rec_Hex.year, BackgroudRTC_Rec_Hex.month, BackgroudRTC_Rec_Hex.day);
			Set_time(BackgroudRTC_Rec_Hex.hour, BackgroudRTC_Rec_Hex.minute, BackgroudRTC_Rec_Hex.second);
			Set_batter(98);
			Set_main_state(main_state);
			Light_Up_Screen();
		}
//////���flag�����ռ���//////
		flag_recok=0;
		lcd_receive_count=0;
	}
}


//���ڽ����жϺ���//
void UART1_REC(u8 data)
{
	receive_buf[lcd_receive_count] = data;
	
	if(receive_buf[lcd_receive_count]!=0x0D)
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



void LCD_INIT(void)
{
	LCD_UART_INIT();

	
}






	
