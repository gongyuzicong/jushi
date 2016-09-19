#include "lcd.h"
#include "timer_opts.h"
#include "motion_control.h"
#include "fiberglas.h"
#include "cfg_gpio.h"
#include "fiberglas.h"


//添加到全局变量//
u8 receive_buf[150];
u8 lcd_receive_count=0;
u8 flag_recok=0;

u16 main_state[10]={11,22,33,44,55,66,77,88,99,100};
/*
mian_state里的十个数据分别为
应完成，总个数，合格，超重，超轻
总重量，超重率，超轻率，合格率，效率
*/


void Uart_Send_Char(u8 data)
{
	//u16 timeout = U16_MAX;
	u32 timeout = 0;
	USART_SendData(USART1, data);
	
#if 0
	
	//while (!(USART1->SR & USART_FLAG_TXE));
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//等待串口发送完成
	
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

//设置日期//
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

//设置时间//
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


//设置电量//
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
//更新主页任务状态//
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

//更新称重页面//z为整数部分,x为小数部分//
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

//处理从LCD接收的数据//
void Lcd_Handle(void)
{
	if(flag_recok==1)
	{
/////////处理三个字节数据/////////
		if(lcd_receive_count==2)
		{

/////////LCD反馈信息////////////
			if(receive_buf[0]==0x43)
			{
				if(receive_buf[1]==0x2D)
				{
					//指令错误//
				}
				if(receive_buf[1]==0x2B)
				{
					//指令正确//
				}								
			}
///////页面切换/////////
			if(receive_buf[0]==0x50)
			{
				if(receive_buf[1]==0x41)//任务状态页面
				{
					Set_batter(98);
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
					Set_main_state(main_state);
					getWeightCtrl_Ptr->weightReportFlag = 0;
					WarningLedCtrlPtr->twinkleNum = 2;
				}
				if(receive_buf[1]==0x42)//称重页面
				{
					Set_batter(98);
					ctrlParasPtr->agvWalkingMode = AutomaticMode;
					getWeightCtrl_Ptr->weightReportFlag = 1;
					//Set_scale_weight(00, 00);
					WarningLedCtrlPtr->twinkleNum = 2;
				}
				if(receive_buf[1]==0x44)//手动控制页面
				{
					Set_batter(98);
					////进入手动控制界面////
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
		//////删除数据///////
			if(receive_buf[0]==0x44)
			{
				if(receive_buf[1]==0x44)
				{
					//////删除数据///////
					
				}
			}
/////手动控制//////
			if(receive_buf[0]==0x56)
			{
				if(receive_buf[1]==0x46)
				{
					////前进//////
					ctrlParasPtr->manualCtrl = Man_Forward;
					//printf("Man_Forward\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if(receive_buf[1]==0x42)
				{
					///后退/////
					ctrlParasPtr->manualCtrl = Man_Backward;
					//printf("Man_Backward\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if(receive_buf[1]==0x4C)
				{
					///左转/////
					ctrlParasPtr->manualCtrl = Man_CirL;
					//printf("Man_CirL\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if(receive_buf[1]==0x52)
				{
					///右转/////
					ctrlParasPtr->manualCtrl = Man_CirR;
					//printf("Man_CirR\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if(receive_buf[1]==0x53)
				{
					///停止/////
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
				/////////数据导出////////////
				
			}
		}
/////////处理十八个字节数据/////////

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
//////清除flag，接收计数//////
		flag_recok=0;
		lcd_receive_count=0;
	}
}


//串口接收中断函数//
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

	/* 开启GPIO时钟和复用功能时钟RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* 配置 USART Tx 复用推挽输出 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* 配置 USART Rx 浮空输入 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
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
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;		//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);								//初始化

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

	/* 开启GPIO时钟和复用功能时钟RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* 配置 USART Tx 复用推挽输出 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置 USART Rx 浮空输入 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
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
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 			//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel; 	//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//使能中断
	NVIC_Init(&NVIC_InitStructure); 							//初始化

	
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






	
