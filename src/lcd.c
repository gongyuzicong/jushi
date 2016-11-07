#include "lcd.h"
#include "timer_opts.h"
#include "motion_control.h"
#include "fiberglas.h"
#include "cfg_gpio.h"
#include "fiberglas.h"
#include "ecv_control.h"
#include "fiberglas.h"
#include "battery.h"

//添加到全局变量//
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
//更新主页任务状态//
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
	if(1 == flag_recok)
	{
		/////////处理三个字节数据/////////
		if(2 == lcd_receive_count)
		{


			/////////LCD反馈信息////////////
			//printf("If flag_recok=1: receive_buf=%c%c\r\n",receive_buf[0],receive_buf[1]);
			
			if('C' == receive_buf[0])
			{
				if('-' == receive_buf[1])
				{
					//printf("C-\r\n");
					//指令错误//
				}
				if('+' == receive_buf[1])
				{
					//printf("C+\r\n");
					//指令正确//
				}								
			}
/*
1=PA	//主页面
2=PB	//称重
3=PC	//系统设置
4=PD	//手动控制
5=PE	//操作说明
6=PF	//权限设置
7=PG	//数据导出
8=PH	//RFID设置
9=PK	//任务统计页面
*/			
			
			///////页面切换/////////
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
				else if('B' == receive_buf[1])//称重页面
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
				else if('D' == receive_buf[1])//手动控制页面
				{
					////进入手动控制界面////
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
					/////////数据导出////////////
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
				else if('K' == receive_buf[1])//任务统计页面
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
			//////删除数据///////
			else if('D' == receive_buf[0])
			{
				LCD_Info_Ptr->LCD_Ready = 1;
				if('D' == receive_buf[1])
				{
					//////删除数据///////
					
				}
			}
			/////手动控制//////
			else if('V' == receive_buf[0])
			{
				LCD_Info_Ptr->LCD_Ready = 1;
				if('F' == receive_buf[1])
				{
					////前进//////
					ctrlParasPtr->manualCtrl = Man_Forward;
					//printf("Man_Forward\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if('B' == receive_buf[1])
				{
					///后退/////
					ctrlParasPtr->manualCtrl = Man_Backward;
					//printf("Man_Backward\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if('L' == receive_buf[1])
				{
					///左转/////
					ctrlParasPtr->manualCtrl = Man_CirL;
					//printf("Man_CirL\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if('R' == receive_buf[1])
				{
					///右转/////
					ctrlParasPtr->manualCtrl = Man_CirR;
					//printf("Man_CirR\r\n");
					//ctrlParasPtr->agvWalkingMode = ManualMode;
				}
				if('S' == receive_buf[1])
				{
					///停止/////
					ctrlParasPtr->manualCtrl = Man_Stop;
					//printf("Man_Stop\r\n");
					//ctrlParasPtr->agvWalkingMode = AutomaticMode;
				}
			}
		}

		if (('E' == receive_buf[0]) && ('+' == receive_buf[1]))
		{
			// 收到屏幕完全启动信号
			LCD_Info_Ptr->LCD_Ready = 1;
			Set_date(BackgroudRTC_Rec_Hex.year, BackgroudRTC_Rec_Hex.month, BackgroudRTC_Rec_Hex.day);
			Set_time(BackgroudRTC_Rec_Hex.hour, BackgroudRTC_Rec_Hex.minute, BackgroudRTC_Rec_Hex.second);
		}
				
		//////清除flag，接收计数//////
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


//串口接收中断函数//
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






	
