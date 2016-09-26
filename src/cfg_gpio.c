#include "cfg_gpio.h"


void CB_CAN_GPIO_CFG(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // 上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure CAN pin: TX */	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#if 0
	/* Configure TJA1041T pin: TJA_EN */	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure TJA1041T pin: TJA_STB */	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
}

void CB_LED_GPIO_CFG(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*设置GPIOA.2和GPIOA.3为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_5);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7);

	GPIO_SetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_SetBits(GPIOD, GPIO_Pin_3);
}

void CB_USART_GPIO_CFG(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*设置UART1的TX脚(PA.9)为第二功能推挽输出模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*设置UART1的RX脚(PA.10)为第二功能推挽输出模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

		
}

void CB_OUTPUT_CLK_GPIO_CFG(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_MCOConfig(RCC_MCO_PLLCLK_Div2);
	
}

void CB_I2C_GPIO_CFG(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void CB_SYNC_GPIO_CFG(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void PWM_GPIO_CFG(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/****TIM3 CH1 CH2 CH3 CH4 PWM输出 GPIO分别为 PA6 PA7 PB0 PB1*****/
	/*设置GPIOA.2和GPIOA.3为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/**********************************************************************************
函数名: 
函数描述: 
输入参数: 
输出结果: 
返回值: 
***********************************************************************************/

void CB_GPIO_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);	/*打开APB2总线上的GPIOA时钟*/
	
	CB_LED_GPIO_CFG();
	CB_USART_GPIO_CFG();
	//CB_CAN_GPIO_CFG();
	//CB_OUTPUT_CLK_GPIO_CFG();
	//CB_I2C_GPIO_CFG();
	//CB_SYNC_GPIO_CFG();
	PWM_GPIO_CFG();
	//I2C_GPIO_Config();
	
}


