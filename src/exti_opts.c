#include "exti_opts.h"



void EXTI_GPIO_Configuration(void)
{
	/* 定义GPIO初始化结构体 GPIO_InitStructure */
  	GPIO_InitTypeDef GPIO_InitStructure;
#if 1
  	/* 设置PA.0为上拉输入（EXTI Line0）*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOD , &GPIO_InitStructure);

	/* 定义PA.0为外部中断0输入通道（EXIT0） */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD , GPIO_PinSource2);
#else
	/* 设置PA.0为上拉输入（EXTI Line0）*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOC , &GPIO_InitStructure);

	/* 定义PA.0为外部中断0输入通道（EXIT0） */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC , GPIO_PinSource1);
#endif
}


void NVIC_Configuration(void)
{
	#if 0
	/* #ifdef...#else...#endif结构的作用是根据预编译条件决定中断向量表起始地址*/   
#ifdef  VECT_TAB_RAM  
  	/* 中断向量表起始地址从 0x20000000 开始 */ 
 	NVIC_SetVectorTable(NVIC_VectTab_RAM , 0x0); 
#else 	/* VECT_TAB_FLASH */
  	/* 中断向量表起始地址从 0x80000000 开始 */ 
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH , 0x0);   
#endif
	#endif

#if 0
	/* 定义NVIC初始化结构体 NVIC_InitStructure */
	NVIC_InitTypeDef NVIC_InitStructure;
  	/* 选择NVIC优先级分组1 */
	
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  	/* 使能EXIT 0通道 ，0级先占优先级 ，0级次占优先级 */
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#else
	/* 使能EXIT 0通道 ，2级先占优先级 ，3级次占优先级 */
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//选择中断分组
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQChannel;		//选择中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢断式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//响应式中断优先级设置
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);//初始化
#endif

}


void EXIT_Configuration(void)
{
	/* 定义EXIT初始化结构体 EXTI_InitStructure */
	EXTI_InitTypeDef EXTI_InitStructure;

	/* 设置外部中断0通道（EXIT Line0）在下降沿时触发中断 */  
  	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);
}



void ExtiInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	EXTI_GPIO_Configuration();
	NVIC_Configuration();
	EXIT_Configuration();
}



