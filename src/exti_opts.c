#include "exti_opts.h"



void EXTI_GPIO_Configuration(void)
{
	/* ����GPIO��ʼ���ṹ�� GPIO_InitStructure */
  	GPIO_InitTypeDef GPIO_InitStructure;
#if 1
  	/* ����PA.0Ϊ�������루EXTI Line0��*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOD , &GPIO_InitStructure);

	/* ����PA.0Ϊ�ⲿ�ж�0����ͨ����EXIT0�� */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD , GPIO_PinSource2);
#else
	/* ����PA.0Ϊ�������루EXTI Line0��*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOC , &GPIO_InitStructure);

	/* ����PA.0Ϊ�ⲿ�ж�0����ͨ����EXIT0�� */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC , GPIO_PinSource1);
#endif
}


void NVIC_Configuration(void)
{
	#if 0
	/* #ifdef...#else...#endif�ṹ�������Ǹ���Ԥ�������������ж���������ʼ��ַ*/   
#ifdef  VECT_TAB_RAM  
  	/* �ж���������ʼ��ַ�� 0x20000000 ��ʼ */ 
 	NVIC_SetVectorTable(NVIC_VectTab_RAM , 0x0); 
#else 	/* VECT_TAB_FLASH */
  	/* �ж���������ʼ��ַ�� 0x80000000 ��ʼ */ 
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH , 0x0);   
#endif
	#endif

#if 0
	/* ����NVIC��ʼ���ṹ�� NVIC_InitStructure */
	NVIC_InitTypeDef NVIC_InitStructure;
  	/* ѡ��NVIC���ȼ�����1 */
	
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  	/* ʹ��EXIT 0ͨ�� ��0����ռ���ȼ� ��0����ռ���ȼ� */
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#else
	/* ʹ��EXIT 0ͨ�� ��2����ռ���ȼ� ��3����ռ���ȼ� */
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//ѡ���жϷ���
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQChannel;		//ѡ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//����ʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//��Ӧʽ�ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ��
#endif

}


void EXIT_Configuration(void)
{
	/* ����EXIT��ʼ���ṹ�� EXTI_InitStructure */
	EXTI_InitTypeDef EXTI_InitStructure;

	/* �����ⲿ�ж�0ͨ����EXIT Line0�����½���ʱ�����ж� */  
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



