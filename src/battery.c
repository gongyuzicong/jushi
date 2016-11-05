#include "battery.h"
#include "timer_opts.h"

Battery_Info_Str BatteryStr;
Battery_Info_Str_P BatteryStrPtr = &BatteryStr;


/**
  * @brief ���ADCֵ
  * @param ADCx: ADC1 / ADC2 / ADC3
  * @param ch:ͨ��ֵ ADC_Channel_0 ~ ADC_Channel_17
  * @retval ADCת��ֵ
  * @note 
  */
u16 Get_Adc(ADC_TypeDef* ADCx, u8 ch)   
{
	/* ADC�Ĺ���ͨ�������ã�һ�����У�����ʱ��239.5����	*/
	ADC_RegularChannelConfig(ADCx, ch, 1, ADC_SampleTime_239Cycles5);

	ADC_SoftwareStartConvCmd(ADCx, ENABLE);				//��ʼת��
	while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC ));		//�ȴ�ת������

	return ADC_GetConversionValue(ADCx);				//����ADC1ת�����
}

/**
  * @brief ��ȡ����10��ADC������ƽ��ֵ
  * @param ADCx: ADC1 / ADC2 / ADC3
  * @param ch:ͨ��ֵ ADC_Channel_0 ~ ADC_Channel_17
  * @retval ADCת����ƽ��ֵ
  * @note 
  */
u16 Get_Adc_Average(ADC_TypeDef* ADCx, u8 ch)
{
	u32 adc_val = 0;
	u8 i;

	for(i = 0; i < 10; i++)
	{
		adc_val += Get_Adc(ADCx, ch);
		Delay_ms(5);
	}
	
	return (adc_val / 10);
}


void Get_Voltage(void)
{
	u16 AdcValue;
	float Voltage;

	/* ��ȡ ADC1_IN1 ����ֵ */
	AdcValue = Get_Adc_Average(ADC1, ADC_Channel_1);
	
	Voltage = (float)(AdcValue * (3.3 / 4096.0));

	printf("Voltage = %f\r\n", Voltage);
}

void Scan_Voltage(void)
{
	
	
}



/**
  * @brief ADC��ʼ��
  * @param none
  * @retval none
  * @note  ��ʼ��PA.00ΪADC1_CH0������ת�����������ADCת��
  */ 
void ADC_Initialize(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��GPIOA��ADC1,AFIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

	/* ����ADCCLK��Ƶ���� ADCCLK = PCLK2/6���� 72MHz/6 = 12MHz */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	/* ���� PA.00 (ADC1_IN0) ��Ϊģ���������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	ADC_DeInit(ADC1);  //��ADC1��Ϊȱʡֵ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					//����ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//�������ADCת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//����ת��ͨ����Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);									//����ADC_InitStruct��ʼ��ADC

	ADC_Cmd(ADC1, ENABLE);												//ʹ��ADC1

	ADC_ResetCalibration(ADC1);											//��λADCУ׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1));							//�ȴ���λУ׼����
	ADC_StartCalibration(ADC1);	 										//����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 							//�ȴ�У׼���� 
	
}


void Battery_Init(void)
{
	
	ADC_Initialize();
	
}




