#include "i2c_opts.h"
#include "cfg_gpio.h"



void I2C_delay(void)
{	
	u8 i=50; //��������Ż��ٶȣ���������͵�5����д��
	while(i) 
	{ 
		i--; 
	} 
}


bool I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	
	if(!SDA_READ)
	{
		return FALSE;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	}
	
	SDA_L;
	I2C_delay();
	
	if(SDA_READ)
	{
		return FALSE;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	}
	
	SDA_L;
	I2C_delay();
	
	return TRUE;
}









