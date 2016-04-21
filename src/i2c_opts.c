#include "i2c_opts.h"
#include "cfg_gpio.h"



void I2C_delay(void)
{	
	u8 i=50; //这里可以优化速度，经测试最低到5还能写入
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
		return FALSE;	//SDA线为低电平则总线忙,退出
	}
	
	SDA_L;
	I2C_delay();
	
	if(SDA_READ)
	{
		return FALSE;	//SDA线为高电平则总线出错,退出
	}
	
	SDA_L;
	I2C_delay();
	
	return TRUE;
}









