#include "iic.h"

///��Ҫһ����ʱ5΢��ĺ����滻delay_us(4);///
#include "timer_opts.h"



//��ʼ��IIC,��main������ʼʱ����
void IIC_Init(void)
{					     
 	RCC->APB2ENR |= 1 << 3;//��ʹ������IO PORTBʱ�� 							 
	GPIOB->CRL &= 0X00FFFFFF;	//PB6/7 �������
	GPIOB->CRL |= 0X33000000;	   
	GPIOB->ODR |= 3 << 5;     //PB6/7 �����
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	Delay_us(4);
 	IIC_SDA=0;
	Delay_us(4);
	IIC_SCL=0;
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;
 	Delay_us(4); 
	IIC_SCL=1;
 	Delay_us(4); 
	IIC_SDA=1;					   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();       
	IIC_SDA=1;Delay_us(10);	   
	IIC_SCL=1;Delay_us(10);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>500)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	Delay_us(2);
	IIC_SCL=1;
	Delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	Delay_us(2);
	IIC_SCL=1;
	Delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		Delay_us(2); 
		IIC_SCL=0;	
		Delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        Delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		Delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}








