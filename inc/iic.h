#ifndef __IIC_H
#define __IIC_H

#include "cfg_gpio.h"
#include "i2c_opts.h"		   


//IO��������	 
#define IIC_SCL		PBout(6) 	//SCL
#define IIC_SDA		PBout(7) 	//SDA	 
//#define READ_SDA	PBout(7)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);//daddrΪ������ַ��addrΪ���ݵ�ַ��dataΪд�������
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 // daddrΪ������ַ��addrΪ���ݵ�ַ

#endif
















