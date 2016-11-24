#ifndef __I2C_OPTS_H__
#define __I2C_OPTS_H__

#include "common_include.h"
#include "data_type.h"



#define I2C_SPEED_1K		5000	//���ݴ������ٶ����ã����ﴦ�����ٶ���72MHz

//I2C�˿ڶ���
#define I2C_SCL    	GPIOout(GPIOB, 6)	//SCL
#define I2C_SDA    	GPIOout(GPIOB, 7)	//SDA	 
#define READ_SDA   	GPIOin(GPIOB, 7)	//����SDA
#define I2C_WP		GPIOout(GPIOD, 12)

//����PB7�������
#define SDA_IN()  {GPIOB->CRL &= 0x0FFFFFFF; GPIOB->CRL |= 8 << 28;}
#define SDA_OUT() {GPIOB->CRL &= 0x0FFFFFFF; GPIOB->CRL |= 3 << 28;}
//#define SDA_IN()  {Change_SDA_IN();}
//#define SDA_OUT() {Change_SDA_OUT();}

typedef enum
{
	I2C_SUCCESS = 0,
	I2C_TIMEOUT,
	I2C_ERROR,
}I2C_StatusTypeDef;

extern u32 i2c_speed;	//I2C�����ٶ� = I2C_SPEED_1K / i2c_speed

/* ---------------------------����I2CЭ���д��ʱ����------------------------------*/
void My_I2C_Init(void);				//��ʼ��I2C��IO��				 
void I2C_Start(void);				//����I2C��ʼ�ź�
void I2C_Stop(void);					//����I2Cֹͣ�ź�
u8 I2C_Wait_ACK(void);				//I2C�ȴ�ACK�ź�
void I2C_ACK(void);					//I2C����ACK�ź�
void I2C_NACK(void);					//I2C������ACK�ź�
void I2C_Send_Byte(u8 data);		//I2C����һ���ֽ�
u8 I2C_Read_Byte(u8 ack);			//I2C��ȡһ���ֽ�
u16 I2C_SetSpeed(u16 speed);		//����I2C�ٶ�(1Kbps~400Kbps,speed��λ��Kbps)

/* ---------------------------���²����Ƿ�װ�õ�I2C��д����--------------------------- */

//���嵽ĳһ������������ϸ�Ķ�������������I2C���ֵ�˵������ΪĳЩ������I2C�Ķ�д������
//��һЩ���죬����Ĵ��������ھ��������I2C�����У�������֤OK�ģ�
I2C_StatusTypeDef I2C_WriteOneByte(u8 DevAddr, u8 DataAddr, u8 Data);			//��I2C���豸д��һ���ֽ�
I2C_StatusTypeDef I2C_WriteBurst(u8 DevAddr, u8 DataAddr, u8* pData, u32 Num);	//��I2C���豸����д��Num���ֽ�
I2C_StatusTypeDef I2C_ReadOneByte(u8 DevAddr, u8 DataAddr, u8* Data);			//��I2C���豸��ȡһ���ֽ�
I2C_StatusTypeDef I2C_ReadBurst(u8 DevAddr, u8 DataAddr, u8* pData, u32 Num);	//��I2C�豸������ȡNum���ֽ�
I2C_StatusTypeDef I2C_WriteBit(u8 DevAddr, u8 DataAddr, u8 Bitx, u8 BitSet);

#endif








