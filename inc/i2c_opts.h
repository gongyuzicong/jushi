#ifndef __I2C_OPTS_H__
#define __I2C_OPTS_H__

#include "common_include.h"
#include "data_type.h"



#define I2C_SPEED_1K		5000	//根据处理器速度设置，这里处理器速度是72MHz

//I2C端口定义
#define I2C_SCL    	GPIOout(GPIOB, 6)	//SCL
#define I2C_SDA    	GPIOout(GPIOB, 7)	//SDA	 
#define READ_SDA   	GPIOin(GPIOB, 7)	//输入SDA
#define I2C_WP		GPIOout(GPIOD, 12)

//设置PB7输入输出
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

extern u32 i2c_speed;	//I2C访问速度 = I2C_SPEED_1K / i2c_speed

/* ---------------------------依照I2C协议编写的时序函数------------------------------*/
void My_I2C_Init(void);				//初始化I2C的IO口				 
void I2C_Start(void);				//发送I2C开始信号
void I2C_Stop(void);					//发送I2C停止信号
u8 I2C_Wait_ACK(void);				//I2C等待ACK信号
void I2C_ACK(void);					//I2C发送ACK信号
void I2C_NACK(void);					//I2C不发送ACK信号
void I2C_Send_Byte(u8 data);		//I2C发送一个字节
u8 I2C_Read_Byte(u8 ack);			//I2C读取一个字节
u16 I2C_SetSpeed(u16 speed);		//设置I2C速度(1Kbps~400Kbps,speed单位，Kbps)

/* ---------------------------以下部分是封装好的I2C读写函数--------------------------- */

//具体到某一个器件，请仔细阅读器件规格书关于I2C部分的说明，因为某些器件在I2C的读写操作会
//有一些差异，下面的代码我们在绝大多数的I2C器件中，都是验证OK的！
I2C_StatusTypeDef I2C_WriteOneByte(u8 DevAddr, u8 DataAddr, u8 Data);			//向I2C从设备写入一个字节
I2C_StatusTypeDef I2C_WriteBurst(u8 DevAddr, u8 DataAddr, u8* pData, u32 Num);	//向I2C从设备连续写入Num个字节
I2C_StatusTypeDef I2C_ReadOneByte(u8 DevAddr, u8 DataAddr, u8* Data);			//从I2C从设备读取一个字节
I2C_StatusTypeDef I2C_ReadBurst(u8 DevAddr, u8 DataAddr, u8* pData, u32 Num);	//从I2C设备连续读取Num个字节
I2C_StatusTypeDef I2C_WriteBit(u8 DevAddr, u8 DataAddr, u8 Bitx, u8 BitSet);

#endif








