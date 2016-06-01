#ifndef __MAGN_SENSOR_H__
#define __MAGN_SENSOR_H__

#include "common_include.h"
#include "data_type.h"

#define FMS_Hex		GPIOF->IDR
#define RMS_Hex		GPIOG->IDR

#define Max_MAGN_SCAN_TIME	10

#define FIX_SCAN


typedef enum
{
	Six_Bit = 0,
	Five_Bit,
}Bit_Num;


typedef struct
{
	vu16 MSD_Hex;
	
	vs16 MSD_Dec;	// 左偏为正值, 右偏为负值, 中间为0
	
	Bit_Num BitNum;

	vs16 VelocityX;

	vs16 AcceleratedX;
	
}Magn_Sensor_Data_Sturct, *Magn_Sensor_Data_Sturct_P;


typedef struct
{
	void (*MY_MSD_Operator)(u16, Magn_Sensor_Data_Sturct_P);
	void (*MS_Scan)(void);
	void (*MSD_SHOW)(u32, u32);
	void (*MSD_Test)(void);
	void (*MSD_Show_Bin)(u32);
}MSD_Functions_Struct, *MSD_Functions_Struct_P;


extern Magn_Sensor_Data_Sturct_P FMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Ptr;


extern MSD_Functions_Struct_P MSDF_Opts_Ptr;

extern vu8 MagnSensorScanTime;

void Magn_Sensor_Init(void);


#endif





