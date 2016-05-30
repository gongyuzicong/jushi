#ifndef __MAGN_SENSOR_H__
#define __MAGN_SENSOR_H__

#include "common_include.h"
#include "data_type.h"

#define FMS_Hex		GPIOF->IDR
#define RMS_Hex		GPIOG->IDR

typedef enum
{
	Offset_None = 0,
	Offset_Left,
	Offset_Right,
}Offset_Flag;

typedef enum
{
	Six_Bit = 0,
	Five_Bit,
}Bit_Num;


typedef struct
{
	vu16 MSD_Hex;
	
	vu8 MSD_Dec;
	
	Bit_Num BitNum;

	Offset_Flag OffsetFlag;
	
}Magn_Sensor_Data_Sturct, *Magn_Sensor_Data_Sturct_P;


typedef struct
{
	void (*MY_MSD_Operator)(u16, Magn_Sensor_Data_Sturct_P);
	void (*MS_Scan)(void);
	void (*MSD_SHOW)(u32, u32);
	void (*MSD_Test)(void);
}MSD_Functions_Struct, *MSD_Functions_Struct_P;


extern Magn_Sensor_Data_Sturct_P FMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Ptr;


extern MSD_Functions_Struct_P MSDF_Opts_Ptr;

void Magn_Sensor_Init(void);


#endif





