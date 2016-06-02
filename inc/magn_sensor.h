#ifndef __MAGN_SENSOR_H__
#define __MAGN_SENSOR_H__

#include "common_include.h"
#include "data_type.h"

#define FMS_Hex		GPIOF->IDR
#define RMS_Hex		GPIOG->IDR

#define FIX_SCAN

typedef enum
{
	AgvInitS = 0,
	AgvCenter,
	AgvCent2Left,
	AgvCent2Right,
	AgvRight2Cent,
	AgvLeft2Cent,
	AgvLeft2Right,
	AgvRight2Left,
}AgvDirection;

typedef struct
{
	vu16 MSD_Hex;
	
	vs16 MSD_Dec;	// 左偏为正值, 右偏为负值, 中间为0

	vs16 VelocityX;

	vs16 AcceleratedX;

	vu32 VelocityXt;

	vs32 AcceleratedXt;

	s16 LeftRemain;

	s16 RightRemain;

	u8 BitNum;

	AgvDirection agvDirection;

	u32 TimeRecoder;
	
}Magn_Sensor_Data_Sturct, *Magn_Sensor_Data_Sturct_P;


typedef struct
{
	void (*MY_MSD_Operator)(Magn_Sensor_Data_Sturct_P);
	void (*MS_Scan)(void);
	void (*MSD_SHOW)(u32, u32);
	void (*MSD_Test)(void);
	void (*MSD_Show_Bin)(u32);
	void (*Show_Opt_MSD)(Magn_Sensor_Data_Sturct_P);
	void (*magn_show)(Magn_Sensor_Data_Sturct_P);
}MSD_Functions_Struct, *MSD_Functions_Struct_P;


extern Magn_Sensor_Data_Sturct_P FMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P FMSDS_Pre_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Pre_Ptr;


extern MSD_Functions_Struct_P MSDF_Opts_Ptr;

extern vu8 MagnSensorScanTime;

void Magn_Sensor_Init(void);


#endif





