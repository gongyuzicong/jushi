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
	AgvNone,
	AgvCent2Left,
	AgvCent2Right,
	AgvRight2Cent,
	AgvLeft2Cent,
	AgvLeft2Right,
	AgvRight2Left,
}AgvDirection;

typedef enum
{
	AgvInits = 0,
	
	Agv_MS_Center,
	
	Agv_MS_Left_Begin,
	Agv_MS_Left_1,
	Agv_MS_Left_2,
	Agv_MS_Left_3,
	Agv_MS_Left_4,
	Agv_MS_Left_5,
	Agv_MS_Left_6,
	Agv_MS_Left_7,
	Agv_MS_Left_8,
	Agv_MS_Left_9,
	Agv_MS_Left_10,
	Agv_MS_Left_End,
	
	Agv_MS_Right_Begin,
	Agv_MS_Right_1,
	Agv_MS_Right_2,
	Agv_MS_Right_3,
	Agv_MS_Right_4,
	Agv_MS_Right_5,
	Agv_MS_Right_6,
	Agv_MS_Right_7,
	Agv_MS_Right_8,
	Agv_MS_Right_9,
	Agv_MS_Right_10,
	Agv_MS_Right_End,
	
	Agv_MS_Right_Outside,
	Agv_MS_Left_Outside,

	Agv_MS_Undefine,
	
}Agv_MS_Location;

typedef enum
{
	MSD_NORMAL = 0,
	MSD_LINE,
	MSD_OUTSIDE,
}MSD_Category;

typedef enum
{
	Agv_MP_Unknow = 0,
	Agv_MP_Center,

	Agv_MP_Left_Begin,
	Agv_MP_Left_1,
	Agv_MP_Left_2,
	Agv_MP_Left_3,
	Agv_MP_Left_4,
	Agv_MP_Left_5,
	Agv_MP_Left_6,
	Agv_MP_Left_7,
	Agv_MP_Left_8,
	Agv_MP_Left_End,

	Agv_MP_Right_Begin,
	Agv_MP_Right_1,
	Agv_MP_Right_2,
	Agv_MP_Right_3,
	Agv_MP_Right_4,
	Agv_MP_Right_5,
	Agv_MP_Right_6,
	Agv_MP_Right_7,
	Agv_MP_Right_8,
	Agv_MP_Right_End,
	
}Agv_Midpoint_Offset;

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

	Agv_MS_Location AgvMSLocation;

	s16 AgvMSLocation_s;

	MSD_Category MSDCategory;

	u8 bruce_crossroads_counter;

	u16 zeropointfive;
	u8 zflag;
}Magn_Sensor_Data_Sturct, *Magn_Sensor_Data_Sturct_P;


typedef struct
{
	Agv_Midpoint_Offset AgvMPLocation;

	s16 AgvMPLocation_s;
}Agv_Midpoint_Location_Struct, *Agv_Midpoint_Location_Struct_P;


typedef struct
{
	void (*MY_MSD_Operator)(Magn_Sensor_Data_Sturct_P);
	void (*MS_Scan)(void);
	void (*MSD_Test)(void);
	void (*MSD_Show_Bin)(u32);
	void (*Show_Opt_MSD)(Magn_Sensor_Data_Sturct_P);
	void (*magn_show)(Magn_Sensor_Data_Sturct_P);
	void (*Show_Infomation)(void);
}MSD_Functions_Struct, *MSD_Functions_Struct_P;


extern Magn_Sensor_Data_Sturct_P FMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P FMSDS_Pre_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Pre_Ptr;
extern Agv_Midpoint_Location_Struct_P AGV_MPLS_Ptr;


extern MSD_Functions_Struct_P MSDF_Opts_Ptr;

extern vu8 MagnSensorScanTime;

extern u8 MagnInfomationUpdate;

void Magn_Sensor_Init(void);


#endif





