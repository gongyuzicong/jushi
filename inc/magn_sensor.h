#ifndef __MAGN_SENSOR_H__
#define __MAGN_SENSOR_H__

#include "common_include.h"
#include "data_type.h"
#include "magn_d_algo.h"

#define FMS_Hex		GPIOF->IDR
#define RMS_Hex		GPIOG->IDR

#define FIX_SCAN

#define STATION_1AND2_RFID		0x0001
#define STATION_3AND4_RFID		0x0002
#define STATION_5AND6_RFID		0x0003
#define STATION_7AND8_RFID		0x0004
#define STATION_9AND10_RFID		0x0005


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
	u8 lock;
	u8 updateFlag;
	u32 rfidData;
	u8 noValide;
}RFID_Struct, *RFID_Struct_P;


typedef struct
{
	Agv_Midpoint_Offset AgvMPLocation;

	s16 AgvMPLocation_s;
}Agv_Midpoint_Location_Struct, *Agv_Midpoint_Location_Struct_P;


typedef struct
{
	void (*MY_MSD_Operator)(Magn_Sensor_Data_Sturct_P);
	void (*MSD_Show_Bin)(u32);
	void (*Show_Opt_MSD)(Magn_Sensor_Data_Sturct_P);
	void (*magn_show)(Magn_Sensor_Data_Sturct_P);
}MSD_Functions_Struct, *MSD_Functions_Struct_P;



extern Agv_Midpoint_Location_Struct_P AGV_MPLS_Ptr;

extern MSD_Functions_Struct_P MSDF_Opts_Ptr;

extern vu8 MagnSensorScanTime;
extern RFID_Struct_P RFID_Info_Ptr;

void Magn_Sensor_Init(void);
void Magn_Sensor_Scan(void);
void Show_Infomation(void);
void Check_Max_MSLocation(void);
void show_Excel_Analysis_Info(void);


#endif





