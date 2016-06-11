#ifndef __MAGN_D_ALGO_H__
#define __MAGN_D_ALGO_H__


#include "stm32f10x_type.h"
#include "common_include.h"
#include "data_type.h"

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
	
	Agv_MS_Left_End,		// 1
	Agv_MS_Left_10,			// 2
	Agv_MS_Left_9,
	Agv_MS_Left_8,
	Agv_MS_Left_7,
	Agv_MS_Left_6,
	Agv_MS_Left_5,
	Agv_MS_Left_4,
	Agv_MS_Left_3,
	Agv_MS_Left_2,
	Agv_MS_Left_1,			// 11
	//Agv_MS_Left_Begin,

	Agv_MS_Center,		//12
	
	//Agv_MS_Right_Begin,
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
	Agv_MS_Right_End,	// 23
	
	Agv_MS_Right_Outside,
	Agv_MS_Left_Outside,

	Agv_MS_Undefine,
	Agv_MS_Overline,
	
	Agv_MS_LOut_1,		// 27
	Agv_MS_LOut_2,
	Agv_MS_LOut_3,
	Agv_MS_LOut_4,
	Agv_MS_LOut_5,
	Agv_MS_LOut_6,
	Agv_MS_LOut_7,
	Agv_MS_LOut_8,		// 34

	Agv_MS_ROut_1,		// 35
	Agv_MS_ROut_2,
	Agv_MS_ROut_3,
	Agv_MS_ROut_4,
	Agv_MS_ROut_5,
	Agv_MS_ROut_6,
	Agv_MS_ROut_7,
	Agv_MS_ROut_8,		// 42
	
}Agv_MS_Location;


typedef enum
{
	MSD_NORMAL = 0,
	MSD_LINE,
	MSD_OUTSIDE,
}MSD_Category;


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


extern Magn_Sensor_Data_Sturct_P FMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P FMSDS_Pre_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Pre_Ptr;


void MSDS_Init(void);
void MSD_Analy(Magn_Sensor_Data_Sturct_P);
void MSD_Show_Bin(u32);
void Show_Resualt_Analy(Magn_Sensor_Data_Sturct_P);




#endif


