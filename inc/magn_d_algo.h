#ifndef __MAGN_D_ALGO_H__
#define __MAGN_D_ALGO_H__


#include "stm32f10x_type.h"
#include "common_include.h"
#include "data_type.h"

#define CHECK_LEFT_H(value)				((0 == value % 2) ? 1 : 0)
#define CHECK_RIGHT_H(value)			((0 == value % 2) ? 1 : 0)


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
	Agv_MS_Left_9,			// 3
	Agv_MS_Left_8,			// 4
	Agv_MS_Left_7,			// 5
	Agv_MS_Left_6,			// 6
	Agv_MS_Left_5,			// 7
	Agv_MS_Left_4_5,		// 8
	Agv_MS_Left_4,			// 9
	Agv_MS_Left_3_5,		// 10
	Agv_MS_Left_3,			// 11
	Agv_MS_Left_2_5,		// 12
	Agv_MS_Left_2,			// 13
	Agv_MS_Left_1_5,		// 14
	Agv_MS_Left_1,			// 15
	Agv_MS_Left_0_5,		// 16
	//Agv_MS_Left_Begin,

	Agv_MS_Center,			// 17
	
	//Agv_MS_Right_Begin,
	Agv_MS_Right_0_5,		// 18
	Agv_MS_Right_1,			// 19
	Agv_MS_Right_1_5,		// 20
	Agv_MS_Right_2,			// 21
	Agv_MS_Right_2_5,		// 22
	Agv_MS_Right_3,			// 23
	Agv_MS_Right_3_5,		// 24
	Agv_MS_Right_4,			// 25
	Agv_MS_Right_4_5,		// 26
	Agv_MS_Right_5,			// 27
	Agv_MS_Right_6,			// 28
	Agv_MS_Right_7,			// 29
	Agv_MS_Right_8,			// 30
	Agv_MS_Right_9,			// 31
	Agv_MS_Right_10,		// 32
	Agv_MS_Right_End,		// 33
	
	Agv_MS_Right_Outside,
	Agv_MS_Left_Outside,

	Agv_MS_Undefine,
	Agv_MS_Overline,
	
	Agv_MS_LOut_1,			// 40
	Agv_MS_LOut_2,
	Agv_MS_LOut_3,
	Agv_MS_LOut_4,
	Agv_MS_LOut_5,
	Agv_MS_LOut_6,
	Agv_MS_LOut_7,
	Agv_MS_LOut_8,			// 47

	Agv_MS_ROut_1,			// 48
	Agv_MS_ROut_2,
	Agv_MS_ROut_3,
	Agv_MS_ROut_4,
	Agv_MS_ROut_5,
	Agv_MS_ROut_6,
	Agv_MS_ROut_7,
	Agv_MS_ROut_8,			// 55

	Agv_MS_CrossRoad,		// 56
	
}Agv_MS_Location;


typedef enum
{
	MSD_NORMAL = 0,
	MSD_LINE,
	MSD_OUTSIDE,
}MSD_Category;


typedef struct
{
	s16 Angle;
	s16 Midpoint;
	s16 AngleDirection;		// 负数为靠近0, 正数为远离0
	u32 AngleVxt;
	s16 MidpointDirection;	// 负数中点向中线靠近, 正数中点远离中线, 注意! 此值有可能为0! 0代表在原地转!, 还有无论从那个方向进入中点, 都会让此值为0  // 负数中点向中线靠近, 正数中点远离中线
	u32 MidpointVxt;
	u32 MidpointTimRec;
}Pattern_Num_Para, *Pattern_Num_Para_P;


typedef enum
{
	Angle_Level_Unknow = 0,

	Angle_Level_Error,
	
	Angle_Level_L_20,
	Angle_Level_L_19,
	Angle_Level_L_18,
	Angle_Level_L_17,
	Angle_Level_L_16,
	Angle_Level_L_15,
	Angle_Level_L_14,
	Angle_Level_L_13,
	Angle_Level_L_12,
	Angle_Level_L_11,
	Angle_Level_L_10,
	Angle_Level_L_9,
	Angle_Level_L_8,
	Angle_Level_L_7,
	Angle_Level_L_6,
	Angle_Level_L_5,
	Angle_Level_L_4,
	Angle_Level_L_3,
	Angle_Level_L_2,
	Angle_Level_L_1,
	
	Angle_Level_0,

	Angle_Level_R_1,
	Angle_Level_R_2,
	Angle_Level_R_3,
	Angle_Level_R_4,
	Angle_Level_R_5,
	Angle_Level_R_6,
	Angle_Level_R_7,
	Angle_Level_R_8,
	Angle_Level_R_9,
	Angle_Level_R_10,
	Angle_Level_R_11,
	Angle_Level_R_12,
	Angle_Level_R_13,
	Angle_Level_R_14,
	Angle_Level_R_15,
	Angle_Level_R_16,
	Angle_Level_R_17,
	Angle_Level_R_18,
	Angle_Level_R_19,
	Angle_Level_R_20,
	
}Angle_Scale;

typedef enum
{
	Midpoint_Level_Unknow = 0,
	Midpoint_Level_Error,
	
	Midpoint_Level_L_20,
	Midpoint_Level_L_19,
	Midpoint_Level_L_18,
	Midpoint_Level_L_17,
	Midpoint_Level_L_16,
	Midpoint_Level_L_15,
	Midpoint_Level_L_14,
	Midpoint_Level_L_13,
	Midpoint_Level_L_12,
	Midpoint_Level_L_11,
	Midpoint_Level_L_10,
	Midpoint_Level_L_9,
	Midpoint_Level_L_8,
	Midpoint_Level_L_7,
	Midpoint_Level_L_6,
	Midpoint_Level_L_5,
	Midpoint_Level_L_4,
	Midpoint_Level_L_3,
	Midpoint_Level_L_2,
	Midpoint_Level_L_1,
	
	Midpoint_Level_0,

	Midpoint_Level_R_1,
	Midpoint_Level_R_2,
	Midpoint_Level_R_3,
	Midpoint_Level_R_4,
	Midpoint_Level_R_5,
	Midpoint_Level_R_6,
	Midpoint_Level_R_7,
	Midpoint_Level_R_8,
	Midpoint_Level_R_9,
	Midpoint_Level_R_10,
	Midpoint_Level_R_11,
	Midpoint_Level_R_12,
	Midpoint_Level_R_13,
	Midpoint_Level_R_14,
	Midpoint_Level_R_15,
	Midpoint_Level_R_16,
	Midpoint_Level_R_17,
	Midpoint_Level_R_18,
	Midpoint_Level_R_19,
	Midpoint_Level_R_20,
	
	
}Midpoint_Scale;


typedef struct
{
	Angle_Scale Angle;
	Midpoint_Scale Midpoint;
}Pattern_Scale_Para, *Pattern_Scale_Para_P;


typedef struct
{
	vu16 MSD_Hex;	// 最原始的数据
	
	vs16 MSD_Dec;	// 左偏为正值, 右偏为负值, 中间为0

	vs16 VelocityX;		// 磁传感器x方向偏移时间

	vs16 AcceleratedX;	// 加速度时间

	vu32 VelocityXt;

	s32 AcceleratedXt;

	s16 LeftRemain;		// 

	s16 RightRemain;	// 

	u8 BitNum;			// 可以忽略	

	AgvDirection agvDirection;	// 左右偏移的方向

	u32 TimeRecoder;		

	Agv_MS_Location AgvMSLocation;		// 磁传感器最原始的数据处理后结果

	s16 AgvMSLocation_s;

	MSD_Category MSDCategory;

	u8 bruce_crossroads_counter;

	u16 zeropointfive;

	Agv_MS_Location MaxRecoder;

	u32 MS_Two_2_One_Vx;
	
}Magn_Sensor_Data_Sturct, *Magn_Sensor_Data_Sturct_P;


extern Magn_Sensor_Data_Sturct_P FMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Ptr;
extern Magn_Sensor_Data_Sturct_P FMSDS_Pre_Ptr;
extern Magn_Sensor_Data_Sturct_P RMSDS_Pre_Ptr;

extern Pattern_Num_Para_P AGV_Pat_Ptr;
extern Pattern_Num_Para_P AGV_Pat_Pre_Ptr;

void MSDS_Init(void);
void MSD_Analy(Magn_Sensor_Data_Sturct_P);
void MSD_Show_Bin(u32);
void Show_Resualt_Analy(Magn_Sensor_Data_Sturct_P);
void Get_Pattern_Num(Magn_Sensor_Data_Sturct_P, Magn_Sensor_Data_Sturct_P, Pattern_Num_Para_P);
void Get_Pattern_Scale(Magn_Sensor_Data_Sturct_P, Magn_Sensor_Data_Sturct_P, Pattern_Scale_Para_P);
void Get_AngleDirection(Pattern_Num_Para_P, Pattern_Num_Para_P);
void Get_MidpointDirection(Pattern_Num_Para_P, Pattern_Num_Para_P);
void Get_MidpointVxt(Pattern_Num_Para_P, Pattern_Num_Para_P);




#endif


