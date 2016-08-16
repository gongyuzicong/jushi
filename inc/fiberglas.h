#ifndef __FIBERGLAS_H__
#define __FIBERGLAS_H__

#include "common_include.h"



typedef struct
{
	u8 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
}DateInfo, *DateInfo_P;

typedef struct
{
	u8 factoryID;
	u16 areaID;
	u16 stationID;
	u16 machineID;
	u16 agvID;
	DateInfo dateInfo;
	DateInfo dateInfoHex;

	u32 fiberglasCount;

	float weightInfo;
	u8 weight_H;
	u8 weight_L;
}FiberglasInfoStr, *FiberglasInfoStr_P;

typedef struct
{
	u8 getWeightFlag;
	float tempWeight;
	u8 tempWeightUpdate;
	u8 weightReportFlag;
}GetWeight_Ctrl, *GetWeight_Ctrl_P;

typedef struct
{
	void (*SetProvinceID)(u8);
	
}FiberglasInfoOptFunc, *FiberglasInfoOptFunc_P;


void Get_Weight_Data(void);
void Read_RTC_Data(void);
void Get_Weight_Offset_Data(void);
void Get_Weight_Offset_Data_One(void);
void Scan_Weight_Func(void);


extern FiberglasInfoStr_P FiberglasInfo_Ptr;
extern DateInfo BackgroudRTC_Rec;
extern DateInfo BackgroudRTC_Rec_Hex;
extern GetWeight_Ctrl_P getWeightCtrl_Ptr;

#endif






