#ifndef __FIBERGLAS_H__
#define __FIBERGLAS_H__

#include "common_include.h"

#include "rtc.h" 




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
	u8 weightScanEnable;
	u8 weightUpdate;
	u8 getWeightCount;
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
void Clean_Weight_Func(void);
void Report_Weight_Data(void);


extern FiberglasInfoStr_P FiberglasInfo_Ptr;
extern DateInfo BackgroudRTC_Rec;
extern DateInfo BackgroudRTC_Rec_Hex;
extern GetWeight_Ctrl_P getWeightCtrl_Ptr;

#endif






