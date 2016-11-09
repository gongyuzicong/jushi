#ifndef __BATTERY_H__
#define __BATTERY_H__


#include "common_include.h"

typedef struct BatteryValue
{
	u8 Battery_HEX_H;
	u8 Battery_HEX_L;

	u8 Battery_ASCII_H;
	u8 Battery_ASCII_L;
	
}BatteryValueStr, *BatteryValueStr_P;

typedef struct BatteryInfo
{
	u8 BatteryUpdate;
	BatteryValueStr BatteryVal;
	
	void (*Scan_Battery)(struct BatteryInfo *);
}Battery_Info_Str, *Battery_Info_Str_P;

void Battery_Init(void);
void Get_Voltage(void);

extern Battery_Info_Str_P BatteryInfoPtr;


#endif





