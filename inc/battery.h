#ifndef __BATTERY_H__
#define __BATTERY_H__


#include "common_include.h"



typedef struct BatteryInfo
{
	u8 BatteryUpdate;
	u8 Battery_H;
	u8 Battery_L;
	
	void (*Scan_Battery)(struct BatteryInfo *);
}Battery_Info_Str, *Battery_Info_Str_P;

void Battery_Init(void);
void Get_Voltage(void);

extern Battery_Info_Str_P BatteryInfoPtr;


#endif





