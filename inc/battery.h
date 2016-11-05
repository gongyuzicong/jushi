#ifndef __BATTERY_H__
#define __BATTERY_H__


#include "common_include.h"



typedef struct BatteryInfo
{
	u8 Batter_H;
	u8 Batter_L;

	void (*Scan_Battery)(struct BatteryInfo *);
}Battery_Info_Str, *Battery_Info_Str_P;

void Battery_Init(void);
void Get_Voltage(void);


#endif





