#ifndef __BATTERY_H__
#define __BATTERY_H__


#include "common_include.h"



typedef struct BatteryInfo
{
	u8 data;
}Battery_Info_Str, *Battery_Info_Str_P;

void Battery_Init(void);
void Get_Voltage(void);


#endif





