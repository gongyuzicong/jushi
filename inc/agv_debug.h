#ifndef __AGV_DEBUG_H__
#define __AGV_DEBUG_H__

#include "common_include.h"
#include "data_type.h"
#include "motion_control.h"
#include "magn_d_algo.h"
#include "magn_sensor.h"
#include "zigbee.h"



typedef struct AGV_DEBUG_PARA
{
	ReqQueueStr runningInfo;
	u32 goalRFIDnode;
	WalkStep walkingstep;
	u8 	originFlag;
	u8 	rifdAdaptFlag;
	u8 	gear;
	AgvWalkMode agvWalkingMode;
	AgvStatus agvStatus;
	
}AGV_Debug_Str, *AGV_Debug_Str_P;


void AGV_Debug_Func(void);

extern AGV_Debug_Str_P Debug_Para_Ptr;




#endif





