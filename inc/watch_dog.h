#ifndef __WATCH_DOG_H__
#define __WATCH_DOG_H__

#include "common_include.h"

#define IWDG_KR_START()		{IWDG->KR = 0x5555;}

#define IWDG_RELOAD()		{IWDG->KR = (u16)0xAAAA;}

#define USE_IWDG			0


void IWatch_Dog_Init(void);



#endif





