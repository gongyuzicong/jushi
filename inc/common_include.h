#ifndef __COMMON_INCLUDE_H__
#define __COMMON_INCLUDE_H__

#include <stdio.h>
#include "stm32f10x_lib.h"
#include <stddef.h>
#include <stdlib.h>
#include "debug_opts.h"

#define xBitOn(data, x)		(data |= (0x01 << x))
#define xBitOff(data, x)	(data &= ~(0x01 << x))

#define SubAbsV(valu1, valu2)		((valu1 > valu2) ? (valu1 - valu2) : (valu2 - valu1))
#define MaxValu(valu1, valu2)		((valu1 > valu2) ? valu1 : valu2)
#define MinValu(valu1, valu2)		((valu1 < valu2) ? valu1 : valu2)


extern int fputc(int ch, FILE *f);

extern vu32 SystemRunningTime;

#endif
