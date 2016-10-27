#ifndef __DEBUG_OPTS_H__
#define __DEBUG_OPTS_H__

#include "common_include.h"
#include "data_type.h"

typedef enum
{
	errorTypeBegin = 0x00,
		
	errorTypeEnd,
}ErrorCodeType;

typedef enum
{
	errorCodeBegin = 0x00,
	
	errorCodeEnd,
}ErrorCodeData;

typedef struct
{
	ErrorCodeType errorType;
	u8 errorData;
}ErrorInfo, *ErrorInfo_P;

void errorFunc(void);
void errorFunc2(CanRxMsg RxMessage);
void errorFunc4(u8 recv);

extern ErrorInfo errorInfo;


#endif





