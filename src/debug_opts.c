#include "debug_opts.h"

ErrorInfo errorInfo;

void errorFunc(void)
{
	//printf("error_log");
	printf("errorInfo.errorType = %x, errorInfo.errorData = %x\r\n", errorInfo.errorType, errorInfo.errorData);
	
}

void errorFunc2(CanRxMsg RxMessage)
{
	//printf("error_log");
	//errorFunc();
	printf("errorInfo.errorType = %x, errorInfo.errorData = %x\r\n", errorInfo.errorType, errorInfo.errorData);
	
}


void errorFunc4(u8 recv)
{
	//printf("error_log");
	printf("errorInfo.errorType = %x, errorInfo.errorData = %x\r\n", errorInfo.errorType, errorInfo.errorData);
	
}


