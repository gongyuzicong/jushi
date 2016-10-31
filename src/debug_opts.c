#include "debug_opts.h"



ErrorInfo errorInfo;




void errorFunc(void)
{
	//printf("error_log");
	printf("errorInfo.errorType = %x, errorInfo.errorData = %x\r\n", errorInfo.errorType, errorInfo.errorData);
	
}



