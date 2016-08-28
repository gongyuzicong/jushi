#include "watch_dog.h"
#include "timer_opts.h"


void IWatch_Dog_Init(void)
{
	IWDG_KR_START();
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	IWDG_KR_START();
	IWDG_SetReload(0xFFF);
	Delay_ms(10);
	IWDG_Enable();
}




