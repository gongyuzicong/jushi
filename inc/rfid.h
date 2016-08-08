#ifndef __RFID_H__
#define __RFID_H__


#include "common_include.h"
//#include "data_type.h"



typedef enum
{
	Write_1 = 0,
	
}RFID_Write_Type, *RFID_Write_Type_Ptr;



void RFID_Usart_Init(void);
void RFID_Write(u8);
void RFID_Read(void);



#endif



