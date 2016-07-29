#ifndef __EEPROM_H
#define __EEPROM_H
#include "iic.h"   

void EEPROM_Write_Byte(u8 addr,u8 data);
u8 EEPROM_Read_Byte(u8 addr);

void EEPROM_Write_Data(u8 addr,u8 *databuf,u16 len);
void EEPROM_Read_Data(u8 addr,u8 *databuf,u16 len);

#endif
















