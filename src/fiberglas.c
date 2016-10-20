#include "fiberglas.h"
#include "ads1256.h"
#include "rtc.h"
#include "timer_opts.h"
#include "eeprom.h" 
#include <string.h>
#include "lcd.h"

#define FIBERGLAS_COUNT_INFO_ADDR	0x0000
#define FACTORY_ID_ADDR				0x0004
#define AREA_ID_ADDR				0x0005
#define STATION_ID_ADDR				0x0007
#define AGV_ID_ADDR					0x0009

#define FIBERGLAS_SAVE_ADDR_OFFSET	16

#define FIBERGLAS_RECODE_BASE_ADDR	0x2FFF

// 5公斤0.125v

FiberglasInfoStr FiberglasInfo;
FiberglasInfoStr_P FiberglasInfo_Ptr = &FiberglasInfo;

FiberglasInfoOptFunc FiberglasInfoOpt;
FiberglasInfoOptFunc_P FiberglasInfoOpt_Ptr = &FiberglasInfoOpt;

GetWeight_Ctrl getWeightCtrl;
GetWeight_Ctrl_P getWeightCtrl_Ptr = &getWeightCtrl;


double VoltsOffset = 0;

DateInfo BackgroudRTC_Rec;
DateInfo BackgroudRTC_Rec_Hex;

/****************** USE RTC START *******************************/

void Get_RTC_Data(DateInfo_P ptr)
{
	u8 date[3], time[3];
	
	READ_datetime(date, time);
	
	#if 1
	
	ptr->year = date[0] & 0x7F;
	ptr->month = date[1] & 0x1F;
	ptr->day = date[2] & 0x3F;
	
	ptr->hour = time[0] & 0x3F;
	ptr->minute = time[1] & 0x7F;
	ptr->second = time[2] & 0x7F;
	
	#else

	ptr->year = BCD_CHANGE2HEX_U8(date[0] & 0x7F);
	ptr->month = BCD_CHANGE2HEX_U8(date[1] & 0x1F);
	ptr->day = BCD_CHANGE2HEX_U8(date[2] & 0x3F);
	
	ptr->hour = BCD_CHANGE2HEX_U8(time[0] & 0x3F);
	ptr->minute = BCD_CHANGE2HEX_U8(time[1] & 0x7F);
	ptr->second = BCD_CHANGE2HEX_U8(time[2] & 0x7F);

	#endif
	
}

void Read_RTC_Data(void)
{
	static u32 timRec = 0;
	//DateInfo date;
	
	if(Delay_Func(&timRec, 1000))
	{
		timRec = 0;
		Get_RTC_Data(&BackgroudRTC_Rec);

		BackgroudRTC_Rec_Hex.year = BCD_CHANGE2HEX_U8(BackgroudRTC_Rec.year);
		BackgroudRTC_Rec_Hex.month = BCD_CHANGE2HEX_U8(BackgroudRTC_Rec.month);
		BackgroudRTC_Rec_Hex.day= BCD_CHANGE2HEX_U8(BackgroudRTC_Rec.day);
		BackgroudRTC_Rec_Hex.hour = BCD_CHANGE2HEX_U8(BackgroudRTC_Rec.hour);
		BackgroudRTC_Rec_Hex.minute = BCD_CHANGE2HEX_U8(BackgroudRTC_Rec.minute);
		BackgroudRTC_Rec_Hex.second = BCD_CHANGE2HEX_U8(BackgroudRTC_Rec.second);
		
		//printf("20%02x-%02x-%02x, %02x:%02x:%02x\r\n", BackgroudRTC_Rec.year, BackgroudRTC_Rec.month, BackgroudRTC_Rec.day, BackgroudRTC_Rec.hour, BackgroudRTC_Rec.minute, BackgroudRTC_Rec.second);
	}
		
}


/****************** USE RTC END *******************************/


/********************** USE ADS1256 START ***************************************/
void Get_Weight_Offset_Data_One(void)
{
	int Adc = 0;
	
	Adc = ADS1256ReadData( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM );	// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM );	
	VoltsOffset = (Adc * 0.000000598);
	
}

void Get_Weight_Offset_Data(void)
{
	u8 cir = 0;
	int Adc = 0;
	double tempVolts = 0.0;
	
	for(cir = 0; cir < 10; cir++)
	{
		Adc = ADS1256ReadData( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM );
		tempVolts += (Adc * 0.000000598);
		
	}

	VoltsOffset = tempVolts / 10.0;
	//printf("VoltsOffset = %.4lf\r\n", VoltsOffset);
}

void Get_Weight_Data(void)
{
	int Adc;
	double Volts;
	float weight_g;
	//float weight_Kg;
	float offset = 0;
	double volte_re = 0;
	#if 0
	
	for(cir = 0; cir < 8; cir++)
	{

		Adc = ADS1256ReadData( (cir << 4) | ADS1256_MUXN_AINCOM);// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		

		 /*差分采集方式*/
		//Adc = ADS1256ReadData( ADS1256_MUXP_AIN0|ADS1256_MUXN_AIN1); //P = AIN0 ,N = AIN1 差分方式*/
		Volts = Adc*0.000000598;
			

		printf(" %.4fV  ",Volts);
		
	}
	printf("\r\n");
	
	#else
	
	Adc = ADS1256ReadData( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM );	// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM );	
	volte_re = (Adc * 0.000000598);
	Volts = volte_re - VoltsOffset;
	//Volts = (Adc * 0.000000598);
	weight_g = (Volts / 0.000025);
	offset = (((u32)weight_g / 1000) * 12);
	//offset = (((u32)weight_g / 1000) * 12);
	//weight_g += offset;
	//weight_Kg = weight_g / 1000.0;
	//weight = (Volts / 0.000025);
	//weight += (((u32)(Volts / 0.000025) / 1000) * 0.01);
	
	if(weight_g > 0)
	{
		FiberglasInfo_Ptr->weight_H = (u32)(weight_g - offset * 2) / 1000;
		FiberglasInfo_Ptr->weight_L = ((u32)(weight_g - offset * 2) % 1000) / 10;
	}
	else
	{
		FiberglasInfo_Ptr->weight_H = 0x00;
		FiberglasInfo_Ptr->weight_L = 0x00;
	}
	
	//printf("volte_re = %.3lfV, Volts = %.3lfV, VoltsOffset = %.3lfV, offset = %.4f\r\n", volte_re, Volts, VoltsOffset, offset);
	//printf("weight_H = %d, weight_L = %d\r\n\r\n", FiberglasInfo_Ptr->weight_H, FiberglasInfo_Ptr->weight_L);
	
	#endif
}

void Report_Weight_Data(void)
{
	Set_scale_weight(FiberglasInfo_Ptr->weight_H, FiberglasInfo_Ptr->weight_L);
}

void Clean_Weight_Func(void)
{
	FiberglasInfo_Ptr->weight_H = 0x00;
	FiberglasInfo_Ptr->weight_L = 0x00;
	
	Report_Weight_Data();
}

void Scan_Weight_Func(void)
{
	static u32 timRec = 0;
	int Adc;
	double Volts;

	if(1 == getWeightCtrl_Ptr->weightScanEnable)
	{
		if(Delay_Func(&timRec, 500))
		{
			Adc = ADS1256ReadData( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM );	// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM );	
			Volts = (Adc * 0.000000598);
			
			if(Volts - VoltsOffset > 0.005)
			{
				Get_Weight_Data();
			}
			else
			{
				FiberglasInfo_Ptr->weight_H = 0x00;
				FiberglasInfo_Ptr->weight_L = 0x00;
			}

			getWeightCtrl_Ptr->weightUpdate = 1;

			timRec = 0;
		}
		
	}	
	
}


/********************** USE ADS1256 END ***************************************/



/********************** USE EEPROM START ***************************************/

u8 ReadFactoryID(void)
{
	u8 temp = 0x00;
	
	EEPROM_Read_Data(FACTORY_ID_ADDR, &temp, 1);

	if(0x00 == temp)
	{
		printf("ReadFactoryID Error: %04x\r\n", temp);
	}
	else
	{
		//FiberglasInfo_Ptr->factoryID = temp;
	}
	
	return temp;
}

u8 SetFactoryID(u8 factoryID)
{
	u8 flag = 0x00;
	u8 temp = 0x00;
	
	EEPROM_Write_Data(FACTORY_ID_ADDR, &factoryID, 1);

	temp = ReadFactoryID();

	if(temp == factoryID)
	{
		FiberglasInfo_Ptr->factoryID = factoryID;
	}
	else
	{
		printf("SetFactoryID Error\r\n");
		flag = 1;
	}
	
	return flag;
}


u16 ReadAreaID(void)
{
	u8 temp[2] = {0x00, 0x00};
	u16 temp2 = 0x0000;
	
	EEPROM_Read_Data(AREA_ID_ADDR, temp, 2);

	temp2 = (temp[0] << 8) | temp[1];

	if(0x0000 == temp2)
	{
		printf("ReadAreaID Error: %04x\r\n", temp2);
	}
	else
	{
		//FiberglasInfo_Ptr->areaID = temp2;
	}

	return temp2;
}


u8 SetAreaID(u16 areaID)
{
	u8 flag = 0;
	u8 temp[2];
	u16 temp2 = 0x0000;

	temp[0] = (areaID >> 8);
	temp[1] = (areaID & 0x00FF);
	
	EEPROM_Write_Data(AREA_ID_ADDR, temp, 2);
	
	temp2 = ReadAreaID();
	
	if(temp2 == areaID)
	{
		FiberglasInfo_Ptr->areaID = areaID;
	}
	else
	{
		printf("SetAreaID Error\r\n");
		flag = 1;
	}

	return flag;
}

u16 ReadStationID(void)
{
	u8 temp[2] = {0x00, 0x00};
	u16 temp2 = 0x0000;
	
	EEPROM_Read_Data(STATION_ID_ADDR, temp, 2);

	temp2 = (temp[0] << 8) | temp[1];

	if(0x0000 == temp2)
	{
		printf("ReadStationID Error: %04x\r\n", temp2);
	}
	else
	{
		//FiberglasInfo_Ptr->stationID = temp2;
	}

	return temp2;
}


u8 SetStationID(u16 stationID)
{
	u8 flag = 0x00;
	u8 temp[2];
	u16 temp2 = 0x0000;

	temp[0] = (stationID >> 8);
	temp[1] = (stationID & 0x00FF);
	
	EEPROM_Write_Data(STATION_ID_ADDR, temp, 2);

	temp2 = ReadStationID();

	if(temp2 == stationID)
	{
		FiberglasInfo_Ptr->stationID = stationID;
		
	}
	else
	{
		printf("SetStationID Error\r\n");
		flag = 1;
	}

	return flag;
	
}

u16 ReadAgvID(void)
{
	u8 temp[2] = {0x00, 0x00};
	u16 temp2 = 0x0000;
	
	EEPROM_Read_Data(AGV_ID_ADDR, temp, 2);

	temp2 = (temp[0] << 8) | temp[1];

	if(0x0000 == temp2)
	{
		printf("ReadAgvID Error: %04x\r\n", temp2);
	}
	else
	{
		//FiberglasInfo_Ptr->agvID = temp2;
	}

	return temp2;
}


u8 SetAgvID(u16 agvID)
{
	u8 flag = 0x00;
	u8 temp[2];
	u16 temp2 = 0x0000;

	temp[0] = (agvID >> 8);
	temp[1] = (agvID & 0x00FF);
	
	EEPROM_Write_Data(AGV_ID_ADDR, temp, 2);

	temp2 = ReadAgvID();

	if(temp2 == agvID)
	{
		FiberglasInfo_Ptr->agvID = agvID;
	}
	else
	{
		printf("SetStationID Error\r\n");
		flag = 1;
	}

	return flag;
	
}

u32 ReadFiberglasCount(void)
{
	u8 temp[4] = {0x00, 0x00, 0x00, 0x00};
	u16 temp2 = 0x00000000;
	
	EEPROM_Read_Data(FIBERGLAS_COUNT_INFO_ADDR, temp, 4);

	temp2 = (temp[0] << 24) | (temp[1] << 16) | (temp[2] << 8) | temp[3];

	if(0x00000000 == temp2)
	{
		printf("ReadFiberglasCount Error: %04x\r\n", temp2);
	}
	else
	{
		//FiberglasInfo_Ptr->agvID = temp2;
	}

	return temp2;
}

u8 SetFiberglasCount(u32 fiberglasCount)
{
	u8 flag = 0x00;
	u8 temp[4];
	u32 temp2 = 0x00000000;

	temp[0] = (fiberglasCount >> 24);
	temp[1] = (fiberglasCount >> 16) & 0x000000FF;
	temp[2] = (fiberglasCount >> 8) & 0x000000FF;
	temp[3] = (fiberglasCount & 0x000000FF);
	
	EEPROM_Write_Data(AGV_ID_ADDR, temp, 4);

	temp2 = ReadFiberglasCount();

	if(temp2 == fiberglasCount)
	{
		FiberglasInfo_Ptr->fiberglasCount = fiberglasCount;
	}
	else
	{
		printf("SetFiberglasCount Error\r\n");
		flag = 1;
	}

	return flag;
	
}


void ChangeWeightF2I(FiberglasInfoStr_P ptr)
{
	
	
}

void ChangeWeightI2F(FiberglasInfoStr_P ptr)
{
	
	
}

void SaveOneFiberglasInfoTo_EEPROM(FiberglasInfoStr_P ptr, u16 addr)
{
	
	
	
	
}

void ReadOneFiberglasInfoFrom_EEPROM(FiberglasInfoStr_P ptr, u16 addr)
{
	
	
}


void Fiberglas_Init(void)
{
	FiberglasInfo_Ptr->fiberglasCount = 0x00000000;
	FiberglasInfo_Ptr->factoryID = 0x00;
	FiberglasInfo_Ptr->areaID = 0x0000;
	FiberglasInfo_Ptr->stationID = 0x0000;
	FiberglasInfo_Ptr->agvID = 0x0000;
	
	FiberglasInfo_Ptr->machineID = 0x00;
	FiberglasInfo_Ptr->weightInfo = 0;
	FiberglasInfo_Ptr->weight_H = 0x00;
	FiberglasInfo_Ptr->weight_L = 0x00;

	FiberglasInfo_Ptr->dateInfo.year = 0x00;
	FiberglasInfo_Ptr->dateInfo.month = 0x00;
	FiberglasInfo_Ptr->dateInfo.day = 0x00;
	FiberglasInfo_Ptr->dateInfo.hour = 0x00;
	FiberglasInfo_Ptr->dateInfo.minute = 0x00;
	FiberglasInfo_Ptr->dateInfo.second = 0x00;

	
	FiberglasInfo_Ptr->factoryID = ReadFactoryID();
	FiberglasInfo_Ptr->areaID = ReadAreaID();
	FiberglasInfo_Ptr->stationID = ReadStationID();
	FiberglasInfo_Ptr->agvID = ReadAgvID();
	FiberglasInfo_Ptr->fiberglasCount = ReadFiberglasCount();
	
	getWeightCtrl_Ptr->weightScanEnable = 0;
	getWeightCtrl_Ptr->weightUpdate = 0;
}

/********************** USE EEPROM END ***************************************/


