#include "circle_recoder.h"


#if USE_CIRCLE_INFO_RECODER
CIRCLE_INFO_STRUCT CircleInfoStr;
CIRCLE_INFO_STRUCT_P CircleInfoStrPtr = &CircleInfoStr;
CIRCLE_INFO_STRUCT ReadCircleInfoStr;
CIRCLE_INFO_STRUCT_P ReadCircleInfoStrPtr = &ReadCircleInfoStr;


void Write_CircleInfo_Count(u16 data)
{
	
	EEPROM_Write_Byte(EEPROM_REC_COUNT_ADDR, (data >> 8));
	EEPROM_Write_Byte(EEPROM_REC_COUNT_ADDR + 0x1, (data & 0xFF));
}

u16 Get_CircleInfo_Count(void)
{
	u16 temp = 0;

	temp = (EEPROM_Read_Byte(EEPROM_REC_COUNT_ADDR) << 8);
	temp |= EEPROM_Read_Byte(EEPROM_REC_COUNT_ADDR + 0x1);

	return temp;
}

void Clean_CircleInfo_Count(void)
{
	CircleInfoStrPtr->CircleRecoderCount = 0;

	EEPROM_Write_Byte( EEPROM_REC_COUNT_ADDR, 0x00 );
	EEPROM_Write_Byte( EEPROM_REC_COUNT_ADDR + 0x1, 0x00 );
}

void Clean_CircleInfoStr(CIRCLE_INFO_STRUCT_P ptr)
{
	ptr->Station = 0;
	ptr->REQ_TIME.year = 0;
	ptr->REQ_TIME.month = 0;
	ptr->REQ_TIME.day = 0;
	ptr->REQ_TIME.hour = 0;
	ptr->REQ_TIME.minute = 0;
	ptr->REQ_TIME.second = 0;
	ptr->RESPOND_TIME.year = 0;
	ptr->RESPOND_TIME.month = 0;
	ptr->RESPOND_TIME.day = 0;
	ptr->RESPOND_TIME.hour = 0;
	ptr->RESPOND_TIME.minute = 0;
	ptr->RESPOND_TIME.second = 0;
	ptr->CircleTime.Go2RFIDTime = 0;
	ptr->CircleTime.GoCirTime = 0;
	ptr->CircleTime.EntryTime = 0;
	ptr->CircleTime.CatchTime = 0;
	ptr->CircleTime.ExitTime = 0;
	ptr->CircleTime.WeightTime = 0;
	ptr->CircleTime.BackCirTime = 0;
	ptr->CircleTime.BackTime = 0;
	ptr->CircleTime.BackToOriginTime = 0;
	ptr->ManualOptTime = 0;
	ptr->TakeAwayTime = 0;

	
	
}


void Save_OneCircleInfo(CIRCLE_INFO_STRUCT_P ptr)
{
	u16 StrAddr = EEPROM_REC_CIRCLE_BASE_ADDR + (ptr->CircleRecoderCount - 1) * ONE_CIRCLE_INFO_SIZE_BYTE;
	
	// save station ID
	EEPROM_Write_Byte(StrAddr, ptr->Station);
	StrAddr += STATION_DATA_SIZE;
	
	// save requst time
	EEPROM_Write_Byte(StrAddr, ptr->REQ_TIME.year);
	StrAddr += REQ_TIME_YEAR_SIZE;
	
	EEPROM_Write_Byte(StrAddr, ptr->REQ_TIME.month);
	StrAddr += REQ_TIME_MONTH_SIZE;
	
	EEPROM_Write_Byte(StrAddr, ptr->REQ_TIME.day);
	StrAddr += REQ_TIME_DAY_SIZE;
	
	EEPROM_Write_Byte(StrAddr, ptr->REQ_TIME.hour);
	StrAddr += REQ_TIME_HOUR_SIZE;

	EEPROM_Write_Byte(StrAddr, ptr->REQ_TIME.minute);
	StrAddr += REQ_TIME_MINUTE_SIZE;

	EEPROM_Write_Byte(StrAddr, ptr->REQ_TIME.second);
	StrAddr += REQ_TIME_SECOND_SIZE;

	// save respond time
	EEPROM_Write_Byte(StrAddr, ptr->RESPOND_TIME.year);
	StrAddr += REQ_TIME_YEAR_SIZE;
	
	EEPROM_Write_Byte(StrAddr, ptr->RESPOND_TIME.month);
	StrAddr += REQ_TIME_MONTH_SIZE;
	
	EEPROM_Write_Byte(StrAddr, ptr->RESPOND_TIME.day);
	StrAddr += REQ_TIME_DAY_SIZE;
	
	EEPROM_Write_Byte(StrAddr, ptr->RESPOND_TIME.hour);
	StrAddr += REQ_TIME_HOUR_SIZE;

	EEPROM_Write_Byte(StrAddr, ptr->RESPOND_TIME.minute);
	StrAddr += REQ_TIME_MINUTE_SIZE;

	EEPROM_Write_Byte(StrAddr, ptr->RESPOND_TIME.second);
	StrAddr += REQ_TIME_SECOND_SIZE;

	// save circle time info
	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.Go2RFIDTime  >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.Go2RFIDTime & 0xFF));
	StrAddr += GO2_RFID_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.GoCirTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.GoCirTime & 0xFF));
	StrAddr += GO_CIR_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.EntryTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.EntryTime & 0xFF));
	StrAddr += ENTRY_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.CatchTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.CatchTime & 0xFF));
	StrAddr += CATCH_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.ExitTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.ExitTime & 0xFF));
	StrAddr += EXIT_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.WeightTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.WeightTime & 0xFF));
	StrAddr += WEIGHT_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.BackCirTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.BackCirTime & 0xFF));
	StrAddr += BACK_CIR_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.BackTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.BackTime & 0xFF));
	StrAddr += BACK_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->CircleTime.BackToOriginTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->CircleTime.BackToOriginTime & 0xFF));
	StrAddr += BACK_TO_ORI_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->ManualOptTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->ManualOptTime & 0xFF));
	StrAddr += MANUAL_OPT_TIME_SIZE;

	EEPROM_Write_Byte(StrAddr, (ptr->TakeAwayTime >> 8));
	EEPROM_Write_Byte(StrAddr + 0x1, (ptr->TakeAwayTime & 0xFF));
	StrAddr += TAKE_AWAY_TIME_SIZE;

	Write_CircleInfo_Count(ptr->CircleRecoderCount);
	
	
}


void Save_OneCircleInfo2(CIRCLE_INFO_STRUCT_P ptr)
{
	u16 StrAddr = EEPROM_REC_CIRCLE_BASE_ADDR + (ptr->CircleRecoderCount - 1) * ONE_CIRCLE_INFO_SIZE_BYTE;
	u8 cir = 0;
	u8 temp[6];
	u16 temp1[11];
	// save station ID
	EEPROM_Write_Byte(StrAddr, ptr->Station);
	StrAddr += STATION_DATA_SIZE;

	temp[0] = ptr->REQ_TIME.year;
	temp[1] = ptr->REQ_TIME.month;
	temp[2] = ptr->REQ_TIME.day;
	temp[3] = ptr->REQ_TIME.hour;
	temp[4] = ptr->REQ_TIME.minute;
	temp[5] = ptr->REQ_TIME.second;
	
	for(cir = 0; cir < 6; cir++)
	{
		// save requst time
		EEPROM_Write_Byte(StrAddr, temp[cir]);
		StrAddr += 1;
	}

	temp[0] = ptr->RESPOND_TIME.year;
	temp[1] = ptr->RESPOND_TIME.month;
	temp[2] = ptr->RESPOND_TIME.day;
	temp[3] = ptr->RESPOND_TIME.hour;
	temp[4] = ptr->RESPOND_TIME.minute;
	temp[5] = ptr->RESPOND_TIME.second;

	for(cir = 0; cir < 6; cir++)
	{
		// save respond time
		EEPROM_Write_Byte(StrAddr, temp[cir]);
		StrAddr += 1;
	}

	temp1[0] = ptr->CircleTime.Go2RFIDTime;
	temp1[1] = ptr->CircleTime.GoCirTime;
	temp1[2] = ptr->CircleTime.EntryTime;
	temp1[3] = ptr->CircleTime.CatchTime;
	temp1[4] = ptr->CircleTime.ExitTime;
	temp1[5] = ptr->CircleTime.WeightTime;
	temp1[6] = ptr->CircleTime.BackCirTime;
	temp1[7] = ptr->CircleTime.BackTime;
	temp1[8] = ptr->CircleTime.BackToOriginTime;
	temp1[9] = ptr->ManualOptTime;
	temp1[10] = ptr->TakeAwayTime;

	for(cir = 0; cir < 11; cir++)
	{
		// save circle time info
		EEPROM_Write_Byte(StrAddr, (temp1[cir]  >> 8));
		EEPROM_Write_Byte(StrAddr + 0x1, (temp1[cir] & 0xFF));
		StrAddr += 2;
	}

	Write_CircleInfo_Count(ptr->CircleRecoderCount);
}

void Read_OneCircleInfo(CIRCLE_INFO_STRUCT_P ptr, u16 baseAddr)
{
	u16 temp = 0;

	// read station
	ptr->Station = EEPROM_Read_Byte(baseAddr);
	baseAddr += STATION_DATA_SIZE;

	// read requst time
	ptr->REQ_TIME.year = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_YEAR_SIZE;
	
	ptr->REQ_TIME.month = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_MONTH_SIZE;

	ptr->REQ_TIME.day = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_DAY_SIZE;

	ptr->REQ_TIME.hour = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_HOUR_SIZE;

	ptr->REQ_TIME.minute = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_MINUTE_SIZE;

	ptr->REQ_TIME.second = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_SECOND_SIZE;

	// read respond time
	ptr->RESPOND_TIME.year = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_YEAR_SIZE;
	
	ptr->RESPOND_TIME.month = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_MONTH_SIZE;

	ptr->RESPOND_TIME.day = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_DAY_SIZE;

	ptr->RESPOND_TIME.hour = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_HOUR_SIZE;

	ptr->RESPOND_TIME.minute = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_MINUTE_SIZE;

	ptr->RESPOND_TIME.second = EEPROM_Read_Byte(baseAddr);
	baseAddr += REQ_TIME_SECOND_SIZE;

	// read circle time info
	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.Go2RFIDTime = temp;
	baseAddr += GO2_RFID_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.GoCirTime = temp;
	baseAddr += GO_CIR_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.EntryTime = temp;
	baseAddr += ENTRY_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.CatchTime = temp;
	baseAddr += CATCH_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.ExitTime = temp;
	baseAddr += EXIT_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.WeightTime = temp;
	baseAddr += WEIGHT_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.BackCirTime = temp;
	baseAddr += BACK_CIR_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.BackTime = temp;
	baseAddr += BACK_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->CircleTime.BackToOriginTime = temp;
	baseAddr += BACK_TO_ORI_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->ManualOptTime = temp;
	baseAddr += MANUAL_OPT_TIME_SIZE;

	temp = (EEPROM_Read_Byte(baseAddr) << 8);
	temp |= EEPROM_Read_Byte(baseAddr + 0x1);
	ptr->TakeAwayTime = temp;
	baseAddr += TAKE_AWAY_TIME_SIZE;
	
}

void Read_OneCircleInfo2(CIRCLE_INFO_STRUCT_P ptr, u16 baseAddr)
{
	u16 temp = 0, temp2[11];
	u8 cir, temp1[6];

	// read station
	ptr->Station = EEPROM_Read_Byte(baseAddr);
	baseAddr += STATION_DATA_SIZE;

	// read request time
	for(cir = 0; cir < 6; cir++)
	{
		temp1[cir] = EEPROM_Read_Byte(baseAddr);
		baseAddr += 1;
	}
	ptr->REQ_TIME.year 		= temp1[0];
	ptr->REQ_TIME.month 	= temp1[1];
	ptr->REQ_TIME.day 		= temp1[2];
	ptr->REQ_TIME.hour 		= temp1[3];
	ptr->REQ_TIME.minute 	= temp1[4];
	ptr->REQ_TIME.second 	= temp1[5];

	// read respond time
	for(cir = 0; cir < 6; cir++)
	{
		temp1[cir] = EEPROM_Read_Byte(baseAddr);
		baseAddr += 1;
	}
	ptr->RESPOND_TIME.year 		= temp1[0];
	ptr->RESPOND_TIME.month 	= temp1[1];
	ptr->RESPOND_TIME.day 		= temp1[2];
	ptr->RESPOND_TIME.hour 		= temp1[3];
	ptr->RESPOND_TIME.minute 	= temp1[4];
	ptr->RESPOND_TIME.second 	= temp1[5];

	// read circle info
	for(cir = 0; cir < 11; cir++)
	{
		temp2[cir] = (EEPROM_Read_Byte(baseAddr) << 8);
		temp2[cir] |= EEPROM_Read_Byte(baseAddr + 0x1);
		baseAddr += 2;
	}
	ptr->CircleTime.Go2RFIDTime 		= temp2[0];
	ptr->CircleTime.GoCirTime 			= temp2[1];
	ptr->CircleTime.EntryTime 			= temp2[2];
	ptr->CircleTime.CatchTime 			= temp2[3];
	ptr->CircleTime.ExitTime 			= temp2[4];
	ptr->CircleTime.WeightTime 			= temp2[5];
	ptr->CircleTime.BackCirTime 		= temp2[6];
	ptr->CircleTime.BackTime 			= temp2[7];
	ptr->CircleTime.BackToOriginTime 	= temp2[8];
	ptr->ManualOptTime 					= temp2[9];
	ptr->TakeAwayTime 					= temp2[10];
}

void Show_CircleInfo(CIRCLE_INFO_STRUCT_P ptr)
{
	float temp = 0;
	//printf("Station,Request Time,Respond Time,Go To RFID Time,Go Circle Time,Entry Time,Catch Time,Exit Time,Weight Time,Back Cir Time");
	//printf("工站,请求时间,响应时间,从原点到达RFID,转向,靠近络纱机,抓取纱团,退回RFID,抬高机械手和称重,转向,返回到取物点,回到原点,人工操作,等待龙门抓取纱团\r\n");

	printf("%d,", ptr->Station);
	printf("20%02x-%02x-%02x %02x:%02x:%02x,", ptr->REQ_TIME.year, ptr->REQ_TIME.month, ptr->REQ_TIME.day, ptr->REQ_TIME.hour, ptr->REQ_TIME.minute, ptr->REQ_TIME.second);
	printf("20%02x-%02x-%02x %02x:%02x:%02x,", ptr->RESPOND_TIME.year, ptr->RESPOND_TIME.month, ptr->RESPOND_TIME.day, ptr->RESPOND_TIME.hour, ptr->RESPOND_TIME.minute, ptr->RESPOND_TIME.second);

	temp = ptr->CircleTime.Go2RFIDTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->CircleTime.GoCirTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->CircleTime.EntryTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->CircleTime.CatchTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->CircleTime.ExitTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->CircleTime.WeightTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->CircleTime.BackCirTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->CircleTime.BackTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->CircleTime.BackToOriginTime / 10.0;
	printf("%.1f,", temp);

	temp = ptr->ManualOptTime / 10.0;
	printf("%.1f,", temp);
	temp = ptr->TakeAwayTime / 10.0;
	printf("%.1f\r\n", temp);
	
}

void Show_CircleInfo2(CIRCLE_INFO_STRUCT_P ptr)
{
	float temp = 0;
	
	printf("Station = %d\r\n", ptr->Station);
	printf("REQ_TIME = 20%02x-%02x-%02x %02x:%02x:%02x\r\n", ptr->REQ_TIME.year, ptr->REQ_TIME.month, ptr->REQ_TIME.day, ptr->REQ_TIME.hour, ptr->REQ_TIME.minute, ptr->REQ_TIME.second);
	printf("RESPOND_TIME = 20%02x-%02x-%02x %02x:%02x:%02x\r\n", ptr->RESPOND_TIME.year, ptr->RESPOND_TIME.month, ptr->RESPOND_TIME.day, ptr->RESPOND_TIME.hour, ptr->RESPOND_TIME.minute, ptr->RESPOND_TIME.second);

	printf("Go2RFIDTime: %d\r\n", ptr->CircleTime.Go2RFIDTime);
	printf("GoCirTime: %d\r\n", ptr->CircleTime.GoCirTime);
	printf("EntryTime: %d\r\n", ptr->CircleTime.EntryTime);
	printf("CatchTime: %d\r\n", ptr->CircleTime.CatchTime);
	printf("ExitTime: %d\r\n", ptr->CircleTime.ExitTime);
	printf("WeightTime: %d\r\n", ptr->CircleTime.WeightTime);
	printf("BackCirTime: %d\r\n", ptr->CircleTime.BackCirTime);
	printf("BackTime: %d\r\n", ptr->CircleTime.BackTime);
	printf("BackToOriginTime: %d\r\n", ptr->CircleTime.BackToOriginTime);
	
	printf("ManualOptTime: %d\r\n", ptr->ManualOptTime);
	printf("TakeAwayTime: %d\r\n\r\n", ptr->TakeAwayTime);

}


void Print_CircleInfo(void)
{
	u16 cir = 0;
	
	printf("工站,请求时间,响应时间,从原点到达RFID,转向,靠近络纱机,抓取纱团,退回RFID,抬高机械手和称重,转向,返回到取物点,回到原点,人工操作,等待龙门抓取纱团\r\n");

	for(cir = 0; cir < CircleInfoStrPtr->CircleRecoderCount; cir++)
	{
		Read_OneCircleInfo(ReadCircleInfoStrPtr, (EEPROM_REC_CIRCLE_BASE_ADDR + cir * ONE_CIRCLE_INFO_SIZE_BYTE));
		Show_CircleInfo(ReadCircleInfoStrPtr);
	}

	printf("总数 = %d\r\n", CircleInfoStrPtr->CircleRecoderCount);
}


void CircleInfo_Test(void)
{
	CircleInfoStrPtr->CircleRecoderCount++;
	CircleInfoStrPtr->Station = 1 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->REQ_TIME = BackgroudRTC_Rec;
	CircleInfoStrPtr->RESPOND_TIME = BackgroudRTC_Rec;
	CircleInfoStrPtr->CircleTime.Go2RFIDTime = 82 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->CircleTime.GoCirTime = 53 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->CircleTime.EntryTime = 45 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->CircleTime.CatchTime = 5 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->CircleTime.ExitTime = 51 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->CircleTime.WeightTime = 63 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->CircleTime.BackCirTime = 49 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->CircleTime.BackTime = 142 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->CircleTime.BackToOriginTime = 56 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->ManualOptTime = 135 + CircleInfoStrPtr->CircleRecoderCount;
	CircleInfoStrPtr->TakeAwayTime = 67 + CircleInfoStrPtr->CircleRecoderCount;
	
	Save_OneCircleInfo(CircleInfoStrPtr);
	
}

void CircleInfo_CMD_Handle(u8 data)
{
	if(0x01 == data)
	{
		// 读出数据
		Print_CircleInfo();
	}
	else if(0x02 == data)
	{
		// 清除数据
		Clean_CircleInfo_Count();
		//printf("clean!!!\r\n");
	}
	else if(0x03 == data)
	{
		CircleInfo_Test();
		printf("Add!\r\n");
	}
}

void circle_recoder_init(void)
{
	Clean_CircleInfoStr(CircleInfoStrPtr);
	
	CircleInfoStrPtr->CircleRecoderCount = 0;
	CircleInfoStrPtr->TimeTempRec = 0;
	CircleInfoStrPtr->SaveFlag = 1;
	
	CircleInfoStrPtr->CircleRecoderCount = Get_CircleInfo_Count();
	//printf("CircleRecoderCount = %d\r\n", CircleInfoStrPtr->CircleRecoderCount);
	
}

#endif


