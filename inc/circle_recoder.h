#ifndef __CIRCLE_RECODER_H__
#define __CIRCLE_RECODER_H__

#include "common_include.h"
#include "fiberglas.h"
#include "eeprom.h"

#define USE_CIRCLE_INFO_RECODER			0

#define EEPROM_REC_COUNT_ADDR			0x1
#define REC_COUNT_SIZE					2

#define EEPROM_REC_CIRCLE_BASE_ADDR		EEPROM_REC_COUNT_ADDR + REC_COUNT_SIZE
#define ONE_CIRCLE_INFO_SIZE_BYTE		(1 + 6 + 6 + 9 * 2 + 3 * 2)

#define STATION_DATA_SIZE				1
#define REQ_TIME_YEAR_SIZE				1
#define REQ_TIME_MONTH_SIZE				1
#define REQ_TIME_DAY_SIZE				1
#define REQ_TIME_HOUR_SIZE				1
#define REQ_TIME_MINUTE_SIZE			1
#define REQ_TIME_SECOND_SIZE			1
#define GO2_RFID_TIME_SIZE				2
#define GO_CIR_TIME_SIZE				2
#define ENTRY_TIME_SIZE					2
#define CATCH_TIME_SIZE					2
#define EXIT_TIME_SIZE					2
#define WEIGHT_TIME_SIZE				2
#define BACK_CIR_TIME_SIZE				2
#define BACK_TIME_SIZE					2
#define BACK_TO_ORI_TIME_SIZE			2
#define MANUAL_OPT_TIME_SIZE			2
#define TAKE_AWAY_TIME_SIZE				2


typedef struct
{
	u16 Go2RFIDTime;
	u16 GoCirTime;
	u16 EntryTime;
	u16 CatchTime;
	u16 ExitTime;
	u16 WeightTime;
	u16 BackCirTime;
	u16 BackTime;
	u16 BackToOriginTime;
}AGV_Circle_Time_Info, *AGV_Circle_Time_Info_P;

typedef struct
{
	u8 Station;
	DateInfo REQ_TIME;
	DateInfo RESPOND_TIME;
	AGV_Circle_Time_Info CircleTime;
	u16 ManualOptTime;
	u16 TakeAwayTime;
	u16 CircleRecoderCount;
	
	u32 TimeTempRec;
	u8 SaveFlag;
}CIRCLE_INFO_STRUCT, *CIRCLE_INFO_STRUCT_P;


#if USE_CIRCLE_INFO_RECODER

extern CIRCLE_INFO_STRUCT_P CircleInfoStrPtr;
extern CIRCLE_INFO_STRUCT_P ReadCircleInfoStrPtr;

void circle_recoder_init(void);
void CircleInfo_CMD_Handle(u8);
void Save_OneCircleInfo(CIRCLE_INFO_STRUCT_P);
void Save_OneCircleInfo2(CIRCLE_INFO_STRUCT_P);
void Clean_CircleInfoStr(CIRCLE_INFO_STRUCT_P);


#endif



#endif






