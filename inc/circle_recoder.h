#ifndef __CIRCLE_RECODER_H__
#define __CIRCLE_RECODER_H__

#include "common_include.h"
#include "fiberglas.h"
#include "eeprom.h"

#define USE_CIRCLE_INFO_RECODER			1

#define EEPROM_REC_COUNT_ADDR			0x1
#define REC_COUNT_SIZE					2

#define EEPROM_REC_CIRCLE_BASE_ADDR		EEPROM_REC_COUNT_ADDR + REC_COUNT_SIZE



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
#define CIR_REC_COUNT_SIZE_BYTE			2


#define DATE_INFO_SIZE_BYTE				(REQ_TIME_YEAR_SIZE + REQ_TIME_MONTH_SIZE + REQ_TIME_DAY_SIZE +\
										 REQ_TIME_HOUR_SIZE + REQ_TIME_MINUTE_SIZE + REQ_TIME_SECOND_SIZE)

#define CIR_TIME_SIZE_BYTE				(GO2_RFID_TIME_SIZE + GO_CIR_TIME_SIZE 	+ ENTRY_TIME_SIZE +\
										 CATCH_TIME_SIZE 	+ EXIT_TIME_SIZE	+ WEIGHT_TIME_SIZE +\
										 BACK_CIR_TIME_SIZE	+ BACK_TIME_SIZE	+ BACK_TO_ORI_TIME_SIZE)										

#define ONE_CIRCLE_INFO_SIZE_BYTE		(STATION_DATA_SIZE + DATE_INFO_SIZE_BYTE + DATE_INFO_SIZE_BYTE +\
										 CIR_TIME_SIZE_BYTE + MANUAL_OPT_TIME_SIZE + TAKE_AWAY_TIME_SIZE +\
										 CIR_REC_COUNT_SIZE_BYTE)


typedef struct
{
	u16 Go2RFIDTime;		// ֱ�ߵ���RFID���ʱ��
	u16 GoCirTime;			// ת��ʱ��
	u16 EntryTime;			// ����ʱ��
	u16 CatchTime;			// ȡɴʱ��
	u16 ExitTime;			// �˳�ʱ��
	u16 WeightTime;			// ����ʱ��(��ʱ)
	u16 BackCirTime;		// ����ת��ʱ��
	u16 BackTime;			// �س�ʱ��
	u16 BackToOriginTime;	// ����ԭ��ʱ��
}AGV_Circle_Time_Info, *AGV_Circle_Time_Info_P;

typedef struct
{
	u8 Station;							// ���й�վ
	DateInfo REQ_TIME;					// ����ʱ��
	DateInfo RESPOND_TIME;				// ��Ӧʱ��
	AGV_Circle_Time_Info CircleTime;	// ����ʱ��
	u16 ManualOptTime;					// �˹�����ʱ��
	u16 TakeAwayTime;					// ����ȡ��ʱ��
	u16 CircleRecoderCount;				// ��¼����������
	
	u32 TimeTempRec;					// ��¼����ʱʱ�����
}CIRCLE_INFO_STRUCT, *CIRCLE_INFO_STRUCT_P;


#if USE_CIRCLE_INFO_RECODER

extern CIRCLE_INFO_STRUCT_P CircleInfoStrPtr;
extern CIRCLE_INFO_STRUCT_P ReadCircleInfoStrPtr;

void circle_recoder_init(void);
void CircleInfo_CMD_Handle(u8);
void Save_OneCircleInfo(CIRCLE_INFO_STRUCT_P);
void Save_OneCircleInfo2(CIRCLE_INFO_STRUCT_P);
void Clean_CircleInfoStr(CIRCLE_INFO_STRUCT_P);
void Save_OneCircleInfo3(CIRCLE_INFO_STRUCT_P);


#endif



#endif






