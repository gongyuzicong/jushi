#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__

#include "stm32f10x_lib.h"


#define MAX_CMD_BUF 100
#define MAX_CMD_DATA_LENGTH 2
#define MAX_INFO_DATA_LENGTH 8

#define DataHeadCode 				0x2E


typedef struct
{
	u8 dataType;
	u8 dataLength;
	u8 data[MAX_CMD_DATA_LENGTH];
	u8 checkSum;
}CmdRecvFromFYT, *CmdRecvFromFYT_P;


typedef struct
{
	u8 HeadVernier;
	u8 TailVernier;
	u8 Total;
	u8 MaxNum;
}BufferControl, *BufferControl_P;


/*
typedef enum
{
	RecvNoError = 0x00,
	RecvAtHeadCodeError,
	RecvAtDataTypeError,
	RecvAtDataLengthError,
	RecvAtDataError,
	RecvAtCheckSumError,
	RecvBufFullError,
}RecvCmdFrameError;
*/

typedef enum
{
	RecvAtHeadCode = 0x00,
	RecvAtDataType,
	RecvAtDataLength,
	RecvAtData,
	RecvAtCheckSum,
}RecvFlag;

typedef enum
{
	SendedAtHeadCode = 0x00,
	SendedAtDataType,
	SendedAtDataLength,
	SendedAtData,
	SendedAtCheckSum,
}SendedUsartFlag;

typedef struct
{
	RecvFlag nextRecvFlag;
	vu8 recvAtDataN;
	//u8 recvData;
}RecvUsartControler, *RecvUsartControler_P;

typedef enum
{
	SendAtHeadCode = 0x00,
	SendAtDataType,
	SendAtDataLength,
	SendAtData,
	SendAtCheckSum,
	WaitForAck,
}SendFytStepFlag;

typedef struct
{
	SendFytStepFlag nextSendFlag;
	vu8 sendAtDataN;
}SendUsartControler;

typedef struct
{
	vu8 sendUsartComplete;
	vu8 checkUsartTransmit;
}UsartItCtrl;

typedef struct
{
	u8 headCode;
	u8 dataType;
	u8 dataLength;
	u8 checkSum;
	vu8 data[80];
}DataInfo, *DataInfo_P;


typedef struct
{
	u8 headCode;
	u8 dataType;
	u8 dataLength;
	u8 checkSum;
}DataFixedInfo, *DataFixedInfo_P;


typedef enum
{
	RadarFlag = 0x00,
	AcFlag,
	WheelKeyFlag,
	CdPlayingInfoFlag,
	CdPlayingTimeFlag,
	CdPlayListFlag,
	CdPlayingSongNameFlag,
	CdPlayingSingerNameFlag,
	RadioPlayingInfoFlag,
	RadioTenPresetInfoFlag,
	RadioPresetNumInfoFlag,
	MediaWorkStatusFlag,
	SyncMenuInfoFlag,
	SyncMenuItemInfoFlag,
	UsbPlayingTimeFlag,
	PhoneingTimeFlag,
	SyncStatusFlag,
}InfoTypeFlag, *InfoTypeFlag_P;



typedef struct
{
	u8 Nvic_PreemptionPriority;			// 中断抢占优先级
	u8 Nvic_SubPriority;				// 中断响应优先级
	u8 Nvic_Channel;					// 中断编号 0~59
	u8 Nvic_Group;						// 中断分组 0~4
}Nvic_Paramater, *Nvic_Paramater_P;


typedef struct
{
	u8 humidity_integer;
	u8 humidity_decimals;
	u8 temperature_integer;
	u8 temperature_decimals;
	u8 checkSum;
}Dht11_DataInfoStruct, *Dht11_DataInfoStruct_P;


typedef struct
{
	vu16 ARR_Val;
	vu16 CCR1_Val;
	vu16 CCR2_Val;
	vu16 CCR3_Val;
	vu16 CCR4_Val;
	TIM_TypeDef *TIMx;
}TIMx_PwmOpts_Struct, *TIMx_PwmOpts_Struct_P;


/*
typedef union
{
	void (*func_type_1)(void);
}Host2SlaveCmdCtrlFunc, *Host2SlaveCmdCtrlFunc_P;
*/

#endif




