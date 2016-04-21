#ifndef __DEBUG_OPTS_H__
#define __DEBUG_OPTS_H__

#include "common_include.h"
#include "data_type.h"

typedef enum
{
	errorTypeBegin = 0x00,
	checkUsartFuncOverflow,
	checkUsart3FuncOverflow,
	h2sCmdFuncOverflow,
	h2sCmdFuncUndefine,
	initLinkedListError,			// 0x05
	usartRecvHeadCodeError,			// 0x06
	usartRecvDataLengthError,		// 0x07
	usartRecvCheckSumError,			// 0x08
	unSyncCtrlFuncOverflow,			// 0x09
	CdCtrlFuncUndefine,
	syncCtrlFuncOverflow,
	syncCtrlFuncUndefine,
	radioCtrlFuncOverflow,
	radioCtrlFuncUndefine,
	ctrlCmdFuncTableOverflow,		// 0x0f
	ctrlCmdFuncTableUndefine,		// 0x10
	reportDataFuncTableOverflow,	
	reportDataFuncTableUndefine,
	dataRegisterFuncOverflow,
	dataRegisterFuncUndefine,
	getCanDataMallocError,			// 0x15
	recvCanInitMallocError,
	sendUsart1FuncOverflow,

	recvCanRadarStatusError,
	recvCanAc_FanAndTempError,
	recvCanTempInfoError,			// 0x1a
	recvCanLeftSeatHeatDataError,
	recvCanRightSeatHeatDataError,
	recvBackWindscreenDataError,
	recvMaxStatusDataError,
	recvAutoStatusDataError,
	recvAcStatusDataError,			// 0x20
	recvMaxAcStatusDataError,
	recvCarInterCirStatusDataError,
	recvWheelKeyInfoError,
	recvRadioPlayingInfoError,
	recvCanAirControlerD6Error,		// 0x25
	recvMediaWorkStatusInfoError,

	recvFytCmdBufOverFlow,
	recvCanDataBufOverFlow,			// 0x28
	
	errorTypeEnd,
}ErrorCodeType;

typedef enum
{
	errorCodeBegin = 0x00,
	
	errorCodeEnd,
}ErrorCodeData;

typedef struct
{
	ErrorCodeType errorType;
	u8 errorData;
}ErrorInfo, *ErrorInfo_P;

void errorFunc(void);
void errorFunc2(CanRxMsg RxMessage);
void errorFunc4(u8 recv);

extern ErrorInfo errorInfo;


#endif





