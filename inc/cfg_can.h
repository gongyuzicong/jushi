#ifndef __CFG_CAN_H__
#define __CFG_CAN_H__

#include "common_include.h"

#define TransmitTimeOut ((u32)0x500)  


typedef struct
{
	void (*YL_KeySendCan)(void);
	void (*YL_Transmit_Can_Data)(CanTxMsg tmp);
	void (*YL_Transmit_Can_Data_2)(CanTxMsg tmp);
	void (*YL_Transmit_Can_Data_3)(CanTxMsg TxMessage);
	//void (*YL_Transmit_Can_Data_Unwait)(CanTxMsg TxMessage);
	void (*YL_can_loop_test)(void);
	void (*YL_can_test)(void);
}StructCanOpts, *StructCanOpts_P;

void CB_CAN_Config(void);
void CAN_Transmit_Personal(u8 TransmitMailbox, CanTxMsg* TxMessage);
u8 CAN_Transmit_Amend(CanTxMsg* TxMessage);


extern StructCanOpts_P CanOptsPtr;

#endif
