#ifndef __ZIGBEE_H__
#define __ZIGBEE_H__


#include "common_include.h"
#include "data_type.h"
#include "motion_control.h"

#define NC_MODE 0x00
#define ZD_MODE 0x01

#define USE_SEND_ZIGBEE

/******************数据和函数定义*********************/
#define baurd_uart1 (u32)19200

typedef struct
{
	u8 receive_end;
	u8 recvValidDataFlag;
	ReqQueueStr runningInfo;
}Zigbee_Info, *Zigbee_Info_P;

typedef enum
{
	NcNone = 0,
	GoodReq,
	GoodReqCancel,
	GetTheGood,
	ArrGoal,
	GoodLeav,
	ZigbeeACK,
	AutoReq,
}NC_Flag;

typedef struct
{
	NC_Flag cmdFlag;
	NC_Flag Cancel_Flag;
}RecvCmdFlag, *RecvCmdFlag_P;

typedef struct
{
	u8 zigbee_ID1;
	u8 zigbee_ID2;
}ZigbeeID_Info;

typedef struct
{
	u8 resendFlag;
	u16 intervalTime_ms;
	u8 resendNum;
	u8 *resendInfo;
	void (*resendCtrlFunc)(void);
}Zigbee_ACK_Info, *Zigbee_ACK_Info_P;



extern Zigbee_Info_P Zigbee_Ptr;
extern RecvCmdFlag_P CMD_Flag_Ptr;
extern Zigbee_ACK_Info_P ZigbeeResendInfo_Ptr;

void Zigbee_Init(void);
u8 SendChar_Zigbee(u8);
void send_N_char(u8 *, u8);
void UART2_REC_IRQ(u8);
void Send_Arrive(void);
void Send_WaitForGoods(void);
void Receive_handle(void);
void Send_GettedGoods3(void);
void Send_Zigbee_LM_ACK(void);
void Send_FiberMachine(void);
void ZB_Data_Analysis(void);

extern u8 receive_state;		//接收完成标志
extern u8 receive_count;		//接收数据计数
extern u8 nc_receive[8];		//接收数据缓存

#endif







