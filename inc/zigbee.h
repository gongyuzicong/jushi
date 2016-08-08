#ifndef __ZIGBEE_H__
#define __ZIGBEE_H__


#include "common_include.h"
#include "data_type.h"

#define NC_MODE 0x00
#define ZD_MODE 0x01


/******************数据和函数定义*********************/
#define baurd_uart1 (u32)19200

typedef struct 
{
  u8 startID;
  u8 length;
  u8 decID0;
  u8 decID1;
  u8 cmd1;
  u8 cmd2;
  u8 buf[255];
}frmFmt; //帧格式

typedef struct
{
	frmFmt frm_1;
	u8 receive_end;
	u8 recvValidDataFlag;
	u16 recvId;
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
}NC_Flag;

typedef struct
{
	NC_Flag cmdFlag;
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

void Protocol_analysis(u8 rec_dat);
void Zigbee_Init(void);
//void send_cmd(u8 cmd[]);
void send_cmd(u8 *);
u8 SendChar_Zigbee(u8);
void Zigbee_Data_Scan(void);
void send_N_char(u8 *, u8);
void UART2_REC_IRQ(u8);
void Receive_handle(void);
void Send_Arrive(void);
void Send_WaitForGoods(void);
void Send_GettedGoods(u8);
void Receive_handle2(void);
void Send_GettedGoods2(u16);
void Send_GettedGoods3(void);

extern u8 receive_state;		//接收完成标志
extern u8 receive_count;		//接收数据计数
extern u8 nc_receive[8];		//接收数据缓存

#endif







