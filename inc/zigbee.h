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
}NC_Flag;

typedef struct
{
	NC_Flag cmdFlag;
}RecvCmdFlag, *RecvCmdFlag_P;

extern Zigbee_Info_P Zigbee_Ptr;
extern RecvCmdFlag_P CMD_Flag_Ptr;

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
void Send_GettedGoods(void);


#endif







