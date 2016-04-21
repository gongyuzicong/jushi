#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "common_include.h"
#include "data_type.h"

#define MAX_FYT_CMD_BUF_NUM 	10			// ��FYT��������֡��������С: n֡
#define MAX_CAN_DATA_BUF_NUM	10			// ��CAN��������֡��������С: n֡
#define MAX_USART_SEND_BUF_NUM	10			// ���͸�FYT����֡��������С: n֡
#define MAX_CAN_SEND_BUF_NUM	10			// ���͸�CAN����֡��������С: n֡
#define MAX_DHT11_DATA_BUF_NUM	10

#define USE_TC
#define SEND_IN_TC

typedef struct
{
	u8 RQCP0;
	u8 RQCP1;
	u8 RQCP2;
}RqcpCtrlStruct;

extern BufferControl fytCmdBufCtrl;
extern BufferControl canBufCtrl;
extern BufferControl usartBufCtrl;
extern BufferControl sendCanBufCtrl;

extern RqcpCtrlStruct rqcpCtrl;

void BufferOpts_Init(void);
void FytBuf_Append(CmdRecvFromFYT node);
void FytBuf_Delete(void);
void CanBuf_Append(CanRxMsg node);
void CanBufRecvFunc(void);
void CanBuf_Delete(void);
void UsartBuf_Append(DataInfo node);
void UsartBuf_Delete(void);
void CanBufRecvFunc(void);
void SendCanBuf_Append(CanTxMsg node);
void SendCanBuf_Delete(void);
void Buf_Delete_Common(BufferControl *bufCtrl);
CmdRecvFromFYT* get_fytCmd_data(void);
CanRxMsg get_canData(void);
DataInfo* get_sendUsart_Data(void);
CanTxMsg* get_sendToCan_Data(void);
void DHT11DataBuf_Append(Dht11_DataInfoStruct node);
void DHT11DataBuf_Delete(void);
Dht11_DataInfoStruct_P Get_DHT11_Data(void);



#endif







