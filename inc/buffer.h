#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "common_include.h"
#include "data_type.h"

#define MAX_FYT_CMD_BUF_NUM 	2			// 从FYT接收数据帧缓存区大小: n帧
#define MAX_CAN_DATA_BUF_NUM	2			// 从CAN接收数据帧缓存区大小: n帧
#define MAX_USART_SEND_BUF_NUM	2			// 发送给FYT数据帧缓存区大小: n帧
#define MAX_CAN_SEND_BUF_NUM	2			// 发送给CAN数据帧缓存区大小: n帧

#define MAX_DHT11_DATA_BUF_NUM	5

#define MAX_NRF_SEND_BUF_NUM	5		// 通过NRF24L01发送数据的缓存区大小
#define MAX_NRF_RECV_BUF_NUM	5		// 通过NRF24L01接收数据的缓存区大小

#define USE_TC
#define SEND_IN_TC

#define MAX_ZIGBEE_QUEUE_NUM	20

typedef struct
{
	u8 RQCP0;
	u8 RQCP1;
	u8 RQCP2;
}RqcpCtrlStruct;


typedef struct
{
	u8 a;
}BufOperaterSturct, *BufOperaterStruct_P;

extern BufferControl fytCmdBufCtrl;
extern BufferControl canBufCtrl;
extern BufferControl usartBufCtrl;
extern BufferControl sendCanBufCtrl;

extern BufferControl zigbeeQueueCtrl;

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
void zigbeeRecvDataBuf_Append(u16);
void zigbeeRecvDataBuf_Delete(void);
void get_zigbeeData(u16 *);



#endif







