#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "common_include.h"
#include "data_type.h"
#include "motion_control.h"

#define MAX_CAN_DATA_BUF_NUM	2			// 从CAN接收数据帧缓存区大小: n帧
#define MAX_USART_SEND_BUF_NUM	2			// 发送给FYT数据帧缓存区大小: n帧
#define MAX_CAN_SEND_BUF_NUM	2			// 发送给CAN数据帧缓存区大小: n帧

#define MAX_DHT11_DATA_BUF_NUM	5

#define MAX_NRF_SEND_BUF_NUM	5		// 通过NRF24L01发送数据的缓存区大小
#define MAX_NRF_RECV_BUF_NUM	5		// 通过NRF24L01接收数据的缓存区大小

#define USE_TC
#define SEND_IN_TC

#define MAX_ZIGBEE_QUEUE_NUM	20

#define MAX_ZB_BUF_SIZE			16

#define USE_NEW_QUEUE			1

extern BufferControl usartBufCtrl;

extern BufferControl zigbeeQueueCtrl;
extern BufferControl_P zbDataRecvBufCtrl_Ptr;

void BufferOpts_Init(void);
void Buf_Delete_Common(BufferControl *bufCtrl);
CanTxMsg* get_sendToCan_Data(void);

void DHT11DataBuf_Append(Dht11_DataInfoStruct node);
void DHT11DataBuf_Delete(void);
Dht11_DataInfoStruct_P Get_DHT11_Data(void);

void zigbeeRecvDataBuf_Append(ReqQueueStr);
void zigbeeRecvDataBuf_Delete(void);
void get_zigbeeData(ReqQueueStr_P);
void zigbeeReqQueue(ReqQueueStr);
void zigbeeDeleteQueue(u8);
void zigbeeCancelQueue(u8);
u8 searchZigbeeData(u8, u8 *);

void ZB_DATA_BUF_APPEND(u8 data);
void ZB_DATA_BUF_DELETE(void);
u8 Get_ZB_DATA(void);
void Show_Queue_Data(void);


#endif







