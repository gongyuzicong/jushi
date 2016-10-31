#include "buffer.h"
#include "debug_opts.h"
#include "motion_control.h"

BufferControl dht11DataBufCtrl;

BufferControl zbDataRecvBufCtrl;
BufferControl_P zbDataRecvBufCtrl_Ptr = &zbDataRecvBufCtrl;

BufferControl zigbeeQueueCtrl;

ReqQueueStr zigbeeInfoQueueBuf[MAX_ZIGBEE_QUEUE_NUM];


u8 zbRecvDataBuf[MAX_ZB_BUF_SIZE];

void *tmpNode;

/*******************Common************************/

u8 Buf_Append_Common(BufferControl_P bufCtrl, void (*dataOptsFunc)(void))
{
	if(bufCtrl->Total < bufCtrl->MaxNum)
	{
		dataOptsFunc();
		
		if(bufCtrl->TailVernier < bufCtrl->MaxNum - 1)
		{
			bufCtrl->TailVernier++;
		}
		else
		{
			bufCtrl->TailVernier = 0;
		}
		
		bufCtrl->Total++;
		
	}
	else
	{
		//errorInfo.errorType = recvFytCmdBufOverFlow;
		errorFunc();
		//printf("fytCmdBuf_FULL\r\n");
		return 1;
	}

	return 0;
}

void Buf_Delete_Common(BufferControl_P bufCtrl)
{
	if(bufCtrl->Total > 0)
	{
		if(bufCtrl->HeadVernier < (bufCtrl->MaxNum - 1))
		{
			bufCtrl->HeadVernier++;
		}
		else
		{
			bufCtrl->HeadVernier = 0;
		}
		
		bufCtrl->Total--;
		
	}
}


/************************ 呼叫队列操作函数: start **************************************/

#if USE_NEW_QUEUE

void Show_Queue_Data(void)
{
	u8 cir;

	printf("\r\nQueue: ");
	
	for(cir = 0; cir < zigbeeQueueCtrl.Total; cir++)
	{
		printf("%d", zigbeeInfoQueueBuf[cir].Req_Station);
		if(TypeManuReq == zigbeeInfoQueueBuf[cir].Req_Type)
		{
			printf("(ma)\t");
		}
		else if(TypeAutoReq == zigbeeInfoQueueBuf[cir].Req_Type)
		{
			printf("(au)\t");
		}
		else if(TypeUnknow == zigbeeInfoQueueBuf[cir].Req_Type)
		{
			printf("(un)\t");
		}
	}

	printf("\r\n");
	
}

u8 searchZigbeeData(u8 data, u8 *index)
{
	u8 cir = 0, flag = 0, searchIndex = 0;

	for(cir = 0; cir < zigbeeQueueCtrl.Total; cir++)
	{
		if(data == zigbeeInfoQueueBuf[cir].Req_Station)
		{
			flag = 1;
			break;
		}
	}

	*index = searchIndex;
	
	return flag;
}


void zigbeeRecvDataBuf_Append(ReqQueueStr data)
{
	if(zigbeeQueueCtrl.Total < MAX_ZIGBEE_QUEUE_NUM)
	{
		if(0 == zigbeeQueueCtrl.Total)
		{
			zigbeeQueueCtrl.TailVernier = 0;
		}
		else
		{
			zigbeeQueueCtrl.TailVernier++;
			
		}
		
		zigbeeInfoQueueBuf[zigbeeQueueCtrl.TailVernier] = data;
		
		zigbeeQueueCtrl.Total++;
		
	}
	else
	{
		//errorInfo.errorType = recvZigbeeBufOverFlow;
		errorFunc();
		
	}

}

void zigbeeRecvDataBuf_Delete(void)
{
	if(zigbeeQueueCtrl.Total > 0)
	{
		if(zigbeeQueueCtrl.Total > 1)
		{
			u8 cir;

			for(cir = 0; cir < zigbeeQueueCtrl.TailVernier; cir++)
			{
				zigbeeInfoQueueBuf[cir].Req_Station = zigbeeInfoQueueBuf[cir + 1].Req_Station;
				zigbeeInfoQueueBuf[cir].Req_Type = zigbeeInfoQueueBuf[cir + 1].Req_Type;
			}
			
			zigbeeQueueCtrl.TailVernier--;
		}
		
		zigbeeQueueCtrl.Total--;
	}
	
}


void zigbeeReqQueue(ReqQueueStr data)
{
	u8 index = 0;

	if(0 == searchZigbeeData(data.Req_Station, &index))		// 还无数据存在
	{
		zigbeeRecvDataBuf_Append(data);
		BuzzerCtrlPtr->buzzerFlag = 1;
	}
}



void zigbeeDeleteQueue(u8 index)
{
	u8 cir;

	for(cir = index; cir < zigbeeQueueCtrl.TailVernier; cir++)
	{
		zigbeeInfoQueueBuf[cir].Req_Station = zigbeeInfoQueueBuf[cir + 1].Req_Station;
		zigbeeInfoQueueBuf[cir].Req_Type = zigbeeInfoQueueBuf[cir + 1].Req_Type;
	}
	
	zigbeeQueueCtrl.TailVernier--;
	zigbeeQueueCtrl.Total--;
}


void zigbeeCancelQueue(u8 data)
{
	u8 index = 0;

	if(1 == searchZigbeeData(data, &index))
	{
		zigbeeDeleteQueue(index);
		BuzzerCtrlPtr->buzzerFlag = 1;
	}
}



void get_zigbeeData(ReqQueueStr_P data)
{
	data->Req_Station = zigbeeInfoQueueBuf[0].Req_Station;
	data->Req_Type = zigbeeInfoQueueBuf[0].Req_Type;
}


#else


u8 searchZigbeeData(u8 data, u8 *index)
{
	u8 cir = 0, flag = 0, searchIndex = 0;

	for(cir = 0; cir < zigbeeQueueCtrl.Total; cir++)
	{

		if(zigbeeQueueCtrl.HeadVernier + cir < MAX_ZIGBEE_QUEUE_NUM)
		{
			searchIndex = zigbeeQueueCtrl.HeadVernier + cir;
		}
		else
		{
			searchIndex = zigbeeQueueCtrl.HeadVernier + cir - MAX_ZIGBEE_QUEUE_NUM;
		}

		if(data == zigbeeInfoQueueBuf[searchIndex].Req_Station)
		{
			flag = 1;
			break;
		}
	}

	*index = searchIndex;
	
	return flag;
}


void zigbeeRecvDataBuf_Append(ReqQueueStr data)
{
	if(zigbeeQueueCtrl.Total < MAX_ZIGBEE_QUEUE_NUM)
	{
		
		zigbeeInfoQueueBuf[zigbeeQueueCtrl.TailVernier] = data;
		
		if(zigbeeQueueCtrl.TailVernier < MAX_ZIGBEE_QUEUE_NUM - 1)
		{
			zigbeeQueueCtrl.TailVernier++;
		}
		else
		{
			zigbeeQueueCtrl.TailVernier = 0;
		}
		
		zigbeeQueueCtrl.Total++;
		
	}
	else
	{
		//errorInfo.errorType = recvZigbeeBufOverFlow;
		errorFunc();
		
	}

}

void zigbeeReqQueue(ReqQueueStr data)
{
	u8 index = 0;

	if(0 == searchZigbeeData(data.Req_Station, &index))		// 还无数据存在
	{
		zigbeeRecvDataBuf_Append(data);
		BuzzerCtrlPtr->buzzerFlag = 1;
	}
}



void zigbeeDeleteQueue(u8 index)
{
	u8 cir = 0;
	
	if(zigbeeQueueCtrl.HeadVernier + zigbeeQueueCtrl.Total > MAX_ZIGBEE_QUEUE_NUM)
	{
		if(index >= zigbeeQueueCtrl.HeadVernier)
		{
			for(cir = index; cir < (zigbeeQueueCtrl.Total + zigbeeQueueCtrl.HeadVernier) - 1; cir++)
			{
				if(index < MAX_ZIGBEE_QUEUE_NUM - 1)
				{
					zigbeeInfoQueueBuf[cir] = zigbeeInfoQueueBuf[cir + 1];
				}
				else if(MAX_ZIGBEE_QUEUE_NUM - 1 == index)
				{
					zigbeeInfoQueueBuf[cir] = zigbeeInfoQueueBuf[0];
				}
				else if(index > MAX_ZIGBEE_QUEUE_NUM - 1)
				{
					zigbeeInfoQueueBuf[cir - MAX_ZIGBEE_QUEUE_NUM] = zigbeeInfoQueueBuf[cir - MAX_ZIGBEE_QUEUE_NUM + 1];
				}
			}
		}
		else
		{
			for(cir = index; cir < zigbeeQueueCtrl.TailVernier - 1; cir++)
			{
				zigbeeInfoQueueBuf[cir] = zigbeeInfoQueueBuf[cir + 1];
			}
		}
		zigbeeQueueCtrl.TailVernier--;
	}
	else if(MAX_ZIGBEE_QUEUE_NUM == zigbeeQueueCtrl.HeadVernier + zigbeeQueueCtrl.Total)
	{
		for(cir = index; cir < (zigbeeQueueCtrl.Total + zigbeeQueueCtrl.HeadVernier) - 1; cir++)
		{
			zigbeeInfoQueueBuf[cir] = zigbeeInfoQueueBuf[cir + 1];
		}
		zigbeeQueueCtrl.TailVernier = MAX_ZIGBEE_QUEUE_NUM - 1;
	}
	else if(zigbeeQueueCtrl.HeadVernier + zigbeeQueueCtrl.Total < MAX_ZIGBEE_QUEUE_NUM)
	{
		for(cir = index; cir < (zigbeeQueueCtrl.Total + zigbeeQueueCtrl.HeadVernier) - 1; cir++)
		{
			zigbeeInfoQueueBuf[cir] = zigbeeInfoQueueBuf[cir + 1];
		}
		zigbeeQueueCtrl.TailVernier--;
	}

	zigbeeQueueCtrl.Total--;
}


void zigbeeCancelQueue(u8 data)
{
	u8 index = 0;

	if(1 == searchZigbeeData(data, &index))
	{
		zigbeeDeleteQueue(index);
		BuzzerCtrlPtr->buzzerFlag = 1;
	}
}

void zigbeeRecvDataBuf_Delete(void)
{
	if(zigbeeQueueCtrl.Total > 0)
	{
		if(zigbeeQueueCtrl.HeadVernier < MAX_ZIGBEE_QUEUE_NUM - 1)
		{
			zigbeeQueueCtrl.HeadVernier++;
		}
		else
		{
			zigbeeQueueCtrl.HeadVernier = 0;
		}
		
		zigbeeQueueCtrl.Total--;
		
	}
}

void get_zigbeeData(ReqQueueStr_P data)
{
	data->Req_Station = zigbeeInfoQueueBuf[zigbeeQueueCtrl.HeadVernier].Req_Station;
	data->Req_Type = zigbeeInfoQueueBuf[zigbeeQueueCtrl.HeadVernier].Req_Type;
}

#endif

/************************ 呼叫队列操作函数: end **************************************/


void ZB_DATA_BUF_APPEND(u8 data)
{
	if(zbDataRecvBufCtrl_Ptr->Total < zbDataRecvBufCtrl_Ptr->MaxNum)
	{
		zbRecvDataBuf[zbDataRecvBufCtrl_Ptr->TailVernier] = data;
		
		if(zbDataRecvBufCtrl_Ptr->TailVernier < zbDataRecvBufCtrl_Ptr->MaxNum - 1)
		{
			zbDataRecvBufCtrl_Ptr->TailVernier++;
		}
		else
		{
			zbDataRecvBufCtrl_Ptr->TailVernier = 0;
		}
		
		zbDataRecvBufCtrl_Ptr->Total++;
		
	}
	else
	{
		//errorInfo.errorType = Usart1RecvBufOverFlow;
		//errorFunc();
	}
	
}

void ZB_DATA_BUF_DELETE(void)
{
	if(zbDataRecvBufCtrl_Ptr->Total > 0)
	{
		if(zbDataRecvBufCtrl_Ptr->HeadVernier < zbDataRecvBufCtrl_Ptr->MaxNum - 1)
		{
			zbDataRecvBufCtrl_Ptr->HeadVernier++;
		}
		else
		{
			zbDataRecvBufCtrl_Ptr->HeadVernier = 0;
		}
		
		zbDataRecvBufCtrl_Ptr->Total--;
		
	}
}

u8 Get_ZB_DATA(void)
{
	
	return (zbRecvDataBuf[zbDataRecvBufCtrl_Ptr->HeadVernier]);
	
}

u8 Get_ZB_DATA_DELETE(void)
{
	u8 data = 0;

	data = zbRecvDataBuf[zbDataRecvBufCtrl_Ptr->HeadVernier];

	ZB_DATA_BUF_DELETE();
	
	return data;
}


/********************************************************************/



void DHT11_Buf_Init(void)
{
	dht11DataBufCtrl.HeadVernier = 0;
	dht11DataBufCtrl.TailVernier = 0;
	dht11DataBufCtrl.Total = 0;
	dht11DataBufCtrl.MaxNum = MAX_DHT11_DATA_BUF_NUM;
}

void Zigbee_Buf_Init(void)
{
	zigbeeQueueCtrl.HeadVernier = 0;
	zigbeeQueueCtrl.TailVernier = 0;
	zigbeeQueueCtrl.Total = 0;
	zigbeeQueueCtrl.MaxNum = MAX_ZIGBEE_QUEUE_NUM;
}

void Zb_Recv_Data_Buf_Init(void)
{
	zbDataRecvBufCtrl_Ptr->HeadVernier = 0;
	zbDataRecvBufCtrl_Ptr->TailVernier = 0;
	zbDataRecvBufCtrl_Ptr->Total = 0;
	zbDataRecvBufCtrl_Ptr->MaxNum = MAX_ZB_BUF_SIZE;

	zbDataRecvBufCtrl_Ptr->Append = ZB_DATA_BUF_APPEND;
	zbDataRecvBufCtrl_Ptr->Delete = ZB_DATA_BUF_DELETE;
	zbDataRecvBufCtrl_Ptr->GetData = Get_ZB_DATA;
	zbDataRecvBufCtrl_Ptr->GetDataDelete = Get_ZB_DATA_DELETE;
}

void BufferOpts_Init(void)
{
	Zigbee_Buf_Init();
	
	Zb_Recv_Data_Buf_Init();
	
	
}


