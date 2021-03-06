#include "buffer.h"
#include "debug_opts.h"

BufferControl fytCmdBufCtrl;
BufferControl canBufCtrl;
BufferControl usartBufCtrl;
BufferControl sendCanBufCtrl;
BufferControl dht11DataBufCtrl;

CmdRecvFromFYT fytCmdBuf[MAX_FYT_CMD_BUF_NUM];
CanRxMsg canDataBuf[MAX_CAN_DATA_BUF_NUM];
DataInfo sendUsartBuf[MAX_USART_SEND_BUF_NUM]; 
CanTxMsg sendToCanBuf[MAX_CAN_SEND_BUF_NUM];
Dht11_DataInfoStruct dht11DataBuf[MAX_DHT11_DATA_BUF_NUM];
RqcpCtrlStruct rqcpCtrl;

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
		
		//printf("sendCanBufCtrl.HeadVernier = %d, sendCanBufCtrl.TailVernier = %d, sendCanBufCtrl.Total = %d\r\n", sendCanBufCtrl.HeadVernier, sendCanBufCtrl.TailVernier, sendCanBufCtrl.Total);
	}
	else
	{
		errorInfo.errorType = recvFytCmdBufOverFlow;
		errorFunc();
		//printf("fytCmdBuf_FULL\r\n");
		return 1;
	}

	return 0;
}

void SendCanBuf_Delete_Common(BufferControl_P bufCtrl)
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
		
		//printf("fytCmdBufCtrl.HeadVernier = %d, fytCmdBufCtrl.TailVernier = %d, fytCmdBufCtrl.Total = %d\r\n", fytCmdBufCtrl.HeadVernier, fytCmdBufCtrl.TailVernier, fytCmdBufCtrl.Total);
	}
}


/**************************************************************/


CmdRecvFromFYT* get_fytCmd_data(void)
{
	CmdRecvFromFYT* fytCmdData;

	fytCmdData = &fytCmdBuf[fytCmdBufCtrl.HeadVernier];

	return fytCmdData;
}

CanRxMsg get_canData(void)
{
	CanRxMsg rxData;

	rxData = canDataBuf[canBufCtrl.HeadVernier];

	return rxData;
}

DataInfo* get_sendUsart_Data(void)
{
	DataInfo* sendUsartData;
	
	sendUsartData = &sendUsartBuf[usartBufCtrl.HeadVernier];

	return sendUsartData;
}

CanTxMsg* get_sendToCan_Data(void)
{
	CanTxMsg* sendToCanData;

	sendToCanData = &sendToCanBuf[sendCanBufCtrl.HeadVernier];

	return sendToCanData;
}

/***************USART1接收buffer******************/
void FytBuf_Append(CmdRecvFromFYT node)
{
	if(fytCmdBufCtrl.Total < MAX_FYT_CMD_BUF_NUM)
	{
		
		fytCmdBuf[fytCmdBufCtrl.TailVernier] = node;
		
		if(fytCmdBufCtrl.TailVernier < MAX_FYT_CMD_BUF_NUM - 1)
		{
			fytCmdBufCtrl.TailVernier++;
		}
		else
		{
			fytCmdBufCtrl.TailVernier = 0;
		}
		
		fytCmdBufCtrl.Total++;
		
		//printf("fytCmdBufCtrl.HeadVernier = %d, fytCmdBufCtrl.TailVernier = %d, fytCmdBufCtrl.Total = %d\r\n", fytCmdBufCtrl.HeadVernier, fytCmdBufCtrl.TailVernier, fytCmdBufCtrl.Total);
	}
	else
	{
		errorInfo.errorType = recvFytCmdBufOverFlow;
		errorFunc();
		//printf("fytCmdBuf_FULL\r\n");
	}

}

void FytBuf_Delete(void)
{
	if(fytCmdBufCtrl.Total > 0)
	{
		if(fytCmdBufCtrl.HeadVernier < MAX_FYT_CMD_BUF_NUM - 1)
		{
			fytCmdBufCtrl.HeadVernier++;
		}
		else
		{
			fytCmdBufCtrl.HeadVernier = 0;
		}
		
		fytCmdBufCtrl.Total--;
		
		//printf("fytCmdBufCtrl.HeadVernier = %d, fytCmdBufCtrl.TailVernier = %d, fytCmdBufCtrl.Total = %d\r\n", fytCmdBufCtrl.HeadVernier, fytCmdBufCtrl.TailVernier, fytCmdBufCtrl.Total);
	}
}


/********************CAN接收buffer**************************/
void CanBuf_Append(CanRxMsg node)
{
	if(canBufCtrl.Total < MAX_CAN_DATA_BUF_NUM)
	{
		
		canDataBuf[canBufCtrl.TailVernier] = node;
		

		if(canBufCtrl.TailVernier < MAX_FYT_CMD_BUF_NUM - 1)
		{
			canBufCtrl.TailVernier++;
		}
		else
		{
			canBufCtrl.TailVernier = 0;
		}
		
		canBufCtrl.Total++;
	}
	else
	{
		errorInfo.errorType = recvCanDataBufOverFlow;
		errorFunc();
	}
}

void CanBuf_Delete(void)
{
	if(canBufCtrl.Total > 0)
	{
		if(canBufCtrl.HeadVernier < MAX_CAN_DATA_BUF_NUM - 1)
		{
			canBufCtrl.HeadVernier++;
		}
		else
		{
			canBufCtrl.HeadVernier = 0;
		}
		
		canBufCtrl.Total--;
		//printf("CanBuf_Delete\r\n");
	}
}



/*****************USART发送buffer***************************/
void ShowUsartBuf(void)
{
	u8 cir = 0;

	for(cir = 0; cir < usartBufCtrl.Total; cir++)
	{
		printf("dataType = %x, dataLength = %x\r\n", sendUsartBuf[usartBufCtrl.HeadVernier + cir].dataType, sendUsartBuf[usartBufCtrl.HeadVernier + cir].dataLength);
	}
}

void UsartBuf_Append(DataInfo node)
{
	if(usartBufCtrl.Total < MAX_USART_SEND_BUF_NUM)
	{
	#if 0
		u8 cir = 0;
		sendUsartBuf[usartBufCtrl.TailVernier].headCode = node.headCode;
		sendUsartBuf[usartBufCtrl.TailVernier].dataType = node.dataType;
		sendUsartBuf[usartBufCtrl.TailVernier].dataLength = node.dataLength;

		for(cir = 0; cir < node.dataLength; cir++)
		{
			sendUsartBuf[usartBufCtrl.TailVernier].data[cir] = node.data[cir];
		}

		sendUsartBuf[usartBufCtrl.TailVernier].checkSum = countCheckSum(node);
	#else
		//node.checkSum = countCheckSum(node);
		sendUsartBuf[usartBufCtrl.TailVernier] = node;
	#endif		
		if(usartBufCtrl.TailVernier < MAX_USART_SEND_BUF_NUM - 1)
		{
			usartBufCtrl.TailVernier++;
		}
		else
		{
			usartBufCtrl.TailVernier = 0;
		}
		
		usartBufCtrl.Total++;
		
		//printf("usartBufCtrl.HeadVernier = %d, usartBufCtrl.TailVernier = %d, usartBufCtrl.Total = %d\r\n", usartBufCtrl.HeadVernier, usartBufCtrl.TailVernier, usartBufCtrl.Total);
		//ShowUsartBuf();
	}
	else
	{
		errorInfo.errorType = recvFytCmdBufOverFlow;
		errorFunc();
		printf("sendUsartBuf_Append_Full\r\n");
	}
}


void UsartBuf_Delete(void)
{
	if(usartBufCtrl.Total > 0)
	{
		if(usartBufCtrl.HeadVernier < MAX_CAN_DATA_BUF_NUM - 1)
		{
			usartBufCtrl.HeadVernier++;
		}
		else
		{
			usartBufCtrl.HeadVernier = 0;
		}
		
		usartBufCtrl.Total--;
		//printf("UsartBuf_Delete\r\n");
		//printf("usartBufCtrl.HeadVernier = %d, usartBufCtrl.TailVernier = %d, usartBufCtrl.Total = %d\r\n", usartBufCtrl.HeadVernier, usartBufCtrl.TailVernier, usartBufCtrl.Total);
	}
}


/*******************CAN发送buffer************************/

void SendCanBuf_Append(CanTxMsg node)
{
	if(sendCanBufCtrl.Total < MAX_CAN_SEND_BUF_NUM)
	{
		
		sendToCanBuf[sendCanBufCtrl.TailVernier] = node;
		
		if(sendCanBufCtrl.TailVernier < MAX_FYT_CMD_BUF_NUM - 1)
		{
			sendCanBufCtrl.TailVernier++;
		}
		else
		{
			sendCanBufCtrl.TailVernier = 0;
		}
		
		sendCanBufCtrl.Total++;
		
		//printf("sendCanBufCtrl.HeadVernier = %d, sendCanBufCtrl.TailVernier = %d, sendCanBufCtrl.Total = %d\r\n", sendCanBufCtrl.HeadVernier, sendCanBufCtrl.TailVernier, sendCanBufCtrl.Total);
	}
	else
	{
		errorInfo.errorType = recvFytCmdBufOverFlow;
		errorFunc();
		//printf("fytCmdBuf_FULL\r\n");
	}

}

void SendCanBuf_Delete(void)
{
	if(sendCanBufCtrl.Total > 0)
	{
		if(sendCanBufCtrl.HeadVernier < MAX_CAN_SEND_BUF_NUM - 1)
		{
			sendCanBufCtrl.HeadVernier++;
		}
		else
		{
			sendCanBufCtrl.HeadVernier = 0;
		}
		
		sendCanBufCtrl.Total--;
		
		//printf("fytCmdBufCtrl.HeadVernier = %d, fytCmdBufCtrl.TailVernier = %d, fytCmdBufCtrl.Total = %d\r\n", fytCmdBufCtrl.HeadVernier, fytCmdBufCtrl.TailVernier, fytCmdBufCtrl.Total);
	}
}


/*******************DHT11数据buffer************************/

void DHT11DataOpts(void)
{
	dht11DataBuf[dht11DataBufCtrl.TailVernier] = *((Dht11_DataInfoStruct_P)tmpNode);
}

void DHT11DataBuf_Append(Dht11_DataInfoStruct node)
{
	tmpNode = &node;
	
	Buf_Append_Common(&dht11DataBufCtrl, DHT11DataOpts);
	//printf("HeadVernier = %d\r\n", dht11DataBufCtrl.HeadVernier);
}

void DHT11DataBuf_Delete(void)
{
	SendCanBuf_Delete_Common(&dht11DataBufCtrl);
	//printf("TailVernier = %d\r\n", dht11DataBufCtrl.TailVernier);
}

Dht11_DataInfoStruct_P Get_DHT11_Data(void)
{
	Dht11_DataInfoStruct_P getData = NULL;

	getData = &dht11DataBuf[dht11DataBufCtrl.HeadVernier];

	DHT11DataBuf_Delete();

	return getData;
}



void BufferOpts_Init(void)
{
	fytCmdBufCtrl.HeadVernier = 0;
	fytCmdBufCtrl.TailVernier = 0;
	fytCmdBufCtrl.Total = 0;
	fytCmdBufCtrl.MaxNum = MAX_FYT_CMD_BUF_NUM;

	canBufCtrl.HeadVernier = 0;
	canBufCtrl.TailVernier = 0;
	canBufCtrl.Total = 0;
	canBufCtrl.MaxNum = MAX_CAN_DATA_BUF_NUM;

	usartBufCtrl.HeadVernier = 0;
	usartBufCtrl.TailVernier = 0;
	usartBufCtrl.Total = 0;
	usartBufCtrl.MaxNum = MAX_USART_SEND_BUF_NUM;

	sendCanBufCtrl.HeadVernier = 0;
	sendCanBufCtrl.TailVernier = 0;
	sendCanBufCtrl.Total = 0;
	sendCanBufCtrl.MaxNum = MAX_CAN_SEND_BUF_NUM;

	dht11DataBufCtrl.HeadVernier = 0;
	dht11DataBufCtrl.TailVernier = 0;
	dht11DataBufCtrl.Total = 0;
	dht11DataBufCtrl.MaxNum = MAX_DHT11_DATA_BUF_NUM;

	rqcpCtrl.RQCP0 = 1;
	rqcpCtrl.RQCP1 = 1;
	rqcpCtrl.RQCP2 = 1;
	
}




