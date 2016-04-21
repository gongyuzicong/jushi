#include "dht11_opts.h"
#include "buffer.h"
#include "cfg_gpio.h"
#include "timer_opts.h"

#define DHT11_DATA_OUT_ON 	GPIOA_X_ON(3)
#define DHT11_DATA_OUT_OFF 	GPIOA_X_OFF(3)
#define DHT11_DATA_IN		GPIOA_READ_BITx(3)

void DHT11_Buffer_Init(void)
{
	
}

void DHT11_DataIn_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	             // 浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIOA_X_ON(3);
	DHT11_DATA_OUT_ON;
}

void DHT11_DataOut_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	             
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIOA_X_ON(3);
	DHT11_DATA_OUT_ON;
}

void DHT11_Reset(void)
{
	DHT11_DataOut_Init();
	DHT11_DATA_OUT_OFF;
	Delay_ms(19);			//拉低至少18ms
	DHT11_DATA_OUT_ON;
	Delay_us(30);			//主机拉高20~40us
	
}

u8 DHT11_Check(void)
{
	u8 retry_counter = 0;
	
	DHT11_DataIn_Init();

	while(DHT11_DATA_IN && (retry_counter < 100))	// 总线拉低80us
	{
		retry_counter++;
		
		Delay_us(1);
	}
	
	if(retry_counter >= 100)
	{
		LED4_OFF;
		printf("check_fail1\r\n");
		return 1;
	}
	else
	{
		retry_counter = 0;
		
		while(!(DHT11_DATA_IN) && (retry_counter < 100))
		{
			retry_counter++;
			
			Delay_us(1);
		}

		if(retry_counter >= 100)
		{
			//LED4_OFF;
			printf("check_fail1\r\n");
			return 1;
		}
		else
		{
			
			//LED4_ON;
			//printf("check\r\n");
		}
	}

	return 0;
}

u8 DHT11_Read_Bit(void)
{
	u8 retry_counter = 0;

	while(DHT11_DATA_IN && (retry_counter < 120))	// 等待变为低电平
	{
		retry_counter++;
		
		Delay_us(1);
	}

	if(retry_counter >= 120)
	{
		printf("DHT11_Read_Bit error!\r\n");
	}
	else
	{
		retry_counter = 0;
		
		while(!DHT11_DATA_IN && (retry_counter < 120))	// 等待变为高电平
		{
			retry_counter++;
			
			Delay_us(1);
		}

		Delay_us(40);

		if(DHT11_DATA_IN)		// 如果为1
		{
			//printf("1\r\n");
			return 1;
		}
	}
	
	return 0;
}

u8 DHT11_Read_Byte(void)
{
	u8 cir, getByte = 0;

	for(cir = 0; cir < 8; cir++)
	{
		getByte = getByte << 1;

		getByte |= DHT11_Read_Bit();
	}

	return getByte;
}

u8 clu_checkSum(Dht11_DataInfoStruct node)
{
	u8 checkSum = 0;
	
	checkSum = node.humidity_integer + node.humidity_decimals + node.temperature_integer + node.temperature_decimals;

	return checkSum;
}

u8 DHT11_Get_Data(void)
{
	Dht11_DataInfoStruct tmp;
	
	DHT11_Reset();

	if(0 == DHT11_Check())
	{
		tmp.humidity_integer = DHT11_Read_Byte();
		tmp.humidity_decimals = DHT11_Read_Byte();
		tmp.temperature_integer = DHT11_Read_Byte();
		tmp.temperature_decimals = DHT11_Read_Byte();
		tmp.checkSum = DHT11_Read_Byte();
		
		if(clu_checkSum(tmp) == tmp.checkSum)
		{
			DHT11DataBuf_Append(tmp);
		}
		else
		{
			printf("checkSum error\r\n");
			return 1;
		}
	}
	
	return 0;
}


void printfData(Dht11_DataInfoStruct_P node)
{
	#if 1
	printf("humidity_integer = %d\r\n", node->humidity_integer);
	//printf("humidity_decimals = %d\r\n", node->humidity_decimals);
	printf("temperature_integer = %d\r\n", node->temperature_integer);
	//printf("temperature_decimals = %d\r\n", node->temperature_decimals);
	//printf("checkSum = %d\r\n", node->checkSum);
	printf("\r\n");
	#endif
}


