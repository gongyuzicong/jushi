#include "magn_sensor.h"

Magn_Sensor_Data_Sturct FMSDS;
Magn_Sensor_Data_Sturct_P FMSDS_Ptr = &FMSDS;

Magn_Sensor_Data_Sturct RMSDS;
Magn_Sensor_Data_Sturct_P RMSDS_Ptr = &RMSDS;

MSD_Functions_Struct MSDF_Opts;
MSD_Functions_Struct_P MSDF_Opts_Ptr = &MSDF_Opts;

vu8 MagnSensorScanTime = 0x00;

u8 station = 0x00;

#define STD_MS_NUM		0x07E0	
#define MS_ERROR		0xFFFF



void MSD_Show_Bin(u32 showNum)
{
	u8 cir = 0;
	printf("%x, ", showNum);
	for(cir = 0; cir < 16; cir++)
	{
		printf("%d", ((showNum >> (15 - cir)) & 0x01));
		
		if(((cir + 1) % 4) == 0)
		{
			printf(" ");
		}
	}

	printf("\r\n");
}


/**********************************
正中间为: 0
左偏为: 正值
右偏为: 负值
***********************************/
void My_MSD_Opt(u16 numHex, Magn_Sensor_Data_Sturct_P ptr)
{
	u16 tempNumHex = 0x00;
	s16 numDec = 0;
	u8 cir = 0, bitCount = 0;

	tempNumHex = ~numHex;
	
	if(tempNumHex != STD_MS_NUM)
	{
		for(cir = 0; cir < 16; cir++)
		{
			if(0 != (tempNumHex & 0x01))
			{
				
				//MSD_Show_Bin(tempNumHex);
				if(0x3F == tempNumHex)		//six bit
				{
					ptr->BitNum = Six_Bit;

					ptr->MSD_Dec = 5 - cir;
					//printf("cir = %d\r\n", cir);
					//printf("1ptr->MSD_Dec = %d\r\n", ptr->MSD_Dec);
				}
				else						// five bit
				{
					ptr->BitNum = Five_Bit;

					if(cir <= 5)
					{
						ptr->MSD_Dec = 6 - cir;
						//printf("cir = %d\r\n", cir);
						//printf("2ptr->MSD_Dec = %d\r\n", ptr->MSD_Dec);
					}
					else if(cir > 5)
					{
						ptr->MSD_Dec = 5 - cir;
						//printf("cir = %d\r\n", cir);
						//printf("3ptr->MSD_Dec = %d\r\n", ptr->MSD_Dec);
					}
				}
				
				break;
			}
			tempNumHex = (tempNumHex >> 1);
		}
	}
	else if(tempNumHex == MS_ERROR)
	{
		// stop counter
		
	}
	else
	{
		ptr->MSD_Dec = 0;
		ptr->BitNum = Six_Bit;
		//printf("4ptr->MSD_Dec = %d\r\n", ptr->MSD_Dec);
	}

	
}



void Show_Opt_MSD(Magn_Sensor_Data_Sturct_P ptr)
{
	printf("Offset = %d\r\n", ptr->MSD_Dec);
	
	if(Six_Bit == ptr->BitNum)
	{
		printf("Six_Bit\r\n");
	}
	else
	{
		printf("Five_Bit\r\n");
	}
	
	printf("MSD_Dec = %d\r\n\r\n", ptr->MSD_Dec);
}

void My_MSD_Show(u32 showNumF, u32 showNumR)
{
	printf("F: ");
	MSD_Show_Bin(showNumF);
	printf("R: ");
	MSD_Show_Bin(showNumR);
	printf("\r\n");
}

void My_MSD_Test(void)
{
	static u16 showNumF = 0xFF, showNumR = 0xFF;

	#if 0
	
	if((showNumF != FMS_Hex) || (showNumR != RMS_Hex))
	{
		showNumF = FMS_Hex;
		showNumR = RMS_Hex;
		My_MSD_Show(showNumF, showNumR);
	}
	
	#else

	#if 0
	
	if((showNumF != FMS_Hex) || (showNumR != RMS_Hex))
	{
		
		if(showNumR != FMS_Hex)
		{
			showNumF = FMS_Hex;
			printf("F: ");
			MSD_Show_Bin(showNumF);
		}
		
		if(showNumR != RMS_Hex)
		{
			showNumR = RMS_Hex;
			printf("R: ");
			MSD_Show_Bin(showNumR);
		}
		
		printf("\r\n");
	}
	
	#else

	#if 1
	if(showNumR != RMS_Hex)
	{
		
		if(showNumR != RMS_Hex)
		{
			showNumR = RMS_Hex;
			printf("R: ");
			MSD_Show_Bin(showNumR);
		}
		
	}
	#else
	if(showNumF != FMS_Hex)
	{
		
		if(showNumR != FMS_Hex)
		{
			showNumF = FMS_Hex;
			printf("F: ");
			MSD_Show_Bin(showNumF);
		}
		
	}
	#endif

	#endif
	
	#endif
	
}

void Magn_VandA_Calu(Magn_Sensor_Data_Sturct_P now)
{
	#if 1
	u8 cycleTime = Max_MAGN_SCAN_TIME;
	static u8 flag = 0x01;
	static Magn_Sensor_Data_Sturct pre = {0, 0, Six_Bit, 0, 0};
	
	if(flag)
	{
		flag--;
	}
	else
	{
		if(0 == now->MSD_Dec)
		{
			
		}
		else
		{
			//printf("now->MSD_Dec = %d, pre.MSD_Dec = %d\r\n", now->MSD_Dec, pre.MSD_Dec);
			now->VelocityX = now->MSD_Dec - pre.MSD_Dec;
			//printf("now->VelocityX = %d, pre.VelocityX = %d\r\n", now->VelocityX, pre.VelocityX);
			now->AcceleratedX = now->VelocityX - pre.VelocityX;
			//printf("VelocityX = %d\r\n", now->VelocityX);
			//printf("AcceleratedX = %d\r\n", now->AcceleratedX);
		}
	}

	pre.AcceleratedX = now->AcceleratedX;
	pre.BitNum = now->BitNum;
	pre.MSD_Dec = now->MSD_Dec;
	pre.MSD_Hex = now->MSD_Hex;
	pre.VelocityX = now->VelocityX;
	#endif
}

void Magn_Sensor_Scan(void)
{
	#if 0
	
	My_MSD_Opt(FMS_Hex, FSDS_Ptr);
	My_MSD_Opt(RMS_Hex, RSDS_Ptr);

	#else

	static u16 tempFMS = 0x00, tempRMS = 0x00;
	
	FMSDS_Ptr->MSD_Hex = FMS_Hex;
	

	if(tempFMS != FMSDS_Ptr->MSD_Hex)
	//if(1)
	{
		tempFMS = FMSDS_Ptr->MSD_Hex;
		//printf("F: ");
		//MSD_Show_Bin(FMSDS_Ptr->MSD_Hex);
		My_MSD_Opt(FMSDS_Ptr->MSD_Hex, FMSDS_Ptr);
		//Show_Opt_MSD(FMSDS_Ptr);
		Magn_VandA_Calu(FMSDS_Ptr);
		//printf("\r\n");
	}
	
	
	/*
	if(tempRMS != RMS_Hex)
	{
		tempRMS = RMS_Hex;
		My_MSD_Opt(RMS_Hex, RMSDS_Ptr);
		
	}
	*/
	
	#endif
}

void Magn_Sensor_Gpio_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);	/*打开APB2总线上的GPIOA时钟*/

	/*设置GPIOA.2和GPIOA.3为推挽输出，最大翻转频率为50MHz*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}


void Magn_Sensor_Init(void)
{
	Magn_Sensor_Gpio_Init();
	
	FMSDS_Ptr->MSD_Hex = 0x00;
	RMSDS_Ptr->MSD_Hex = 0x00;
	
	FMSDS_Ptr->MSD_Dec = 0x00;
	RMSDS_Ptr->MSD_Dec = 0x00;

	FMSDS_Ptr->BitNum = Six_Bit;
	RMSDS_Ptr->BitNum = Six_Bit;

	FMSDS_Ptr->VelocityX = 0x00;
	RMSDS_Ptr->VelocityX = 0x00;

	FMSDS_Ptr->AcceleratedX = 0x00;
	RMSDS_Ptr->AcceleratedX = 0x00;
	
	MSDF_Opts_Ptr->MY_MSD_Operator = My_MSD_Opt;
	MSDF_Opts_Ptr->MS_Scan = Magn_Sensor_Scan;
	MSDF_Opts_Ptr->MSD_SHOW = My_MSD_Show;
	MSDF_Opts_Ptr->MSD_Test = My_MSD_Test;
	MSDF_Opts_Ptr->MSD_Show_Bin = MSD_Show_Bin;
	
}



