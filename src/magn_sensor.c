#include "magn_sensor.h"

Magn_Sensor_Data_Sturct FMSDS;
Magn_Sensor_Data_Sturct_P FMSDS_Ptr = &FMSDS;

Magn_Sensor_Data_Sturct RMSDS;
Magn_Sensor_Data_Sturct_P RMSDS_Ptr = &RMSDS;

MSD_Functions_Struct MSDF_Opts;
MSD_Functions_Struct_P MSDF_Opts_Ptr = &MSDF_Opts;

#define STD_MS_NUM		0x07E0	
#define MS_ERROR		0xFFFF


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
			if(0 != ((tempNumHex >> cir) & 0x0001))
			{
				tempNumHex = (tempNumHex >> cir);

				if(0x3F == tempNumHex)		//six bit
				{
					ptr->BitNum = Six_Bit;

					if(cir > 5)		// 往左偏
					{
						ptr->OffsetFlag = Offset_Right;
						ptr->MSD_Dec = cir - 5;
					}
					else			// 往右偏
					{
						ptr->OffsetFlag = Offset_Left;
						ptr->MSD_Dec = 5 - cir;
					}
				}
				else						// five bit
				{
					ptr->BitNum = Five_Bit;

					if(cir <= 5)
					{
						ptr->OffsetFlag = Offset_Left;
						
						if(5 == cir)
						{
							ptr->MSD_Dec = 1;
						}
						else
						{
							ptr->MSD_Dec = 5 - cir;
						}
						
					}
					else if(cir > 5)
					{
						ptr->OffsetFlag = Offset_Right;
						ptr->MSD_Dec = cir - 6;
					}
				}
				
				break;
			}
			
		}
	}
	else if(tempNumHex == MS_ERROR)
	{
		// stop
		
	}
	else
	{
		ptr->OffsetFlag = Offset_None;
		ptr->MSD_Dec = 0;
		ptr->BitNum = Six_Bit;
	}

	
}

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

void Magn_Sensor_Scan(void)
{
	#if 0
	
	My_MSD_Opt(FMS_Hex, FSDS_Ptr);
	My_MSD_Opt(RMS_Hex, RSDS_Ptr);

	#else

	//static u16 tempFMS = 0x00, tempRMS = 0x00;

	if(FMSDS_Ptr->MSD_Hex != FMS_Hex)
	{
		FMSDS_Ptr->MSD_Hex = FMS_Hex;
		printf("F: ");
		MSD_Show_Bin(FMSDS_Ptr->MSD_Hex);
		My_MSD_Opt(FMS_Hex, FMSDS_Ptr);

		/*
		if(Offset_Left == FMSDS_Ptr->OffsetFlag)
		{
			printf("Offset_Left\r\n");
		}
		else if(Offset_Right == FMSDS_Ptr->OffsetFlag)
		{
			printf("Offset_Right\r\n");
		}
		else
		{
			printf("Offset_None\r\n");
		}
		
		if(Six_Bit == FMSDS_Ptr->BitNum)
		{
			printf("Six_Bit\r\n");
		}
		else
		{
			printf("Five_Bit\r\n");
		}
		
		printf("MSD_Dec = %d\r\n\r\n", FMSDS_Ptr->MSD_Dec);
		*/

		
		
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

	FMSDS_Ptr->OffsetFlag = Offset_None;
	RMSDS_Ptr->OffsetFlag = Offset_None;
	
	MSDF_Opts_Ptr->MY_MSD_Operator = My_MSD_Opt;
	MSDF_Opts_Ptr->MS_Scan = Magn_Sensor_Scan;
	MSDF_Opts_Ptr->MSD_SHOW = My_MSD_Show;
	MSDF_Opts_Ptr->MSD_Test = My_MSD_Test;
	
}



