#include "magn_sensor.h"

Magn_Sensor_Data_Sturct FMSDS;
Magn_Sensor_Data_Sturct_P FMSDS_Ptr = &FMSDS;

Magn_Sensor_Data_Sturct FMSDS_Pre;
Magn_Sensor_Data_Sturct_P FMSDS_Pre_Ptr = &FMSDS_Pre;

Magn_Sensor_Data_Sturct RMSDS;
Magn_Sensor_Data_Sturct_P RMSDS_Ptr = &RMSDS;

Magn_Sensor_Data_Sturct RMSDS_Pre;
Magn_Sensor_Data_Sturct_P RMSDS_Pre_Ptr = &RMSDS_Pre;

MSD_Functions_Struct MSDF_Opts;
MSD_Functions_Struct_P MSDF_Opts_Ptr = &MSDF_Opts;

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
void My_MSD_Opt(Magn_Sensor_Data_Sturct_P ptr)
{
	u16 tempNumHex = 0x00;
	s16 numDec = 0;
	u8 cir1 = 0, cir2 = 0, bit0Count = 0, bit1Count = 0, stageThree = 0;

	tempNumHex = ptr->MSD_Hex;

	if(0 == (tempNumHex & 0x01))		// 如果一上来就是在左偏移极限位置
	{
		for(cir1 = 0; cir1 < 16; cir1++)
		{
			if(0 == (tempNumHex & 0x01))
			{
				bit0Count++;
			}
			else
			{
				break;
			}

			tempNumHex = (tempNumHex >> 1);
		}

		ptr->BitNum = bit0Count;
		ptr->LeftRemain = 16 - bit0Count;
		ptr->RightRemain = 0;
		
	}
	else if(0xFFFF == tempNumHex)		// 数据是0xFFFF的状态
	{
		ptr->LeftRemain = 0xFF;
		ptr->RightRemain = 0xFF;
		ptr->BitNum = 0;
	}
	else
	{

		for(cir1 = 0; cir1 < 16; cir1++)
		{
			if(1 == (tempNumHex & 0x01))
			{
				if(0 == stageThree)
				{
					bit1Count++;
				}
				else
				{
					break;
				}
			}
			else if(0 == (tempNumHex & 0x01))
			{
				stageThree = 1;

				bit0Count++;
			}
			
			tempNumHex = (tempNumHex >> 1);
		}

		ptr->BitNum = bit0Count;
		ptr->RightRemain = bit1Count;
		ptr->LeftRemain = 16 - bit0Count - bit1Count;
		
	}
	
}

void Show_Opt_MSD(Magn_Sensor_Data_Sturct_P show)
{
	printf("LeftRemain = %d\r\n", show->LeftRemain);
	printf("RightRemain = %d\r\n", show->RightRemain);
	printf("BitNum = %d\r\n", show->BitNum);
	
	switch(show->agvDirection)
	{
		case AgvCenter:
			printf("AgvCenter\r\n");
			break;
			
		case AgvCent2Left:
			printf("AgvCent2Left\r\n");
			break;
			
		case AgvCent2Right:
			printf("AgvCent2Right\r\n");
			break;
			
		case AgvRight2Cent:
			printf("AgvRight2Cent\r\n");
			break;
			
		case AgvLeft2Cent:
			printf("AgvLeft2Cent\r\n");
			break;
			
		case AgvLeft2Right:
			printf("AgvLeft2Right\r\n");
			break;
			
		case AgvRight2Left:
			printf("AgvRight2Left\r\n");
			break;

		default:
			printf("agvDirection = %d\r\n", show->agvDirection);
			break;
			
	}
	
	printf("VelocityXt = %d\r\n", show->VelocityXt);
	printf("AcceleratedXt = %d\r\n", show->AcceleratedXt);
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

void Magn_VandA_Calu(Magn_Sensor_Data_Sturct_P now, Magn_Sensor_Data_Sturct_P pre)
{
	if(AgvInitS != pre->agvDirection)
	{
		if(now->LeftRemain < 5)		//  左边剩余少于5, 本次状态为右偏
		{
			if(pre->LeftRemain <= 5)
			{
				if(now->LeftRemain > pre->LeftRemain)
				{
					now->agvDirection = AgvRight2Cent;
				}
				else
				{
					now->agvDirection = AgvCent2Right;
				}
			}
			else
			{
				now->agvDirection = AgvLeft2Right;
			}
			
		}
		else if(now->RightRemain < 5)	// 右边剩余少于5, 本次状态为左偏
		{
			if(pre->RightRemain <= 5)
			{
				if(now->RightRemain > pre->RightRemain)
				{
					now->agvDirection = AgvLeft2Cent;
				}
				else
				{
					now->agvDirection = AgvCent2Left;
				}
			}
			else
			{
				now->agvDirection = AgvRight2Left;
			}
			
		}
		else		// 中间位置
		{
			now->agvDirection = AgvCenter;
		}

		now->VelocityXt = now->TimeRecoder - pre->TimeRecoder;		// VelocityXt   的值越大, 则速度越小
		now->AcceleratedXt = now->VelocityXt - pre->VelocityXt;		// AcceleratedXt的值越大, 则加速度越小
	}
	else
	{
		if(now->LeftRemain < 5)
		{
			now->agvDirection = AgvCent2Right;
		}
		else if(now->LeftRemain = 5)
		{
			now->agvDirection = AgvCenter;
		}
		else
		{
			now->agvDirection = AgvCent2Left;
		}
	}
	
}

void magn_show(Magn_Sensor_Data_Sturct_P show)
{
	MSD_Show_Bin(show->MSD_Hex);
	Show_Opt_MSD(show);
}

void Magn_Sensor_Scan(void)
{
	#if 0
	
	My_MSD_Opt(FMS_Hex, FSDS_Ptr);
	My_MSD_Opt(RMS_Hex, RSDS_Ptr);

	#else

	static u16 tempFMS = 0x00, tempRMS = 0x00;
	u32 TimeNow = 0;
	
	FMSDS_Ptr->MSD_Hex = FMS_Hex;
	RMSDS_Ptr->MSD_Hex = RMS_Hex;

	if(tempFMS != FMSDS_Ptr->MSD_Hex)
	{
		FMSDS_Pre = FMSDS;
		
		FMSDS_Ptr->TimeRecoder = SystemRunningTime;
		
		tempFMS = FMSDS_Ptr->MSD_Hex;
		//printf("F: ");
		//MSD_Show_Bin(FMSDS_Ptr->MSD_Hex);

		// 1. 分析出采集到的磁传感器数据,并且将信息处理成 (传感器)
		My_MSD_Opt(FMSDS_Ptr);
		Magn_VandA_Calu(FMSDS_Ptr, FMSDS_Pre_Ptr);
		//Show_Opt_MSD(FMSDS_Ptr);
		//printf("\r\n");
	}
	
	
	if(tempRMS != RMSDS_Ptr->MSD_Hex)
	{
		RMSDS_Pre = RMSDS;
		
		RMSDS_Ptr->TimeRecoder = SystemRunningTime;
		
		tempRMS = RMSDS_Ptr->MSD_Hex;
		//printf("R: ");
		//MSD_Show_Bin(RMSDS_Ptr->MSD_Hex);
		My_MSD_Opt(RMSDS_Ptr);
		Magn_VandA_Calu(RMSDS_Ptr, RMSDS_Pre_Ptr);
		//Show_Opt_MSD(RMSDS_Ptr);
		//printf("\r\n");
	}
	
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
	
	FMSDS_Ptr->MSD_Dec = 0x00;

	FMSDS_Ptr->VelocityX = 0x00;

	FMSDS_Ptr->AcceleratedX = 0x00;

	FMSDS_Ptr->LeftRemain = 0;

	FMSDS_Ptr->RightRemain = 0;

	FMSDS_Ptr->BitNum = 0;

	FMSDS_Ptr->agvDirection = AgvInitS;

	*RMSDS_Ptr = *FMSDS_Pre_Ptr = *RMSDS_Pre_Ptr = *FMSDS_Ptr;
	
	MSDF_Opts_Ptr->MY_MSD_Operator = My_MSD_Opt;
	MSDF_Opts_Ptr->MS_Scan = Magn_Sensor_Scan;
	MSDF_Opts_Ptr->MSD_SHOW = My_MSD_Show;
	MSDF_Opts_Ptr->MSD_Test = My_MSD_Test;
	MSDF_Opts_Ptr->MSD_Show_Bin = MSD_Show_Bin;
	MSDF_Opts_Ptr->Show_Opt_MSD = Show_Opt_MSD;
	MSDF_Opts_Ptr->magn_show = magn_show;
}



