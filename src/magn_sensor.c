#include "magn_sensor.h"
#include "motion_control.h"

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

Agv_Midpoint_Location_Struct AGV_MPLS;
Agv_Midpoint_Location_Struct_P AGV_MPLS_Ptr = &AGV_MPLS;


u8 station = 0x00;

#define STD_MS_NUM		0x07E0	
#define MS_ERROR		0xFFFF

#define CROSSROAD		0x8001

void MSD_Show_Bin(u32 showNum)
{
	u8 cir = 0;
	//printf("%x, ", showNum);
	for(cir = 0; cir < 16; cir++)
	{
		printf("%d", ((showNum >> (15 - cir)) & 0x01));
		
		if(((cir + 1) % 4) == 0)
		{
			printf(" ");
		}
	}

	printf(",\t");
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

	if(0 == (tempNumHex & 0x8001))		// 如果是LINE状态
	{
		ptr->BitNum = 10;
		ptr->RightRemain = 3;
		ptr->LeftRemain = 3;
	}
	else if(0xFFFF == tempNumHex)					// 如果是跑偏了的状态
	{
		ptr->BitNum = 16;
		ptr->RightRemain = 0;
		ptr->LeftRemain = 0;
	}
	else
	{
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
	
	
}

#if 0

void Check_MSD_Category(Magn_Sensor_Data_Sturct_P ptr)
{	
	if((3 == ptr->LeftRemain) && (3 == ptr->RightRemain))
	//if(ptr->LeftRemain == ptr->RightRemain)
	{
		ptr->MSDCategory = MSD_LINE;
		
		if(goStraightStatus == ctrlParasPtr->agvStatus)
		{
			FMSDS_Ptr->bruce_crossroads_counter++;
		}
		else
		{
			FMSDS_Ptr->bruce_crossroads_counter--;
		}
		
	}
	else if((0 == ptr->LeftRemain) && (0 == ptr->RightRemain))
	{
		ptr->MSDCategory = MSD_OUTSIDE;
	}
	else
	{
		ptr->MSDCategory = MSD_NORMAL;
	}
	
}

#else



#endif

void Show_Check_MSD_Category(Magn_Sensor_Data_Sturct_P ptr)
{
	switch(ptr->MSDCategory)
	{
		case MSD_LINE:
			printf("MSD_LIN,\t");
			break;

		case MSD_OUTSIDE:
			printf("MSD_OUT,\t");
			break;

		case MSD_NORMAL:
			printf("MSD_NOR,\t");
			break;
	}
	
}

void Show_My_MSD_Opt(Magn_Sensor_Data_Sturct_P show)
{
	printf("%2d, ", show->LeftRemain);
	printf("%2d, ", show->BitNum);
	printf("%2d\t", show->RightRemain);
}

#if 0
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
			now->agvDirection = AgvNone;
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
		else if(now->LeftRemain == 5)
		{
			now->agvDirection = AgvNone;
		}
		else
		{
			now->agvDirection = AgvCent2Left;
		}
	}
	
}

#endif


#if 0

void Check_Magn_Location(Magn_Sensor_Data_Sturct_P now, Magn_Sensor_Data_Sturct_P pre)
{
	
	if(MSD_NORMAL == now->MSDCategory)	
	{
		if(now->BitNum >= 3)
		{
			if(now->LeftRemain == now->RightRemain)
			{
				now->AgvMSLocation = Agv_MS_Center;
				now->AgvMSLocation_s = 0;
			}
			else if(now->LeftRemain == 5)
			{
				if(now->LeftRemain < now->RightRemain)
				{
					now->AgvMSLocation = Agv_MS_Right_Begin + (now->RightRemain - now->LeftRemain);
					now->AgvMSLocation_s = now->AgvMSLocation - Agv_MS_Right_Begin;
				}
				else
				{
					printf("error: LeftRemain = %d, RightRemain = %d\r\n", now->LeftRemain, now->RightRemain);
				}
			}
			else if(now->LeftRemain < 5)
			{
				if(now->RightRemain > (10 - now->LeftRemain))		//按照右的算
				{
					now->AgvMSLocation = Agv_MS_Right_Begin + (now->RightRemain - 5);
					now->AgvMSLocation_s = (now->RightRemain - now->LeftRemain);
				}
				else		//按照左的算
				{
					now->AgvMSLocation = Agv_MS_Right_Begin + (5 - now->LeftRemain);
					now->AgvMSLocation_s = now->AgvMSLocation - Agv_MS_Right_Begin;
				}
			}
			else if(now->LeftRemain > 5)
			{
				if(now->RightRemain > (10 - now->LeftRemain))		//按照右的算
				{
					now->AgvMSLocation = Agv_MS_Left_Begin + (now->LeftRemain - 5);
					now->AgvMSLocation_s = -(now->AgvMSLocation - Agv_MS_Left_Begin);
				}
				else		//按照左的算
				{
					now->AgvMSLocation = Agv_MS_Left_Begin + (5 - now->RightRemain);
					now->AgvMSLocation_s = now->AgvMSLocation - Agv_MS_Left_Begin;
				}
			}
		}
		
	}
	else if(MSD_OUTSIDE == now->MSDCategory)
	{
		if((pre->AgvMSLocation > Agv_MS_Left_8) && (pre->AgvMSLocation < Agv_MS_Left_End))
		{
			now->AgvMSLocation = Agv_MS_Left_Outside;
			now->AgvMSLocation_s = 0xFF;
		}
		else if((pre->AgvMSLocation > Agv_MS_Right_8) && (pre->AgvMSLocation < Agv_MS_Right_End))
		{
			now->AgvMSLocation = Agv_MS_Right_Outside;
			now->AgvMSLocation_s = 0xFF;
		}
		else
		{
			now->AgvMSLocation = Agv_MS_Undefine;
			now->AgvMSLocation_s = 0xFF;
		}
	}

	
}

#endif


void Show_Check_Magn_Location(Magn_Sensor_Data_Sturct_P now)
{
	switch(now->AgvMSLocation)
	{
		case AgvInits:
			printf("MSInit");
			break;
		
		case Agv_MS_Center:
			printf("MS_Cen");
			break;

		case Agv_MS_Left_1:
			printf("MS_L1 ");
			break;

		case Agv_MS_Left_2:
			printf("MS_L2 ");
			break;

		case Agv_MS_Left_3:
			printf("MS_L3 ");
			break;

		case Agv_MS_Left_4:
			printf("MS_L4 ");
			break;

		case Agv_MS_Left_5:
			printf("MS_L5 ");
			break;

		case Agv_MS_Left_6:
			printf("MS_L6 ");
			break;

		case Agv_MS_Left_7:
			printf("MS_L7 ");
			break;

		case Agv_MS_Left_8:
			printf("MS_L8 ");
			break;

		case Agv_MS_Left_9:
			printf("MS_L9 ");
			break;

		case Agv_MS_Left_10:
			printf("MS_L10");
			break;

		case Agv_MS_Left_End:
			printf("MS_LE ");
			break;
			
		case Agv_MS_Right_1:
			printf("MS_R1 ");
			break;

		case Agv_MS_Right_2:
			printf("MS_R2 ");
			break;

		case Agv_MS_Right_3:
			printf("MS_R3 ");
			break;

		case Agv_MS_Right_4:
			printf("MS_R4 ");
			break;

		case Agv_MS_Right_5:
			printf("MS_R5 ");
			break; 

		case Agv_MS_Right_6:
			printf("MS_R6 ");
			break; 

		case Agv_MS_Right_7:
			printf("MS_R7 ");
			break; 

		case Agv_MS_Right_8:
			printf("MS_R8 ");
			break; 

		case Agv_MS_Right_9:
			printf("MS_R9 ");
			break; 

		case Agv_MS_Right_10:
			printf("MS_R10");
			break;

		case Agv_MS_Right_End:
			printf("MS_RE ");
			break;

		case Agv_MS_Right_Outside:
			printf("MS_RO ");
			break;

		case Agv_MS_Left_Outside:
			printf("MS_LO ");
			break;
	}
	printf(",\t");
	printf("MS_Ls = %3d,\t", now->AgvMSLocation_s);
}

void Check_Magn_Direction(Magn_Sensor_Data_Sturct_P now, Magn_Sensor_Data_Sturct_P pre)
{
	if((now->AgvMSLocation > Agv_MS_Left_End) && (now->AgvMSLocation < Agv_MS_Center))
	{
		if((pre->AgvMSLocation > Agv_MS_Left_End) && (pre->AgvMSLocation < Agv_MS_Center))
		{
			if(now->AgvMSLocation > pre->AgvMSLocation)
			{
				now->agvDirection = AgvCent2Left;
			}
			else if(now->AgvMSLocation < pre->AgvMSLocation)
			{
				now->agvDirection = AgvLeft2Cent;
			}
			else
			{
				now->agvDirection = AgvNone;
			}
		}
		else if((pre->AgvMSLocation > Agv_MS_Center) && (pre->AgvMSLocation < Agv_MS_Right_End))
		{
			now->agvDirection = AgvRight2Left;
		}
	}
	else if((now->AgvMSLocation > Agv_MS_Center) && (now->AgvMSLocation < Agv_MS_Right_End))
	{
		if((pre->AgvMSLocation > Agv_MS_Center) && (pre->AgvMSLocation < Agv_MS_Right_End))
		{
			if(now->AgvMSLocation > pre->AgvMSLocation)
			{
				now->agvDirection = AgvCent2Right;
			}
			else if(now->AgvMSLocation < pre->AgvMSLocation)
			{
				now->agvDirection = AgvRight2Cent;
			}
			else
			{
				now->agvDirection = AgvNone;
			}
		}
		else if((pre->AgvMSLocation > Agv_MS_Left_End) && (pre->AgvMSLocation < Agv_MS_Left_End))
		{
			now->agvDirection = AgvLeft2Right;
		}
	}
	else if(Agv_MS_Center == now->AgvMSLocation)
	{
		if((pre->AgvMSLocation > Agv_MS_Center) && (pre->AgvMSLocation < Agv_MS_Right_End))
		{
			now->agvDirection = AgvCent2Right;
		}
		else if((pre->AgvMSLocation > Agv_MS_Left_End) && (pre->AgvMSLocation < Agv_MS_Center))
		{
			now->agvDirection = AgvCent2Left;
		}
	}
	else if((Agv_MS_Left_Outside == now->AgvMSLocation) || (Agv_MS_Right_Outside == now->AgvMSLocation))
	{
		now->agvDirection = AgvInitS;
	}
	
}


void Show_Check_Magn_Direction(Magn_Sensor_Data_Sturct_P now)
{
	switch(now->agvDirection)
	{
		case AgvInitS:
			printf("DInit");
			break;
			
		case AgvNone:
			printf("DNone");
			break;

		case AgvCent2Left:
			printf("D_C2L");
			break;

		case AgvCent2Right:
			printf("D_C2R");
			break;

		case AgvRight2Cent:
			printf("D_R2C");
			break;

		case AgvLeft2Cent:
			printf("D_L2C");
			break;

		case AgvLeft2Right:
			printf("D_L2R");
			break;

		case AgvRight2Left:
			printf("D_R2L");
			break;
	}

	printf(",\t");
}


void Check_Agv_Location(Agv_Midpoint_Location_Struct_P amls, Magn_Sensor_Data_Sturct_P front, Magn_Sensor_Data_Sturct_P rear)
{
	if(Agv_MS_Center == front->AgvMSLocation)
	{
		if(Agv_MS_Center == rear->AgvMSLocation)
		{
			amls->AgvMPLocation = Agv_MP_Center;
		}
		else if((rear->AgvMSLocation > Agv_MS_Left_End) && (rear->AgvMSLocation < Agv_MS_Right_End))
		{
			switch(rear->AgvMSLocation)
			{
				case Agv_MS_Left_1:
					amls->AgvMPLocation = Agv_MP_Left_1;
					break;

				case Agv_MS_Left_2:
					amls->AgvMPLocation = Agv_MP_Left_2;
					break;

				case Agv_MS_Left_3:
					amls->AgvMPLocation = Agv_MP_Left_3;
					break;

				case Agv_MS_Left_4:
					amls->AgvMPLocation = Agv_MP_Left_4;
					break;

				case Agv_MS_Left_5:
					amls->AgvMPLocation = Agv_MP_Left_5;
					break;

				case Agv_MS_Left_6:
					amls->AgvMPLocation = Agv_MP_Left_6;
					break;

				case Agv_MS_Left_7:
					amls->AgvMPLocation = Agv_MP_Left_7;
					break;

				case Agv_MS_Left_8:
					amls->AgvMPLocation = Agv_MP_Left_8;
					break;
					
			}
		}
	}
	else if((front->AgvMSLocation > Agv_MS_Left_End) && (front->AgvMSLocation < Agv_MS_Right_End))
	{
		
	}
	else
	{
		amls->AgvMPLocation = Agv_MP_Unknow;
	}
}

void Show_Check_Agv_Location(Agv_Midpoint_Location_Struct_P amls)
{
	switch(amls->AgvMPLocation)
	{
		case Agv_MP_Unknow:
			printf("Agv_MP_Unknow\r\n");
			break;

		case Agv_MP_Center:
			printf("Agv_MP_Unknow\r\n");
			break;

		case Agv_MP_Left_1:
			printf("Agv_MP_Left_1\r\n");
			break;

		case Agv_MP_Left_2:
			printf("Agv_MP_Left_2\r\n");
			break;

		case Agv_MP_Left_3:
			printf("Agv_MP_Left_3\r\n");
			break;

		case Agv_MP_Left_4:
			printf("Agv_MP_Left_4\r\n");
			break;

		case Agv_MP_Left_5:
			printf("Agv_MP_Left_5\r\n");
			break;

		case Agv_MP_Left_6:
			printf("Agv_MP_Left_6\r\n");
			break;

		case Agv_MP_Left_7:
			printf("Agv_MP_Left_7\r\n");
			break;

		case Agv_MP_Left_8:
			printf("Agv_MP_Left_8\r\n");
			break;

		case Agv_MP_Right_1:
			printf("Agv_MP_Right_1\r\n");
			break;

		case Agv_MP_Right_2:
			printf("Agv_MP_Right_2\r\n");
			break;

		case Agv_MP_Right_3:
			printf("Agv_MP_Right_3\r\n");
			break;

		case Agv_MP_Right_4:
			printf("Agv_MP_Right_4\r\n");
			break;

		case Agv_MP_Right_5:
			printf("Agv_MP_Right_5\r\n");
			break;

		case Agv_MP_Right_6:
			printf("Agv_MP_Right_6\r\n");
			break;

		case Agv_MP_Right_7:
			printf("Agv_MP_Right_7\r\n");
			break;

		case Agv_MP_Right_8:
			printf("Agv_MP_Right_8\r\n");
			break;
	}
}

void Check_Agv_Location_S(Agv_Midpoint_Location_Struct_P amls, Magn_Sensor_Data_Sturct_P front, Magn_Sensor_Data_Sturct_P rear)
{
	amls->AgvMPLocation_s = front->AgvMSLocation_s - rear->AgvMSLocation_s;
}

void Show_Check_Agv_Location_S(Agv_Midpoint_Location_Struct_P amls)
{
	printf("MPL_s = %d,\t", amls->AgvMPLocation_s);
}

void Magn_VelocityXt_Clau(Magn_Sensor_Data_Sturct_P now, Magn_Sensor_Data_Sturct_P pre)
{
	if(pre->TimeRecoder != 0)
	{
		now->VelocityXt = now->TimeRecoder - pre->TimeRecoder;	// VelocityXt   的值越大, 则速度越小(因为两次数据变化的时间间隔长)
	}
}

void Show_Magn_VelocityXt_Clau(Magn_Sensor_Data_Sturct_P now)
{
	printf("VXt = %d,\t", now->VelocityXt);
}

void Magn_AcceleratedXt_Clau(Magn_Sensor_Data_Sturct_P now, Magn_Sensor_Data_Sturct_P pre)
{
	if(pre->VelocityXt != 0)
	{
		now->AcceleratedXt = now->VelocityXt - pre->VelocityXt;	// AcceleratedXt的值越大, 则加速度越小(因为两次速度数据变化的时间间隔长)
	}
}

void Show_Magn_AcceleratedXt_Clau(Magn_Sensor_Data_Sturct_P now)
{
	printf("AXt = %d,\t", now->AcceleratedXt);
}


void magn_show(Magn_Sensor_Data_Sturct_P show)
{
	MSD_Show_Bin(show->MSD_Hex);
	Show_My_MSD_Opt(show);
}

void Show_Infomation(void)
{
	printf("F: ");
	MSD_Show_Bin(FMSDS_Ptr->MSD_Hex);
	printf("LMD = %2d,\t", ctrlParasPtr->leftMotorSettedSpeed);
	printf("RMD = %2d,\t", ctrlParasPtr->rightMotorSettedSpeed);
	printf("f = %d,\t", ctrlParasPtr->comflag);
	Show_Check_MSD_Category(FMSDS_Ptr);
	//Show_My_MSD_Opt(FMSDS_Ptr);
	Show_Check_Magn_Location(FMSDS_Ptr);
	Show_Check_Magn_Direction(FMSDS_Ptr);
	//Show_Magn_VelocityXt_Clau(FMSDS_Ptr);
	//Show_Magn_AcceleratedXt_Clau(FMSDS_Ptr);
	printf("\t");

	printf("R: ");
	MSD_Show_Bin(RMSDS_Ptr->MSD_Hex);
	Show_Check_MSD_Category(RMSDS_Ptr);
	//Show_My_MSD_Opt(RMSDS_Ptr);
	Show_Check_Magn_Location(RMSDS_Ptr);
	Show_Check_Magn_Direction(RMSDS_Ptr);
	//Show_Magn_VelocityXt_Clau(RMSDS_Ptr);
	//Show_Magn_AcceleratedXt_Clau(RMSDS_Ptr);
	
	//Check_Agv_Location_S(AGV_MPLS_Ptr, FMSDS_Ptr, RMSDS_Ptr);
	//Show_Check_Agv_Location_S(AGV_MPLS_Ptr);
	printf("\r\n");
}

u16 hex_reload(u16 hex)
{
	u8 cir = 0;
	u16 temp = 0x00;

	for(cir = 0; cir < 16; cir++)
	{
		temp |= ((hex >> cir) & 0x01) << (15 - cir);
	}

	return temp;
}


#if 0
void Magn_Sensor_Scan(void)
{
	#if 0
	
	My_MSD_Opt(FMS_Hex, FSDS_Ptr);
	My_MSD_Opt(RMS_Hex, RSDS_Ptr);

	#else

	static u16 tempFMS = 0x00, tempRMS = 0x00;
	u32 TimeNow = 0;
	static u8 lineCounter = 0x00;

	if(backStatus == ctrlParasPtr->agvStatus)
	{
		FMSDS_Ptr->MSD_Hex = hex_reload(RMS_Hex);
		RMSDS_Ptr->MSD_Hex = FMS_Hex;
	}
	else
	{
		FMSDS_Ptr->MSD_Hex = hex_reload(FMS_Hex);
		RMSDS_Ptr->MSD_Hex = RMS_Hex;
	}
	

	if(tempFMS != FMSDS_Ptr->MSD_Hex)
	{
		FMSDS_Pre = FMSDS;
		
		FMSDS_Ptr->TimeRecoder = SystemRunningTime;
		
		tempFMS = FMSDS_Ptr->MSD_Hex;

		// 1. 分析出采集到的磁传感器数据,并且将信息处理成 (传感器)
		My_MSD_Opt(FMSDS_Ptr);
		Check_MSD_Category(FMSDS_Ptr);
		
		if(MSD_LINE == FMSDS_Ptr->MSDCategory)
		{
			if(goStraightStatus == ctrlParasPtr->agvStatus)
			{
				if(lineCounter < 5)
				{
					FMSDS_Ptr->bruce_crossroads_counter++;
				}
				
			}
			else if(backStatus == ctrlParasPtr->agvStatus)
			{
				if(lineCounter > 0)
				{
					FMSDS_Ptr->bruce_crossroads_counter--;
				}
			}
		}
		else
		{
		
			Check_Magn_Location(FMSDS_Ptr, FMSDS_Pre_Ptr);
			

			Check_Magn_Direction(FMSDS_Ptr, FMSDS_Pre_Ptr);
			
			
			Magn_VelocityXt_Clau(FMSDS_Ptr, FMSDS_Pre_Ptr);
			
			
			Magn_AcceleratedXt_Clau(FMSDS_Ptr, FMSDS_Pre_Ptr);
			
			MagnInfomationUpdate = 1;
		}
		
		
	}

	
	if(tempRMS != RMSDS_Ptr->MSD_Hex)
	{
		RMSDS_Pre = RMSDS;
		
		RMSDS_Ptr->TimeRecoder = SystemRunningTime;
		
		tempRMS = RMSDS_Ptr->MSD_Hex;
		
		My_MSD_Opt(RMSDS_Ptr);
		
		
		Check_MSD_Category(RMSDS_Ptr);
		
		
		Check_Magn_Location(RMSDS_Ptr, RMSDS_Pre_Ptr);
		

		Check_Magn_Direction(RMSDS_Ptr, RMSDS_Pre_Ptr);
		
		
		Magn_VelocityXt_Clau(RMSDS_Ptr, RMSDS_Pre_Ptr);
		
		
		Magn_AcceleratedXt_Clau(RMSDS_Ptr, RMSDS_Pre_Ptr);
		
		MagnInfomationUpdate = 1;
	}
	
	
	#endif
}

#else

u8 Check_Zero_Bit_Special(Magn_Sensor_Data_Sturct_P ptr)
{
	u8 flag = 0;

	switch(ptr->MSD_Hex)
	{
		case 0x8000:
			ptr->AgvMSLocation = Agv_MS_LOut_1;
			flag = 1;
			break;

		case 0xC000:
			ptr->AgvMSLocation = Agv_MS_LOut_2;
			flag = 1;
			break;

		case 0xE000:
			ptr->AgvMSLocation = Agv_MS_LOut_3;
			flag = 1;
			break;

		case 0xF000:
			ptr->AgvMSLocation = Agv_MS_LOut_4;
			flag = 1;
			break;

		case 0xF800:
			ptr->AgvMSLocation = Agv_MS_LOut_5;
			flag = 1;
			break;

		case 0xFC00:
			ptr->AgvMSLocation = Agv_MS_LOut_6;
			flag = 1;
			break;

		case 0xFE00:
			ptr->AgvMSLocation = Agv_MS_LOut_7;
			flag = 1;
			break;

		case 0xFF00:
			ptr->AgvMSLocation = Agv_MS_LOut_8;
			flag = 1;
			break;

		case 0x00FF:
			ptr->AgvMSLocation = Agv_MS_ROut_8;
			flag = 1;
			break;

		case 0x007F:
			ptr->AgvMSLocation = Agv_MS_ROut_7;
			flag = 1;
			break;

		case 0x003F:
			ptr->AgvMSLocation = Agv_MS_ROut_6;
			flag = 1;
			break;

		case 0x001F:
			ptr->AgvMSLocation = Agv_MS_ROut_5;
			flag = 1;
			break;

		case 0x000F:
			ptr->AgvMSLocation = Agv_MS_ROut_4;
			flag = 1;
			break;

		case 0x0007:
			ptr->AgvMSLocation = Agv_MS_ROut_3;
			flag = 1;
			break;

		case 0x0003:
			ptr->AgvMSLocation = Agv_MS_ROut_2;
			flag = 1;
			break;

		case 0x0001:
			ptr->AgvMSLocation = Agv_MS_ROut_1;
			flag = 1;
			break;
	}

	return flag;
}


u8 Check_Zero_Bit_LeftOrRight(Magn_Sensor_Data_Sturct_P ptr)		
{
	u8 flag = 0;

	switch(ptr->MSD_Hex)
	{
		case 0xFFFF:
			ptr->AgvMSLocation = Agv_MS_Undefine;
			flag = 1;
			ptr->MSDCategory = MSD_OUTSIDE;
			break;
		
		case 0x7FFF:
			ptr->AgvMSLocation = Agv_MS_Right_10;
			flag = 1;
			break;

		case 0x3FFF:
			ptr->AgvMSLocation = Agv_MS_Right_9;
			flag = 1;
			break;

		case 0x1FFF:
			ptr->AgvMSLocation = Agv_MS_Right_8;
			flag = 1;
			break;

		case 0x0FFF:
			ptr->AgvMSLocation = Agv_MS_Right_7;
			flag = 1;
			break;

		case 0x07FF:
			ptr->AgvMSLocation = Agv_MS_Right_6;
			flag = 1;
			break;

		case 0x03FF:
			ptr->AgvMSLocation = Agv_MS_Right_5;
			flag = 1;
			break;

		case 0x01FF:
			ptr->AgvMSLocation = Agv_MS_Right_5;
			flag = 1;
			break;
		
		case 0xFF80:
			ptr->AgvMSLocation = Agv_MS_Left_5;
			flag = 1;
			break;

		case 0xFFC0:
			ptr->AgvMSLocation = Agv_MS_Left_5;
			flag = 1;
			break;

		case 0xFFE0:
			ptr->AgvMSLocation = Agv_MS_Left_6;
			flag = 1;
			break;

		case 0xFFF0:
			ptr->AgvMSLocation = Agv_MS_Left_7;
			flag = 1;
			break;

		case 0xFFF8:
			ptr->AgvMSLocation = Agv_MS_Left_8;
			flag = 1;
			break;

		case 0xFFFC:
			ptr->AgvMSLocation = Agv_MS_Left_9;
			flag = 1;
			break;

		case 0xFFFE:
			ptr->AgvMSLocation = Agv_MS_Left_10;
			flag = 1;
			break;
	}

	return flag;
}

u8 Check_7_Zero_Bit(Magn_Sensor_Data_Sturct_P ptr)		//CHECK
{
	u8 flag = 0;
	
	switch(ptr->MSD_Hex)
	{
		case 0x80FF:
			ptr->AgvMSLocation = Agv_MS_Right_4;
			flag = 1;
			break;

		case 0xC07F:
			ptr->AgvMSLocation = Agv_MS_Right_3;
			flag = 1;
			break;

		case 0xE03F:
			ptr->AgvMSLocation = Agv_MS_Right_2;
			flag = 1;
			break;

		case 0xF01F:
			ptr->AgvMSLocation = Agv_MS_Right_1;
			flag = 1;
			break;

		case 0xF80F:
			ptr->AgvMSLocation = Agv_MS_Left_1;
			flag = 1;
			break;

		case 0xFC07:
			ptr->AgvMSLocation = Agv_MS_Left_2;
			flag = 1;
			break;

		case 0xFE03:
			ptr->AgvMSLocation = Agv_MS_Left_3;
			flag = 1;
			break;

		case 0xFF01:
			ptr->AgvMSLocation = Agv_MS_Left_4;
			flag = 1;
			break;
		
	}

	return flag;
}

u8 Check_6_Zero_Bit(Magn_Sensor_Data_Sturct_P ptr)		//CHECK
{
	u8 flag = 0;
	
	switch(ptr->MSD_Hex)
	{
		case 0x81FF:
			ptr->AgvMSLocation = Agv_MS_Right_4;
			flag = 1;
			break;

		case 0xC0FF:
			ptr->AgvMSLocation = Agv_MS_Right_3;
			flag = 1;
			break;

		case 0xE07F:
			ptr->AgvMSLocation = Agv_MS_Right_2;
			flag = 1;
			break;

		case 0xF03F:
			ptr->AgvMSLocation = Agv_MS_Right_1;
			flag = 1;
			break;

		case 0xF81F:
			ptr->AgvMSLocation = Agv_MS_Center;
			flag = 1;
			break;

		case 0xFC0F:
			ptr->AgvMSLocation = Agv_MS_Left_1;
			flag = 1;
			break;

		case 0xFE07:
			ptr->AgvMSLocation = Agv_MS_Left_2;
			flag = 1;
			break;

		case 0xFF03:
			ptr->AgvMSLocation = Agv_MS_Left_3;
			flag = 1;
			break;

		case 0xFF81:
			ptr->AgvMSLocation = Agv_MS_Left_4;
			flag = 1;
			break;
		
	}

	return flag;
}

u8 Check_5_Zero_Bit(Magn_Sensor_Data_Sturct_P ptr)		// check
{
	u8 flag = 0;

	switch(ptr->MSD_Hex)
	{
		case 0x83FF:
			ptr->AgvMSLocation = Agv_MS_Right_5;
			flag = 1;
			break;

		case 0xC1FF:
			ptr->AgvMSLocation = Agv_MS_Right_4;
			flag = 1;
			break;

		case 0xE0FF:
			ptr->AgvMSLocation = Agv_MS_Right_3;
			flag = 1;
			break;

		case 0xF07F:
			ptr->AgvMSLocation = Agv_MS_Right_2;
			flag = 1;
			break;

		case 0xF83F:
			ptr->AgvMSLocation = Agv_MS_Right_1;
			flag = 1;
			break;

		case 0xFC1F:
			ptr->AgvMSLocation = Agv_MS_Left_1;
			flag = 1;
			break;

		case 0xFE0F:
			ptr->AgvMSLocation = Agv_MS_Left_2;
			flag = 1;
			break;

		case 0xFF07:
			ptr->AgvMSLocation = Agv_MS_Left_3;
			flag = 1;
			break;

		case 0xFF83:
			ptr->AgvMSLocation = Agv_MS_Left_4;
			flag = 1;
			break;

		case 0xFFC1:
			ptr->AgvMSLocation = Agv_MS_Left_5;
			flag = 1;
			break;
	}

	return flag;
}

u8 Check_4_Zero_Bit(Magn_Sensor_Data_Sturct_P ptr)		// CHECK
{
	u8 flag = 0;

	switch(ptr->MSD_Hex)
	{
		case 0x87FF:
			ptr->AgvMSLocation = Agv_MS_Right_5;
			flag = 1;
			break;

		case 0xC3FF:
			ptr->AgvMSLocation = Agv_MS_Right_4;
			flag = 1;
			break;

		case 0xE1FF:
			ptr->AgvMSLocation = Agv_MS_Right_3;
			flag = 1;
			break;

		case 0xF0FF:
			ptr->AgvMSLocation = Agv_MS_Right_2;
			flag = 1;
			break;

		case 0xF87F:
			ptr->AgvMSLocation = Agv_MS_Right_1;
			flag = 1;
			break;

		case 0xFC3F:
			ptr->AgvMSLocation = Agv_MS_Center;
			flag = 1;
			break;

		case 0xFE1F:
			ptr->AgvMSLocation = Agv_MS_Left_1;
			flag = 1;
			break;

		case 0xFF0F:
			ptr->AgvMSLocation = Agv_MS_Left_2;
			flag = 1;
			break;

		case 0xFF87:
			ptr->AgvMSLocation = Agv_MS_Left_3;
			flag = 1;
			break;

		case 0xFFC3:
			ptr->AgvMSLocation = Agv_MS_Left_4;
			flag = 1;
			break;

		case 0xFFE1:
			ptr->AgvMSLocation = Agv_MS_Left_5;
			flag = 1;
			break;
	}

	return flag;
}


void Magn_Sensor_Scan(void)
{
	static u16 tempFMS = 0x00, tempRMS = 0x00;
	u32 TimeNow = 0;
	static u8 lineCounter = 0x00;

	FMSDS_Pre = FMSDS;
	RMSDS_Pre = RMSDS;
	
	if(backStatus == ctrlParasPtr->agvStatus)
	{
		FMSDS_Ptr->MSD_Hex = hex_reload(RMS_Hex);
		RMSDS_Ptr->MSD_Hex = FMS_Hex;
	}
	else
	{
		FMSDS_Ptr->MSD_Hex = hex_reload(FMS_Hex);
		RMSDS_Ptr->MSD_Hex = RMS_Hex;
	}
	

	if(1 == Check_Zero_Bit_LeftOrRight(FMSDS_Ptr))
	{
		
	}
	else if(1 == Check_6_Zero_Bit(FMSDS_Ptr))
	{
		FMSDS_Ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_5_Zero_Bit(FMSDS_Ptr))
	{
		FMSDS_Ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_4_Zero_Bit(FMSDS_Ptr))
	{
		FMSDS_Ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_Zero_Bit_LeftOrRight(FMSDS_Ptr))
	{
		
	}
	else if(1 == Check_7_Zero_Bit(FMSDS_Ptr))
	{
		FMSDS_Ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_Zero_Bit_Special(FMSDS_Ptr))
	{
		
	}
	else
	{
		//printf("FMSS_Err: ");
		MSDF_Opts_Ptr->MSD_Show_Bin(FMSDS_Ptr->MSD_Hex);
		//printf("LMD = %d,\tRMD = %d,\t", ctrlParasPtr->leftMotorSettedSpeed, ctrlParasPtr->rightMotorSettedSpeed);
		//printf("\r\n");
	}


	if(1 == Check_Zero_Bit_LeftOrRight(RMSDS_Ptr))
	{
		
	}
	else if(1 == Check_6_Zero_Bit(RMSDS_Ptr))
	{
		RMSDS_Ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_5_Zero_Bit(RMSDS_Ptr))
	{
		RMSDS_Ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_4_Zero_Bit(RMSDS_Ptr))
	{
		RMSDS_Ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_Zero_Bit_LeftOrRight(RMSDS_Ptr))
	{
		
	}
	else if(1 == Check_7_Zero_Bit(RMSDS_Ptr))
	{
		RMSDS_Ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_Zero_Bit_Special(RMSDS_Ptr))
	{
		
	}
	else
	{
		printf("RMSS_Err: ");
		MSDF_Opts_Ptr->MSD_Show_Bin(RMSDS_Ptr->MSD_Hex);
		//printf("LMD = %d,\tRMD = %d,\t", ctrlParasPtr->leftMotorSettedSpeed, ctrlParasPtr->rightMotorSettedSpeed);
		printf("\r\n");
	}
	
}


#endif

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

	FMSDS_Ptr->agvDirection = AgvNone;

	FMSDS_Ptr->AgvMSLocation = AgvInits;

	FMSDS_Ptr->AgvMSLocation_s = 0;

	FMSDS_Ptr->MSDCategory = MSD_NORMAL;
	
	FMSDS_Ptr->bruce_crossroads_counter = 0;
	
	FMSDS_Pre_Ptr->MSD_Hex = 0xF81F;
	
	FMSDS_Pre_Ptr->MSD_Dec = 0x00;

	FMSDS_Pre_Ptr->VelocityX = 0x00;

	FMSDS_Pre_Ptr->AcceleratedX = 0x00;

	FMSDS_Pre_Ptr->LeftRemain = 5;

	FMSDS_Pre_Ptr->RightRemain = 5;

	FMSDS_Pre_Ptr->BitNum = 6;

	FMSDS_Pre_Ptr->agvDirection = AgvNone;

	FMSDS_Pre_Ptr->AgvMSLocation = Agv_MS_Center;

	FMSDS_Pre_Ptr->AgvMSLocation_s = 0;

	FMSDS_Pre_Ptr->MSDCategory = MSD_NORMAL;

	FMSDS_Ptr->zeropointfive = 0;
	FMSDS_Ptr->zflag = 0;
	
	*RMSDS_Ptr = *FMSDS_Ptr;
	*RMSDS_Pre_Ptr = *FMSDS_Pre_Ptr;
	
	MSDF_Opts_Ptr->MY_MSD_Operator = My_MSD_Opt;
	MSDF_Opts_Ptr->MS_Scan = Magn_Sensor_Scan;
	MSDF_Opts_Ptr->MSD_Show_Bin = MSD_Show_Bin;
	MSDF_Opts_Ptr->Show_Opt_MSD = Show_My_MSD_Opt;
	MSDF_Opts_Ptr->magn_show = magn_show;
	MSDF_Opts_Ptr->Show_Infomation = Show_Infomation;
}



