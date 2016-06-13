#include "magn_d_algo.h"

Magn_Sensor_Data_Sturct FMSDS;
Magn_Sensor_Data_Sturct RMSDS;
Magn_Sensor_Data_Sturct FMSDS_Pre;
Magn_Sensor_Data_Sturct RMSDS_Pre;


Magn_Sensor_Data_Sturct_P FMSDS_Ptr = &FMSDS;
Magn_Sensor_Data_Sturct_P RMSDS_Ptr = &RMSDS;
Magn_Sensor_Data_Sturct_P FMSDS_Pre_Ptr = &FMSDS_Pre;
Magn_Sensor_Data_Sturct_P RMSDS_Pre_Ptr = &RMSDS_Pre;

Pattern_Num_Para AGV_Pattern;
Pattern_Num_Para_P AGV_Pat_Ptr = &AGV_Pattern;
Pattern_Num_Para AGV_Pattern_Pre;
Pattern_Num_Para_P AGV_Pat_Pre_Ptr = &AGV_Pattern_Pre;


u8 Check_Zero_Bit_Special(Magn_Sensor_Data_Sturct_P ptr)
{
	u8 flag = 0;

	switch(ptr->MSD_Hex)
	{
		case 0x0000:
			ptr->AgvMSLocation = Agv_MS_Overline;
			ptr->LeftRemain = 0;
			ptr->BitNum = 16;
			ptr->RightRemain = 0;
			flag = 1;
			break;
		
		case 0x8000:
			ptr->AgvMSLocation = Agv_MS_LOut_1;
			ptr->LeftRemain = 1;
			ptr->BitNum = 15;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xC000:
			ptr->AgvMSLocation = Agv_MS_LOut_2;
			ptr->LeftRemain = 2;
			ptr->BitNum = 14;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xE000:
			ptr->AgvMSLocation = Agv_MS_LOut_3;
			ptr->LeftRemain = 3;
			ptr->BitNum = 13;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xF000:
			ptr->AgvMSLocation = Agv_MS_LOut_4;
			ptr->LeftRemain = 4;
			ptr->BitNum = 12;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xF800:
			ptr->AgvMSLocation = Agv_MS_LOut_5;
			ptr->LeftRemain = 5;
			ptr->BitNum = 11;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFC00:
			ptr->AgvMSLocation = Agv_MS_LOut_6;
			ptr->LeftRemain = 6;
			ptr->BitNum = 10;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFE00:
			ptr->AgvMSLocation = Agv_MS_LOut_7;
			ptr->LeftRemain = 7;
			ptr->BitNum = 9;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFF00:
			ptr->AgvMSLocation = Agv_MS_LOut_8;
			ptr->LeftRemain = 8;
			ptr->BitNum = 8;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0x00FF:
			ptr->AgvMSLocation = Agv_MS_ROut_8;
			ptr->LeftRemain = 0;
			ptr->BitNum = 8;
			ptr->RightRemain = 8;
			flag = 1;
			break;

		case 0x007F:
			ptr->AgvMSLocation = Agv_MS_ROut_7;
			ptr->LeftRemain = 0;
			ptr->BitNum = 9;
			ptr->RightRemain = 7;
			flag = 1;
			break;

		case 0x003F:
			ptr->AgvMSLocation = Agv_MS_ROut_6;
			ptr->LeftRemain = 0;
			ptr->BitNum = 10;
			ptr->RightRemain = 6;
			flag = 1;
			break;

		case 0x001F:
			ptr->AgvMSLocation = Agv_MS_ROut_5;
			ptr->LeftRemain = 0;
			ptr->BitNum = 11;
			ptr->RightRemain = 5;
			flag = 1;
			break;

		case 0x000F:
			ptr->AgvMSLocation = Agv_MS_ROut_4;
			ptr->LeftRemain = 0;
			ptr->BitNum = 12;
			ptr->RightRemain = 4;
			flag = 1;
			break;

		case 0x0007:
			ptr->AgvMSLocation = Agv_MS_ROut_3;
			ptr->LeftRemain = 0;
			ptr->BitNum = 13;
			ptr->RightRemain = 3;
			flag = 1;
			break;

		case 0x0003:
			ptr->AgvMSLocation = Agv_MS_ROut_2;
			ptr->LeftRemain = 0;
			ptr->BitNum = 14;
			ptr->RightRemain = 2;
			flag = 1;
			break;

		case 0x0001:
			ptr->AgvMSLocation = Agv_MS_ROut_1;
			ptr->LeftRemain = 0;
			ptr->BitNum = 15;
			ptr->RightRemain = 1;
			flag = 1;
			break;

		default:
			ptr->AgvMSLocation = Agv_MS_Undefine;
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
			ptr->LeftRemain = 16;
			ptr->BitNum = 0;
			ptr->RightRemain = 16;
			flag = 1;
			ptr->MSDCategory = MSD_OUTSIDE;
			break;
		
		case 0x7FFF:
			ptr->AgvMSLocation = Agv_MS_Right_10;
			ptr->LeftRemain = 0;
			ptr->BitNum = 1;
			ptr->RightRemain = 15;
			flag = 1;
			break;

		case 0x3FFF:
			ptr->AgvMSLocation = Agv_MS_Right_9;
			ptr->LeftRemain = 0;
			ptr->BitNum = 2;
			ptr->RightRemain = 14;
			flag = 1;
			break;

		case 0x1FFF:
			ptr->AgvMSLocation = Agv_MS_Right_8;
			ptr->LeftRemain = 0;
			ptr->BitNum = 3;
			ptr->RightRemain = 13;
			flag = 1;
			break;

		case 0x0FFF:
			ptr->AgvMSLocation = Agv_MS_Right_7;
			ptr->LeftRemain = 0;
			ptr->BitNum = 4;
			ptr->RightRemain = 12;
			flag = 1;
			break;

		case 0x07FF:
			ptr->AgvMSLocation = Agv_MS_Right_6;
			ptr->LeftRemain = 0;
			ptr->BitNum = 5;
			ptr->RightRemain = 11;
			flag = 1;
			break;

		case 0x03FF:
			ptr->AgvMSLocation = Agv_MS_Right_5;
			ptr->LeftRemain = 0;
			ptr->BitNum = 6;
			ptr->RightRemain = 10;
			flag = 1;
			break;

		case 0x01FF:
			ptr->AgvMSLocation = Agv_MS_Right_5;
			ptr->LeftRemain = 0;
			ptr->BitNum = 7;
			ptr->RightRemain = 9;
			flag = 1;
			break;
		
		case 0xFF80:
			ptr->AgvMSLocation = Agv_MS_Left_5;
			ptr->LeftRemain = 9;
			ptr->BitNum = 7;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFFC0:
			ptr->AgvMSLocation = Agv_MS_Left_5;
			ptr->LeftRemain = 10;
			ptr->BitNum = 6;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFFE0:
			ptr->AgvMSLocation = Agv_MS_Left_6;
			ptr->LeftRemain = 11;
			ptr->BitNum = 5;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFFF0:
			ptr->AgvMSLocation = Agv_MS_Left_7;
			ptr->LeftRemain = 12;
			ptr->BitNum = 4;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFFF8:
			ptr->AgvMSLocation = Agv_MS_Left_8;
			ptr->LeftRemain = 13;
			ptr->BitNum = 3;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFFFC:
			ptr->AgvMSLocation = Agv_MS_Left_9;
			ptr->LeftRemain = 14;
			ptr->BitNum = 2;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		case 0xFFFE:
			ptr->AgvMSLocation = Agv_MS_Left_10;
			ptr->LeftRemain = 15;
			ptr->BitNum = 1;
			ptr->RightRemain = 0;
			flag = 1;
			break;

		default:
			ptr->AgvMSLocation = Agv_MS_Undefine;
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
			ptr->LeftRemain = 1;
			ptr->BitNum = 7;
			ptr->RightRemain = 8;
			flag = 1;
			break;

		case 0xC07F:
			ptr->AgvMSLocation = Agv_MS_Right_3;
			ptr->LeftRemain = 2;
			ptr->BitNum = 7;
			ptr->RightRemain = 7;
			flag = 1;
			break;

		case 0xE03F:
			ptr->AgvMSLocation = Agv_MS_Right_2;
			ptr->LeftRemain = 3;
			ptr->BitNum = 7;
			ptr->RightRemain = 6;
			flag = 1;
			break;

		case 0xF01F:
			ptr->AgvMSLocation = Agv_MS_Right_1;
			ptr->LeftRemain = 4;
			ptr->BitNum = 7;
			ptr->RightRemain = 5;
			flag = 1;
			break;

		case 0xF80F:
			ptr->AgvMSLocation = Agv_MS_Left_1;
			ptr->LeftRemain = 5;
			ptr->BitNum = 7;
			ptr->RightRemain = 4;
			flag = 1;
			break;

		case 0xFC07:
			ptr->AgvMSLocation = Agv_MS_Left_2;
			ptr->LeftRemain = 6;
			ptr->BitNum = 7;
			ptr->RightRemain = 3;
			flag = 1;
			break;

		case 0xFE03:
			ptr->AgvMSLocation = Agv_MS_Left_3;
			ptr->LeftRemain = 7;
			ptr->BitNum = 7;
			ptr->RightRemain = 2;
			flag = 1;
			break;

		case 0xFF01:
			ptr->AgvMSLocation = Agv_MS_Left_4;
			ptr->LeftRemain = 8;
			ptr->BitNum = 7;
			ptr->RightRemain = 1;
			flag = 1;
			break;

		default:
			ptr->AgvMSLocation = Agv_MS_Undefine;
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
			ptr->LeftRemain = 1;
			ptr->BitNum = 6;
			ptr->RightRemain = 9;
			flag = 1;
			break;

		case 0xC0FF:
			ptr->AgvMSLocation = Agv_MS_Right_3;
			ptr->LeftRemain = 2;
			ptr->BitNum = 6;
			ptr->RightRemain = 8;
			flag = 1;
			break;

		case 0xE07F:
			ptr->AgvMSLocation = Agv_MS_Right_2;
			ptr->LeftRemain = 3;
			ptr->BitNum = 6;
			ptr->RightRemain = 7;
			flag = 1;
			break;

		case 0xF03F:
			ptr->AgvMSLocation = Agv_MS_Right_1;
			ptr->LeftRemain = 4;
			ptr->BitNum = 6;
			ptr->RightRemain = 6;
			flag = 1;
			break;

		case 0xF81F:
			ptr->AgvMSLocation = Agv_MS_Center;
			ptr->LeftRemain = 5;
			ptr->BitNum = 6;
			ptr->RightRemain = 5;
			flag = 1;
			break;

		case 0xFC0F:
			ptr->AgvMSLocation = Agv_MS_Left_1;
			ptr->LeftRemain = 6;
			ptr->BitNum = 6;
			ptr->RightRemain = 4;
			flag = 1;
			break;

		case 0xFE07:
			ptr->AgvMSLocation = Agv_MS_Left_2;
			ptr->LeftRemain = 7;
			ptr->BitNum = 6;
			ptr->RightRemain = 3;
			flag = 1;
			break;

		case 0xFF03:
			ptr->AgvMSLocation = Agv_MS_Left_3;
			ptr->LeftRemain = 8;
			ptr->BitNum = 6;
			ptr->RightRemain = 2;
			flag = 1;
			break;

		case 0xFF81:
			ptr->AgvMSLocation = Agv_MS_Left_4;
			ptr->LeftRemain = 9;
			ptr->BitNum = 6;
			ptr->RightRemain = 1;
			flag = 1;
			break;

		default:
			ptr->AgvMSLocation = Agv_MS_Undefine;
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
			ptr->LeftRemain = 1;
			ptr->BitNum = 5;
			ptr->RightRemain = 10;
			flag = 1;
			break;

		case 0xC1FF:
			ptr->AgvMSLocation = Agv_MS_Right_4;
			ptr->LeftRemain = 2;
			ptr->BitNum = 5;
			ptr->RightRemain = 9;
			flag = 1;
			break;

		case 0xE0FF:
			ptr->AgvMSLocation = Agv_MS_Right_3;
			ptr->LeftRemain = 3;
			ptr->BitNum = 5;
			ptr->RightRemain = 8;
			flag = 1;
			break;

		case 0xF07F:
			ptr->AgvMSLocation = Agv_MS_Right_2;
			ptr->LeftRemain =4 ;
			ptr->BitNum = 5;
			ptr->RightRemain = 7;
			flag = 1;
			break;

		case 0xF83F:
			ptr->AgvMSLocation = Agv_MS_Right_1;
			ptr->LeftRemain = 5;
			ptr->BitNum = 5;
			ptr->RightRemain = 6;
			flag = 1;
			break;

		case 0xFC1F:
			ptr->AgvMSLocation = Agv_MS_Left_1;
			ptr->LeftRemain = 6;
			ptr->BitNum = 5;
			ptr->RightRemain = 5;
			flag = 1;
			break;

		case 0xFE0F:
			ptr->AgvMSLocation = Agv_MS_Left_2;
			ptr->LeftRemain = 7;
			ptr->BitNum = 5;
			ptr->RightRemain = 4;
			flag = 1;
			break;

		case 0xFF07:
			ptr->AgvMSLocation = Agv_MS_Left_3;
			ptr->LeftRemain = 8;
			ptr->BitNum = 5;
			ptr->RightRemain = 3;
			flag = 1;
			break;

		case 0xFF83:
			ptr->AgvMSLocation = Agv_MS_Left_4;
			ptr->LeftRemain = 9;
			ptr->BitNum = 5;
			ptr->RightRemain = 2;
			flag = 1;
			break;

		case 0xFFC1:
			ptr->AgvMSLocation = Agv_MS_Left_5;
			ptr->LeftRemain = 10;
			ptr->BitNum = 5;
			ptr->RightRemain = 1;
			flag = 1;
			break;

		default:
			ptr->AgvMSLocation = Agv_MS_Undefine;
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
			ptr->LeftRemain = 1;
			ptr->BitNum = 4;
			ptr->RightRemain = 11;
			flag = 1;
			break;

		case 0xC3FF:
			ptr->AgvMSLocation = Agv_MS_Right_4;
			ptr->LeftRemain = 2;
			ptr->BitNum = 4;
			ptr->RightRemain = 10;
			flag = 1;
			break;

		case 0xE1FF:
			ptr->AgvMSLocation = Agv_MS_Right_3;
			ptr->LeftRemain = 3;
			ptr->BitNum = 4;
			ptr->RightRemain = 9;
			flag = 1;
			break;

		case 0xF0FF:
			ptr->AgvMSLocation = Agv_MS_Right_2;
			ptr->LeftRemain = 4;
			ptr->BitNum = 4;
			ptr->RightRemain = 8;
			flag = 1;
			break;

		case 0xF87F:
			ptr->AgvMSLocation = Agv_MS_Right_1;
			ptr->LeftRemain = 5;
			ptr->BitNum = 4;
			ptr->RightRemain = 7;
			flag = 1;
			break;

		case 0xFC3F:
			ptr->AgvMSLocation = Agv_MS_Center;
			ptr->LeftRemain = 6;
			ptr->BitNum = 4;
			ptr->RightRemain = 6;
			flag = 1;
			break;

		case 0xFE1F:
			ptr->AgvMSLocation = Agv_MS_Left_1;
			ptr->LeftRemain = 7;
			ptr->BitNum = 4;
			ptr->RightRemain = 5;
			flag = 1;
			break;

		case 0xFF0F:
			ptr->AgvMSLocation = Agv_MS_Left_2;
			ptr->LeftRemain = 8;
			ptr->BitNum = 4;
			ptr->RightRemain = 4;
			flag = 1;
			break;

		case 0xFF87:
			ptr->AgvMSLocation = Agv_MS_Left_3;
			ptr->LeftRemain = 9;
			ptr->BitNum = 4;
			ptr->RightRemain = 3;
			flag = 1;
			break;

		case 0xFFC3:
			ptr->AgvMSLocation = Agv_MS_Left_4;
			ptr->LeftRemain = 10;
			ptr->BitNum = 4;
			ptr->RightRemain = 2;
			flag = 1;
			break;

		case 0xFFE1:
			ptr->AgvMSLocation = Agv_MS_Left_5;
			ptr->LeftRemain = 11;
			ptr->BitNum = 4;
			ptr->RightRemain = 1;
			flag = 1;
			break;

		default:
			ptr->AgvMSLocation = Agv_MS_Undefine;
			break;
	}

	return flag;
}


void MSD_Show_Bin(u32 showNum)
{
	u8 cir = 0;
	//printf("%x, ", showNum);
	for(cir = 0; cir < 16; cir++)
	{
		printf("%lu", ((showNum >> (15 - cir)) & 0x01));
		
		if(((cir + 1) % 4) == 0)
		{
			printf(" ");
		}
	}
	
}

void Show_Resualt_Analy(Magn_Sensor_Data_Sturct_P ptr)
{
	//printf("MSD_Hex = %04d,\t", ptr->MSD_Hex);
	//MSD_Show_Bin(ptr->MSD_Hex);
	//printf(",\t");
	//printf("MSL = (%d) MS_", ptr->AgvMSLocation);
	if((ptr->AgvMSLocation > Agv_MS_Left_End) && (ptr->AgvMSLocation < Agv_MS_Center))
	{
		printf("L_");
		printf("%d,\t", Agv_MS_Center - ptr->AgvMSLocation);
	}
	else if((ptr->AgvMSLocation > Agv_MS_Center) && (ptr->AgvMSLocation < Agv_MS_Right_End))
	{
		printf("R_");
		printf("%d,\t", ptr->AgvMSLocation - Agv_MS_Center);
	}
	else if((ptr->AgvMSLocation >= Agv_MS_LOut_1) && (ptr->AgvMSLocation <= Agv_MS_LOut_8))
	{
		printf("LO_");
		printf("%d,\t", ptr->AgvMSLocation - Agv_MS_Overline);
	}
	else if((ptr->AgvMSLocation >= Agv_MS_ROut_1) && (ptr->AgvMSLocation <= Agv_MS_ROut_8))
	{
		printf("RO_");
		printf("%d,\t", ptr->AgvMSLocation - Agv_MS_LOut_8);
	}
	else if(ptr->AgvMSLocation == Agv_MS_Undefine)
	{
		printf("Undef,\t");
	}
	else if(ptr->AgvMSLocation == Agv_MS_Overline)
	{
		printf("Overline,\t");
	}
	else if(ptr->AgvMSLocation == Agv_MS_Right_Outside)
	{
		printf("Right_Outside,\t");
	}
	else if(ptr->AgvMSLocation == Agv_MS_Left_Outside)
	{
		printf("Left_Outside,\t");
	}
	else if(ptr->AgvMSLocation == Agv_MS_Center)
	{
		printf("MS_Center,\t");
	}
	else if(ptr->AgvMSLocation == AgvInits)
	{
		printf("Inits,\t");
	}

	//printf("LRe = %d,\t", ptr->LeftRemain);
	//printf("Zbits = %d,\t", ptr->BitNum);
	//printf("RRe = %d,\t", ptr->RightRemain);
	
}


void Midpoint_Pattern_Num(Magn_Sensor_Data_Sturct_P FMS, Magn_Sensor_Data_Sturct_P RMS, Pattern_Num_Para_P PTR)
{
	
	PTR->Midpoint = (FMS->AgvMSLocation - Agv_MS_Center) + (RMS->AgvMSLocation - Agv_MS_Center);
	
}

void Angle_Pattern_Num(Magn_Sensor_Data_Sturct_P FMS, Magn_Sensor_Data_Sturct_P RMS, Pattern_Num_Para_P PTR)
{
	
	PTR->Angle = (FMS->AgvMSLocation - Agv_MS_Center) - (RMS->AgvMSLocation - Agv_MS_Center);
	
}

void Get_Pattern_Num(Magn_Sensor_Data_Sturct_P FMS, Magn_Sensor_Data_Sturct_P RMS, Pattern_Num_Para_P PTR)
{
	Midpoint_Pattern_Num(FMS, RMS, PTR);
	Angle_Pattern_Num(FMS, RMS, PTR);
}

void Midpoint_Pattern_Scale(Magn_Sensor_Data_Sturct_P FMS, Magn_Sensor_Data_Sturct_P RMS, Pattern_Scale_Para_P PTR)
{
	Pattern_Num_Para ptr;
	
	Midpoint_Pattern_Num(FMS, RMS, &ptr);

	if(ptr.Midpoint < 0)
	{
		switch(ptr.Midpoint)
		{
			case -20:
				PTR->Midpoint = Midpoint_Level_L_20;
				break;

			case -19:
				PTR->Midpoint = Midpoint_Level_L_19;
				break;

			case -18:
				PTR->Midpoint = Midpoint_Level_L_18;
				break;

			case -17:
				PTR->Midpoint = Midpoint_Level_L_17;
				break;

			case -16:
				PTR->Midpoint = Midpoint_Level_L_16;
				break;

			case -15:
				PTR->Midpoint = Midpoint_Level_L_15;
				break;

			case -14:
				PTR->Midpoint = Midpoint_Level_L_14;
				break;

			case -13:
				PTR->Midpoint = Midpoint_Level_L_13;
				break;

			case -12:
				PTR->Midpoint = Midpoint_Level_L_12;
				break;

			case -11:
				PTR->Midpoint = Midpoint_Level_L_11;
				break;

			case -10:
				PTR->Midpoint = Midpoint_Level_L_10;
				break;

			case -9:
				PTR->Midpoint = Midpoint_Level_L_9;
				break;

			case -8:
				PTR->Midpoint = Midpoint_Level_L_8;
				break;

			case -7:
				PTR->Midpoint = Midpoint_Level_L_7;
				break;

			case -6:
				PTR->Midpoint = Midpoint_Level_L_6;
				break;

			case -5:
				PTR->Midpoint = Midpoint_Level_L_5;
				break;

			case -4:
				PTR->Midpoint = Midpoint_Level_L_4;
				break;

			case -3:
				PTR->Midpoint = Midpoint_Level_L_3;
				break;

			case -2:
				PTR->Midpoint = Midpoint_Level_L_2;
				break;

			case -1:
				PTR->Midpoint = Midpoint_Level_L_1;
				break;
			
			default:
				PTR->Midpoint = Midpoint_Level_Error;
				break;
		}
	}
	else if(ptr.Midpoint > 0)
	{
		switch(ptr.Midpoint)
		{
			case 1:
				PTR->Midpoint = Midpoint_Level_R_1;
				break;

			case 2:
				PTR->Midpoint = Midpoint_Level_R_2;
				break;

			case 3:
				PTR->Midpoint = Midpoint_Level_R_3;
				break;

			case 4:
				PTR->Midpoint = Midpoint_Level_R_4;
				break;

			case 5:
				PTR->Midpoint = Midpoint_Level_R_5;
				break;

			case 6:
				PTR->Midpoint = Midpoint_Level_R_6;
				break;

			case 7:
				PTR->Midpoint = Midpoint_Level_R_7;
				break;

			case 8:
				PTR->Midpoint = Midpoint_Level_R_8;
				break;

			case 9:
				PTR->Midpoint = Midpoint_Level_R_9;
				break;

			case 10:
				PTR->Midpoint = Midpoint_Level_R_10;
				break;

			case 11:
				PTR->Midpoint = Midpoint_Level_R_11;
				break;

			case 12:
				PTR->Midpoint = Midpoint_Level_R_12;
				break;

			case 13:
				PTR->Midpoint = Midpoint_Level_R_13;
				break;

			case 14:
				PTR->Midpoint = Midpoint_Level_R_14;
				break;

			case 15:
				PTR->Midpoint = Midpoint_Level_R_15;
				break;

			case 16:
				PTR->Midpoint = Midpoint_Level_R_16;
				break;

			case 17:
				PTR->Midpoint = Midpoint_Level_R_17;
				break;

			case 18:
				PTR->Midpoint = Midpoint_Level_R_18;
				break;

			case 19:
				PTR->Midpoint = Midpoint_Level_R_19;
				break;

			case 20:
				PTR->Midpoint = Midpoint_Level_R_20;
				break;

			default:
				PTR->Midpoint = Midpoint_Level_Error;
				break;
		}
	}
	else if(0 == ptr.Midpoint)
	{
		PTR->Midpoint = Midpoint_Level_0;
	}
	else
	{
		PTR->Midpoint = Midpoint_Level_Unknow;
	}
	
	
}

void Angle_Pattern_Scale(Magn_Sensor_Data_Sturct_P FMS, Magn_Sensor_Data_Sturct_P RMS, Pattern_Scale_Para_P PTR)
{
	Pattern_Num_Para ptr;
	
	Angle_Pattern_Num(FMS, RMS, &ptr);

	if(ptr.Angle < 0)
	{
		switch(ptr.Angle)
		{
			case -20:
				PTR->Angle = Angle_Level_L_20;
				break;

			case -19:
				PTR->Angle = Angle_Level_L_19;
				break;

			case -18:
				PTR->Angle = Angle_Level_L_18;
				break;

			case -17:
				PTR->Angle = Angle_Level_L_17;
				break;

			case -16:
				PTR->Angle = Angle_Level_L_16;
				break;

			case -15:
				PTR->Angle = Angle_Level_L_15;
				break;

			case -14:
				PTR->Angle = Angle_Level_L_14;
				break;

			case -13:
				PTR->Angle = Angle_Level_L_13;
				break;

			case -12:
				PTR->Angle = Angle_Level_L_12;
				break;

			case -11:
				PTR->Angle = Angle_Level_L_11;
				break;

			case -10:
				PTR->Angle = Angle_Level_L_10;
				break;

			case -9:
				PTR->Angle = Angle_Level_L_9;
				break;

			case -8:
				PTR->Angle = Angle_Level_L_8;
				break;

			case -7:
				PTR->Angle = Angle_Level_L_7;
				break;

			case -6:
				PTR->Angle = Angle_Level_L_6;
				break;

			case -5:
				PTR->Angle = Angle_Level_L_5;
				break;

			case -4:
				PTR->Angle = Angle_Level_L_4;
				break;

			case -3:
				PTR->Angle = Angle_Level_L_3;
				break;

			case -2:
				PTR->Angle = Angle_Level_L_2;
				break;

			case -1:
				PTR->Angle = Angle_Level_L_1;
				break;
			
			default:
				PTR->Angle = Angle_Level_Error;
				break;
		}
	}
	else if(ptr.Angle > 0)
	{
		switch(ptr.Angle)
		{
			case 1:
				PTR->Angle = Angle_Level_R_1;
				break;

			case 2:
				PTR->Angle = Angle_Level_R_2;
				break;

			case 3:
				PTR->Angle = Angle_Level_R_3;
				break;

			case 4:
				PTR->Angle = Angle_Level_R_4;
				break;

			case 5:
				PTR->Angle = Angle_Level_R_5;
				break;

			case 6:
				PTR->Angle = Angle_Level_R_6;
				break;

			case 7:
				PTR->Angle = Angle_Level_R_7;
				break;

			case 8:
				PTR->Angle = Angle_Level_R_8;
				break;

			case 9:
				PTR->Angle = Angle_Level_R_9;
				break;

			case 10:
				PTR->Angle = Angle_Level_R_10;
				break;

			case 11:
				PTR->Angle = Angle_Level_R_11;
				break;

			case 12:
				PTR->Angle = Angle_Level_R_12;
				break;

			case 13:
				PTR->Angle = Angle_Level_R_13;
				break;

			case 14:
				PTR->Angle = Angle_Level_R_14;
				break;

			case 15:
				PTR->Angle = Angle_Level_R_15;
				break;

			case 16:
				PTR->Angle = Angle_Level_R_16;
				break;

			case 17:
				PTR->Angle = Angle_Level_R_17;
				break;

			case 18:
				PTR->Angle = Angle_Level_R_18;
				break;

			case 19:
				PTR->Angle = Angle_Level_R_19;
				break;

			case 20:
				PTR->Angle = Angle_Level_R_20;
				break;

			default:
				PTR->Angle = Angle_Level_Error;
				break;
		}
	}
	else if(0 == ptr.Angle)
	{
		PTR->Angle = Angle_Level_0;
	}
	else
	{
		PTR->Angle = Angle_Level_Unknow;
	}
	
}

void Get_Pattern_Scale(Magn_Sensor_Data_Sturct_P FMS, Magn_Sensor_Data_Sturct_P RMS, Pattern_Scale_Para_P PTR)
{
	Midpoint_Pattern_Scale(FMS, RMS, PTR);
	Angle_Pattern_Scale(FMS, RMS, PTR);
}


void MSD_Analy(Magn_Sensor_Data_Sturct_P ptr)
{
	if(1 == Check_Zero_Bit_LeftOrRight(ptr))
	{
		
	}
	else if(1 == Check_6_Zero_Bit(ptr))
	{
		ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_5_Zero_Bit(ptr))
	{
		ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_4_Zero_Bit(ptr))
	{
		ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_Zero_Bit_LeftOrRight(ptr))
	{
		
	}
	else if(1 == Check_7_Zero_Bit(ptr))
	{
		ptr->MSDCategory = MSD_NORMAL;
	}
	else if(1 == Check_Zero_Bit_Special(ptr))
	{
		
	}
	else
	{
		//MSDF_Opts_Ptr->MSD_Show_Bin(FMSDS_Ptr->MSD_Hex);
		//printf("LMD = %d,\tRMD = %d,\t", ctrlParasPtr->leftMotorSettedSpeed, ctrlParasPtr->rightMotorSettedSpeed);
		//printf("\r\n");
	}
	

}



void MSDS_Init(void)
{
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


	AGV_Pat_Ptr->Angle = 0;
	AGV_Pat_Ptr->Midpoint = 0;
}




