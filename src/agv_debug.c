#include "agv_debug.h"

AGV_Debug_Str Debug_Para;
AGV_Debug_Str_P Debug_Para_Ptr = &Debug_Para;


void show_step(WalkStep step)
{
	if(step_gS == step)
	{
		printf("step is: step_gS.\r\n");
	}
	else if(step_gVeer == step)
	{
		printf("step is: step_gVeer.\r\n");
	}
	else if(step_entry == step)
	{
		printf("step is: step_entry.\r\n");
	}
	else if(step_catch == step)
	{
		printf("step is: step_catch.\r\n");
	}
	else if(step_exit == step)
	{
		printf("step is: step_exit.\r\n");
	}
	else if(step_weigh == step)
	{
		printf("step is: step_weigh.\r\n");
	}
	else if(step_bVeer == step)
	{
		printf("step is: step_bVeer.\r\n");
	}
	else if(step_gB == step)
	{
		printf("step is: step_gB.\r\n");
	}
	else if(step_wFTans == step)
	{
		printf("step is: step_wFTans.\r\n");
	}
	else if(step_origin == step)
	{
		printf("step is: step_origin.\r\n");
	}
	else if(step_stop == step)
	{
		printf("step is: step_stop.\r\n");
	}
}

void show_walkingmode(AgvWalkMode mode)
{
	if(AutomaticMode == mode)
	{
		printf("agvWalkingMode = AutomaticMode\r\n");
	}
	else if(ManualMode == mode)
	{
		printf("agvWalkingMode = ManualMode\r\n");
	}
	else if(TestMode == mode)
	{
		printf("agvWalkingMode = TestMode\r\n");
	}
		
}

void Show_AgvStatus(AgvStatus status)
{
	if(stopStatus == status)
	{
		printf("agvStatus = stopStatus\r\n");
	}
	else if(goStraightStatus == status)
	{
		printf("agvStatus = goStraightStatus\r\n");
	}
	else if(backStatus == status)
	{
		printf("agvStatus = backStatus\r\n");
	}
	else if(cirLeft == status)
	{
		printf("agvStatus = cirLeft\r\n");
	}
	else if(cirRight == status)
	{
		printf("agvStatus = cirRight\r\n");
	}
	else if(gSslow == status)
	{
		printf("agvStatus = gSslow\r\n");
	}
	else if(bSslow == status)
	{
		printf("agvStatus = bSslow\r\n");
	}
	
		
}

void AGV_Debug_Func(void)
{
	if(Debug_Para_Ptr->agvWalkingMode != ctrlParasPtr->agvWalkingMode)
	{
		Debug_Para_Ptr->agvWalkingMode = ctrlParasPtr->agvWalkingMode;
		show_walkingmode(ctrlParasPtr->agvWalkingMode);
	}
	
	if(Debug_Para_Ptr->runningInfo.Req_Station != Zigbee_Ptr->runningInfo.Req_Station)
	{
		Debug_Para_Ptr->runningInfo.Req_Station = Zigbee_Ptr->runningInfo.Req_Station;
		Debug_Para_Ptr->runningInfo.Req_Type = Zigbee_Ptr->runningInfo.Req_Type;
		
		printf("get zigbee request station: %d, ", Zigbee_Ptr->runningInfo.Req_Station);
		if(TypeUnknow == Zigbee_Ptr->runningInfo.Req_Type)
		{
			printf("type is: Unknow.\r\n");
		}
		else if(TypeUnknow == Zigbee_Ptr->runningInfo.Req_Type)
		{
			printf("type is: Manual Request.\r\n");
		}
		else if(TypeUnknow == Zigbee_Ptr->runningInfo.Req_Type)
		{
			printf("type is: Auto Request.\r\n");
		}
		
	}
	
	if(Debug_Para_Ptr->goalRFIDnode != ctrlParasPtr->goalRFIDnode)
	{
		Debug_Para_Ptr->goalRFIDnode = ctrlParasPtr->goalRFIDnode;
		printf("goalRFIDnode = %d\r\n", ctrlParasPtr->goalRFIDnode);

	}

	if(Debug_Para_Ptr->walkingstep != ctrlParasPtr->walkingstep)
	{	
		Debug_Para_Ptr->walkingstep = ctrlParasPtr->walkingstep;

		show_step(ctrlParasPtr->walkingstep);
		
	}

	if(Debug_Para_Ptr->originFlag != ctrlParasPtr->originFlag)
	{
		Debug_Para_Ptr->originFlag = ctrlParasPtr->originFlag;

		printf("originFlag = %d\r\n", ctrlParasPtr->originFlag);
	}

	if(Debug_Para_Ptr->rifdAdaptFlag != ctrlParasPtr->rifdAdaptFlag)
	{
		Debug_Para_Ptr->rifdAdaptFlag = ctrlParasPtr->rifdAdaptFlag;
		printf("rifdAdaptFlag = %d\r\n", ctrlParasPtr->rifdAdaptFlag);
		
	}

	if(Debug_Para_Ptr->gear != ctrlParasPtr->gear)
	{
		Debug_Para_Ptr->gear = ctrlParasPtr->gear;
		printf("gear = %d\r\n", ctrlParasPtr->gear);
		
	}

	if(Debug_Para_Ptr->agvStatus != ctrlParasPtr->agvStatus)
	{
		Debug_Para_Ptr->agvStatus = ctrlParasPtr->agvStatus;
		Show_AgvStatus(ctrlParasPtr->agvStatus);
	}

	
}





