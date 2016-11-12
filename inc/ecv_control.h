#ifndef __ECV_CONTROL_H__
#define __ECV_CONTROL_H__

#include "data_type.h"
#include "common_include.h"

#include "pwm_opts.h"
#include "cfg_gpio.h"


#define USE_ECV					1

#define USE_WECV				0

#define WECV_UP_LIMT_SW			PEin(1)
#define WECV_DOWN_LIMT_SW		PEin(3)
#define WECV_UP_LIMT_SW_RESP	(0 == WECV_UP_LIMT_SW)
#define WECV_DOWN_LIMT_SW_RESP	(0 == WECV_DOWN_LIMT_SW)	// ���������� ECV_Ctrl_Func_W �������֮��ĵط��Ͻ�ʹ��

#define ECV1_PWM				PBout(8)		// ǰ���
#define ECV1_DIR				PCout(14)		// ��׷��� 0: ��   1: ��

#define ECV2_PWM				PBout(9)		// ����
#define ECV2_DIR				PCout(15)		// ��׷��� 0: ��   1: ��

#define ECV3_PWM				PEout(8)		// 
#define ECV3_DIR				PEout(7)		// 

#define LMT_INR					PCin(3)			// ��ӦΪ 0
#define LMT_INL					PCin(4)			// ��ӦΪ 0

#define LMT_SW					PEin(0)			// ��ӦΪ 0
#define LMT_SW_Respond			(0 == LMT_SW)
#define LMT_SW_UnRespond		(1 == LMT_SW)

#define Return_SW_LF				PCin(1)		// ��ӦΪ 0
#define Return_SW_LF_Respond		(0 == Return_SW_LF)
#define Return_SW_LF_UnRespond		(1 == Return_SW_LF)

#define Return_SW_LR				PDin(9)		// ��ӦΪ 0
#define Return_SW_LR_Respond		(0 == Return_SW_LR)
#define Return_SW_LR_UnRespond		(1 == Return_SW_LR)

#define Return_SW_RF				PDin(10)		// ��ӦΪ 0
#define Return_SW_RF_Respond		(0 == Return_SW_RF)
#define Return_SW_RF_UnRespond		(1 == Return_SW_RF)

#define Return_SW_RR				PDin(11)		// ��ӦΪ 0
#define Return_SW_RR_Respond		(0 == Return_SW_RR)
#define Return_SW_RR_UnRespond		(1 == Return_SW_RR)

#define Return_SW_Respond			(Return_SW_LF_Respond || Return_SW_LR_Respond || Return_SW_RF_Respond || Return_SW_RR_Respond)
#define Return_SW_UnRespond			(Return_SW_LF_UnRespond && Return_SW_LR_UnRespond && Return_SW_RF_UnRespond && Return_SW_RR_UnRespond)

#define KEY_1					PCin(1)			// ��ӦΪ 0
#define KEY_2					PCin(2)			// ��ӦΪ 0

#define FLMT_SW					PEin(2)			//
#define RLMT_SW					PCin(2)
#define FLMT_SW_RESPOND			(0 == FLMT_SW)
#define RLMT_SW_RESPOND			(0 == RLMT_SW)
#define FLMT_SW_UNRESPOND		(1 == FLMT_SW)
#define RLMT_SW_UNRESPOND		(1 == RLMT_SW)

#define BECV_SW					PEin(5)
#define BECV_SW_RESPOND			(0 == BECV_SW)
#define BECV_SW_UNRESPOND		(1 == BECV_SW)


#define BUZZER_1				PEout(9)
#define BUZZER_2				PEout(10)

#define ECV1_POWER				PDout(13)
#define ECV2_POWER				PDout(14)
#define ECV3_POWER				PEout(11)	//

#if 0
#define FECV_UP()				{ECV2_PWM = 1;   ECV2_DIR = 0;  }		//
#define FECV_DOWN()				{ECV2_PWM = 1;   ECV2_DIR = 1;  }
#define FECV_STOP()				{ECV2_PWM = 0;					}
#define FECV_POWER_ON()			{ECV2_POWER = 1;				}
#define FECV_POWER_OFF()		{ECV2_POWER = 0;				}

#define BECV_UP()				{ECV1_PWM = 1;	 ECV1_DIR = 1;  }
#define BECV_DOWN()				{ECV1_PWM = 1;   ECV1_DIR = 0;  }
#define BECV_STOP()				{ECV1_PWM = 0;					}
#define BECV_POWER_ON()			{ECV1_POWER = 1;				}
#define BECV_POWER_OFF()		{ECV1_POWER = 0;				}

#define WECV_UP()				{ECV3_PWM = 1;   ECV3_DIR = 0;  }
#define WECV_DOWN()				{ECV3_PWM = 1;   ECV3_DIR = 1;  }
#define WECV_STOP()				{ECV3_PWM = 0;					}
#define WECV_POWER_ON()			{ECV3_POWER = 1;				}
#define WECV_POWER_OFF()		{ECV3_POWER = 0;				}

#else

#define FECV_UP()				{ECV1_DIR = 0;  }		//
#define FECV_DOWN()				{ECV1_DIR = 1;  }
#define FECV_STOP()				{ECV1_PWM = 0;	}
#define FECV_POWER_ON()			{ECV1_POWER = 0;}
#define FECV_POWER_OFF()		{ECV1_POWER = 1;}

#define BECV_UP()				{ECV2_DIR = 0;  }
#define BECV_DOWN()				{ECV2_DIR = 1;  }
#define BECV_STOP()				{ECV1_PWM = 0;	}
#define BECV_BRK_ENABLE()		{ECV3_DIR = 1;	}
#define BECV_BRK_DISABLE()		{ECV3_DIR = 0;	}
#define BECV_POWER_ON()			{ECV2_POWER = 0;}
#define BECV_POWER_OFF()		{ECV2_POWER = 1;}

#define WECV_UP()				{ECV3_DIR = 1;  }
#define WECV_DOWN()				{ECV3_DIR = 0;  }
#define WECV_STOP()				{ECV3_PWM = 1;	}
#define WECV_POWER_ON()			{ECV3_POWER = 0;}
#define WECV_POWER_OFF()		{ECV3_POWER = 1;}

#endif

#define ECV_ALL_POWER_ON()		{FECV_POWER_ON();  BECV_POWER_ON();  WECV_POWER_ON();}
#define ECV_ALL_POWER_OFF()		{FECV_POWER_OFF(); BECV_POWER_OFF(); WECV_POWER_OFF();}


#define ECV1_HALL				FECV_Str_Ptr->EcvHallCount
#define ECV2_HALL				BECV_Str_Ptr->EcvHallCount
#define ECV3_HALL				WECV_Str_Ptr->EcvHallCount

#define ECV1_HALL_FLAG			FECV_Str_Ptr->EcvEnableHallFlag
#define ECV2_HALL_FLAG			BECV_Str_Ptr->EcvEnableHallFlag
#define ECV3_HALL_FLAG			WECV_Str_Ptr->EcvEnableHallFlag

#define FECV_SPEED_SET(speed)	{pwmOptsPtr_1->Duty_Cycle_OC3_Set(timer4PwmParaPtr, speed);}
#define BECV_SPEED_SET(speed)	{pwmOptsPtr_1->Duty_Cycle_OC4_Set(timer4PwmParaPtr, speed);}

#define FEcvHallCountCmp_Addr	0x0000
#define BEcvHallCountCmp_Addr	0x0000
#define WEcvHallCountCmp_Addr	0x0000


typedef enum
{
	ECV_DOWN = -1,		// ����½�
	ECV_STOP = 0,		// ���ֹͣ
	ECV_UP,				// �������
}EcvDir;

typedef enum
{
	ECV_USE_HALL_COUNT_MODE_DISABLE = 0,		// Ϊ���ƺ�����ʹ�û��������Ա�ģʽ,һ�������߼���λ���û��߳�ʼ����
	ECV_USE_HALL_COUNT_MODE_ENABLE,				// Ϊ���ƺ���ʹ�û��������Ա�ģʽ
}EcvHallCountMode;

typedef enum
{
	ECV_POWER_OFF = 0,
	ECV_POWER_ON,
}ECV_PowerOnOff;

typedef enum
{
	ECV_BRK_ENABLE,
	ECV_BRK_DISABLE,
}ECV_BRK, *ECV_BRK_P;

typedef enum
{
	ECV_UNUSED = 0,		// ���δʹ��
	ECV_USEING,			// �������ʹ��
	ECV_COMPLETE,
	ECV_TIMEOUT_ERR,
}EcvUseStatus;

typedef struct
{
	// FECV
	u16 FecvBigFiberHall;		// ǰ��״�ɴ�Ż�������
	u16 FecvSmallFiberHall;		// ǰ���Сɴ�Ż�������

	// BECV
	u16 BecvInit;				// ���׳�ʼ����������
	u16 BecvBigFiberHall;		// ���״�ɴ�Ż�������
	u16 BecvSmallFiberHall;		// ����Сɴ�Ż�������
	
}HallCountCmpManagerStruct, *HallCountCmpManagerStruct_P;

typedef struct
{
	u8 	EcvSpeed;									// ����ٶ�, ��Χ 0 ~ 100
	u16 EcvHallCountCmp;							// ���������ȽϵĻ�������
	
	EcvDir				Dir;						// ���Ƶ�׷���
	EcvHallCountMode 	HallCountMode;				// ���������Ա�ģʽ
}Ecv_Para, *Ecv_Para_P;

typedef enum
{
	ECV_UNKNOW,
	ECV_UP_LIMT,
	ECV_DOWN_LIMT,
}EcvLocation, *EcvLocation_P;

typedef struct EcvCtrlStruct
{
	u8 	EcvSpeed;									// ����ٶ�, ��Χ 0 ~ 100
	u8 	EcvSpeedRec;								// ��¼����ٶȵ���ʱ����
	u8	EcvEnableHallFlag;							// ��׻����źż���ʹ��
	u16	EcvHallCount;								// ��׻����źż���
	u16 EcvHallCountRec;							// ��׻����źż�����ʱ����
	u32 EcvHallCountTimeRec;						// ��׻����źż������ʱ��
	u32 EcvHallCountTimeOut_ms;						// ��׻����źż�����ʱʱ��, һ�㲻Ҫ����
	u16 EcvHallCountCmp;							// ���������ȽϵĻ�������
	u16 EcvHallCountTimeout;						// ��ʱ��ʱ��Ļ�������ֵ(��ֵ���ⲿ����)
	u16 EcvHallCountTimeoutUpdate;					// ��ʱ��������ֵ���±�־λ(��ֵ���ⲿ����)
	EcvDir				Dir;						// ���Ƶ�׷���
	ECV_PowerOnOff 		Power;						// ���Ƶ�׵�Դ
	EcvHallCountMode 	HallCountMode;				// ���������Ա�ģʽ
	EcvUseStatus 		UseStatus;					// ���ʹ��״̬
	EcvLocation			Location;
	
	void (*ECV_PowerOnOffFunc)(ECV_PowerOnOff);		// ��׵�Դ���غ��� 0: �ر� 1: ����
	void (*ECV_UpDownFunc)(EcvDir);					// �����������
	void (*ECV_SetSpeedFunc)(u8);					// ����ٶ����ú���
	void (*ECV_SetPara)(Ecv_Para_P);				// ���õ�׿��Ʋ���
	void (*ECV_Clean_Use_Status)(void);				// ������ʹ��״̬
	u8   (*Check_ECV_SW_Status)(void);				// ����׼��޿���״̬ 1: Ϊ���Ｋ��λ��
	void (*ECV_BRK)(ECV_BRK);						//
	void (*ECV_Ctrl_Function)(struct EcvCtrlStruct *);
}Ecv_Ctrl_Struct, *Ecv_Ctrl_Struct_P;




void ECV_Init(void);
void ECV_Ctrl_Func_F(Ecv_Ctrl_Struct_P);
void Machine_Arm_Init3(void);



extern PwmParaStruct_P 		timer4PwmParaPtr;
extern Ecv_Ctrl_Struct_P 	FECV_Str_Ptr;
extern Ecv_Ctrl_Struct_P 	BECV_Str_Ptr;
extern Ecv_Ctrl_Struct_P 	WECV_Str_Ptr;
extern HallCountCmpManagerStruct_P HallCountCmpManager_Str_Ptr;

#endif







