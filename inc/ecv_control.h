#ifndef __ECV_CONTROL_H__
#define __ECV_CONTROL_H__

#include "data_type.h"
#include "common_include.h"

#include "pwm_opts.h"
#include "cfg_gpio.h"


#define USE_ECV				1


#define ECV1_PWM				PBout(8)		// ǰ���
#define ECV1_DIR				PCout(14)		// ��׷��� 0: ��   1: ��

#define ECV2_PWM				PBout(9)		// ����
#define ECV2_DIR				PCout(15)		// ��׷��� 0: ��   1: ��

#define ECV3_PWM				PEout(8)		// 
#define ECV3_DIR				PEout(7)		// 

#define LMT_INR					PCin(3)			// ��ӦΪ 0
#define LMT_INL					PCin(4)			// ��ӦΪ 0

#define LMT_SW					PEin(0)			// ��ӦΪ 0
#define Return_SW				PDin(10)		// ��ӦΪ 0

#define KEY_1					PCin(1)			// ��ӦΪ 0
#define KEY_2					PCin(2)			// ��ӦΪ 0

#define LMT2_SW					PEin(2)			//

#define BUZZER_1				PEout(9)
#define BUZZER_2				PEout(10)

#define ECV_POWER_ON()			{ECV1_POWER = 0; ECV2_POWER = 0; ECV3_POWER = 0;}
#define ECV_POWER_OFF()			{ECV1_POWER = 1; ECV2_POWER = 1; ECV3_POWER = 1;}

#define FECV_UP()				{/*ECV2_PWM = 1;*/   ECV2_DIR = 0;  }		//
#define FECV_DOWN()				{/*ECV2_PWM = 1;*/   ECV2_DIR = 1;  }
#define FECV_STOP()				{ECV2_PWM = 0;					}

#define BECV_UP()				{/*ECV1_PWM = 1;*/	 ECV1_DIR = 1;  }
#define BECV_DOWN()				{/*ECV1_PWM = 1;*/   ECV1_DIR = 0;  }
#define BECV_STOP()				{ECV1_PWM = 0;					}

#define WECV_UP()				{ECV3_PWM = 1;   ECV3_DIR = 1;  }
#define WECV_DOWN()				{ECV3_PWM = 1;   ECV3_DIR = 0;  }
#define WECV_STOP()				{ECV3_PWM = 0;					}


#define FECV_SPEED_SET(speed)	{pwmOptsPtr_1->Duty_Cycle_OC3_Set(timer4PwmParaPtr, speed);}
#define BECV_SPEED_SET(speed)	{pwmOptsPtr_1->Duty_Cycle_OC4_Set(timer4PwmParaPtr, speed);}


typedef struct
{
	s8	EcvDir;					// ���Ƶ�׷���, > 0 Ϊ����, = 0 Ϊֹͣ, < 0 Ϊ�½�
	u8	EcvEnableHallFlag;		// ��׻����źż���ʹ��
	u16	EcvHallCount;			// ��׻����źż���
	u16 EcvHallCountCmpSet;		// ���������ȽϵĻ�������
	u16 EcvSpeed;				// ����ٶ�, ��Χ 0 ~ 100
	
}Ecv_Ctrl_Struct, *Ecv_Ctrl_Struct_P;








void ECV_Init(void);


extern PwmParaStruct_P 		timer4PwmParaPtr;
extern Ecv_Ctrl_Struct_P 	FECV_Str_Ptr;
extern Ecv_Ctrl_Struct_P 	BECV_Str_Ptr;


#endif







