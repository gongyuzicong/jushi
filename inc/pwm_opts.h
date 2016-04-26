#ifndef __PWM_OPTS_H__
#define __PWM_OPTS_H__


#include "data_type.h"
#include "common_include.h"

typedef struct
{
	TIM_TypeDef *TimX;
	
	u8 Oc1DutyCycle;
	u8 Oc2DutyCycle;
	u8 Oc3DutyCycle;
	u8 Oc4DutyCycle;
	u8 Frequency;
	
	u8 PwmResolution;
	u16 PwmFrequencyCounterNum;
	u16 ResolutionPreStep;
}PwmParaStruct, *PwmParaStruct_P;

typedef struct
{
	u8 (*Pwm_Frequency_Set)(PwmParaStruct_P);

	void (*Duty_Cycle_OC1_Set)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC2_Set)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC3_Set)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC4_Set)(PwmParaStruct_P, u8);
	
	void (*Duty_Cycle_OC1_Add)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC2_Add)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC3_Add)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC4_Add)(PwmParaStruct_P, u8);
	
	void (*Duty_Cycle_OC1_Sub)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC2_Sub)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC3_Sub)(PwmParaStruct_P, u8);
	void (*Duty_Cycle_OC4_Sub)(PwmParaStruct_P, u8);
}PwmOperaterStruct, *PwmOperaterStruct_P;


u8 Pwm_Init(void);

extern PwmOperaterStruct_P pwmOptsPtr_1;
extern PwmParaStruct_P pwmParaPtr_1; 


#endif




