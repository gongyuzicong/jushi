#ifndef __PID_H__
#define __PID_H__


#include "common_include.h"


typedef struct
{
	float SetSpeed;		// �����趨ֵ
	float ActualSpeed;	// ����ʵ��ֵ
	float err;			// ����ƫ��ֵ
	float err_last;		// ������һ��ƫ��ֵ
	float Kp, Ki, Kd;	// �������,����,΢��ϵ��
	float voltage;		// �����ѹֵ(����ִ�����ı���)
	float integral;		// �������ֵ
}PID_LOC_SPEED;

typedef struct
{
	float SetSpeed;		// �����趨ֵ
	float ActualSpeed;	// ����ʵ��ֵ
	float err;			// ����ƫ��ֵ
	float err_next;		// ������һ��ƫ��ֵ
	float err_last;		// ��������ǰ��һ��ƫ��ֵ
	float Kp, Ki, Kd;	// �������,����,΢��ϵ��
}PID_INC_SPEED;


#endif








