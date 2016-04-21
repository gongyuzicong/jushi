#ifndef __KEY_OPTS_H__
#define __KEY_OPTS_H__

#include "common_include.h"
#include "data_type.h"

#define KEY_NUM  4
#define KEY_SATAUS_STEP 3

#define KEY1 0
#define KEY2 1
#define KEY3 2
#define KEY4 3

typedef enum
{
    KeyScanState_Check = 0x00,
    KeyScanState_MakeSure = 0x01,
    KeyScanState_Up = 0x02,
}KeyScanState_Typedef;


extern u8 keyScanFlag;


int keyScan(void);
void keyEvent(int key);
void keyConfig(void);

void Key_1(void);
void Key_3(void);






#endif





