#include "nvic_opts.h"

#define SCB_AIRCR_WRITE_KEY 0x05FA0000

// ����NVIC����
void My_NVIC_PriorityGroupConfig(u8 Nvic_Group)
{
	u32 temp, temp1;
	
	temp1 = (~Nvic_Group) & 0x07;		// ȡ����λ
	temp1 <<= 8;
	temp = SCB->AIRCR;					// ��ȡԭ������
	temp &= 0x0000F8FF;					// �����ǰ����[10:8]
	temp |= SCB_AIRCR_WRITE_KEY;		// д��KEY
	temp |= temp1;
	SCB->AIRCR = temp;					// ���÷���
}

// ����NVIC
void My_Nvic_Init(Nvic_Paramater node)
{
	u32 temp;
	u8 IprAddr = node.Nvic_Channel / 4;			// ���ַ,������ڵڼ���IPR�Ĵ�����
	u8 IprOffset = node.Nvic_Channel % 4;		// ����ƫ��,����������ĸ��е���һ��
	IprOffset = IprOffset * 8 + 4;				// ����������ƫ��ֵ,��4λ��Ч,��4λ��Ч,����Ҫ��ƫ��4
	
	My_NVIC_PriorityGroupConfig(node.Nvic_Group);		// ���÷���

	temp = node.Nvic_PreemptionPriority << (4 - node.Nvic_Group);	// ���ٸ�λ��ʾ��ռ���ȼ���,�ɷ������
	temp |= (node.Nvic_SubPriority & (0x0F >> node.Nvic_Group));	// �����ͼ�λ
	temp &= 0x0F;		// ȡ��4λ

	if(node.Nvic_Channel < 32)		// �ж�ʹ��
	{
		NVIC->ISER[0] |= (1 << node.Nvic_Channel);
	}
	else
	{
		NVIC->ISER[1] |= (1 << (node.Nvic_Channel - 32));
	}

	NVIC->IPR[IprAddr] |= (temp << IprOffset);		// ������ռ���ȼ�����Ӧ���ȼ�
}




