#include "nvic_opts.h"

#define SCB_AIRCR_WRITE_KEY 0x05FA0000

// 这是NVIC分组
void My_NVIC_PriorityGroupConfig(u8 Nvic_Group)
{
	u32 temp, temp1;
	
	temp1 = (~Nvic_Group) & 0x07;		// 取后三位
	temp1 <<= 8;
	temp = SCB->AIRCR;					// 读取原本设置
	temp &= 0x0000F8FF;					// 清空先前分组[10:8]
	temp |= SCB_AIRCR_WRITE_KEY;		// 写入KEY
	temp |= temp1;
	SCB->AIRCR = temp;					// 设置分组
}

// 设置NVIC
void My_Nvic_Init(Nvic_Paramater node)
{
	u32 temp;
	u8 IprAddr = node.Nvic_Channel / 4;			// 组地址,计算出在第几组IPR寄存器中
	u8 IprOffset = node.Nvic_Channel % 4;		// 组内偏移,计算出组内四个中的哪一个
	IprOffset = IprOffset * 8 + 4;				// 计算出具体的偏移值,高4位有效,低4位无效,所以要多偏移4
	
	My_NVIC_PriorityGroupConfig(node.Nvic_Group);		// 设置分组

	temp = node.Nvic_PreemptionPriority << (4 - node.Nvic_Group);	// 多少个位表示抢占优先级的,由分组决定
	temp |= (node.Nvic_SubPriority & (0x0F >> node.Nvic_Group));	// 保留低几位
	temp &= 0x0F;		// 取低4位

	if(node.Nvic_Channel < 32)		// 中断使能
	{
		NVIC->ISER[0] |= (1 << node.Nvic_Channel);
	}
	else
	{
		NVIC->ISER[1] |= (1 << (node.Nvic_Channel - 32));
	}

	NVIC->IPR[IprAddr] |= (temp << IprOffset);		// 设置抢占优先级和响应优先级
}




