#include "sys.h"

//The THUMB directive does not support assembly inlining
//The following method is used to implement assembly instruction WFI
//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}

//Turn off all interrupts
//�ر������ж�
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}

//Enables all interrupts
//���������ж�
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}

//Set the stack top address
//addr: the top of the stack address
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //Turn on the auxiliary clock //��������ʱ��	   
	AFIO->MAPR&=0XF8FFFFFF; //Clear MAPR [26:24] //���MAPR��[26:24]
	AFIO->MAPR|=temp;       //Set the JTAG mode //����jtagģʽ
} 

