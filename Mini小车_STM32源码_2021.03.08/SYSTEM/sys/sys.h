#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"

//0,Does not support OS //0,��֧��os
//1,support OS          //1,֧��os
#define SYSTEM_SUPPORT_OS		1		//Define whether the system folder supports OS //����ϵͳ�ļ����Ƿ�֧��OS

//Bit band operation to achieve 51 similar GPIO control function
//Refer to < / p > < p >;&lt;CM3 Authoritative Guide &gt;&gt;Chapter 5 (pp. 87 ~92).
//IO port operation macro definition
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

//IO port address mapping //IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO port operation, only single IO port!
//make sure n is less than 16!
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //Output //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //Input  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //Output //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //Input  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //Output //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //Input  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //Output //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //Input  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //Output //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //Input  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //Output //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //Input  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //Output //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //Input  //����

void Stm32_Clock_Init(u8 PLL); //Clock initialization //ʱ�ӳ�ʼ��  
void Sys_Soft_Reset(void);     //System soft reset    //ϵͳ��λ
void Sys_Standby(void);        //Standby mode         //����ģʽ 	
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset); //Set the offset address //����ƫ�Ƶ�ַ
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group); //Set the NVIC grouping //����NVIC����
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group); //Set interrupt //�����ж�
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM); //External interrupt configuration function (gpioA ~G only) //�ⲿ�ж����ú���(ֻ��GPIOA~G)
void JTAG_Set(u8 mode);

//The following are assembler functions //����Ϊ��ຯ��
void WFI_SET(void);		   //Execute the WFI instruction //ִ��WFIָ��
void INTX_DISABLE(void); //Turn off all interrupts //�ر������ж�
void INTX_ENABLE(void);	 //Enables all interrupts  //���������ж�
void MSR_MSP(u32 addr);	 //Set the stack address   //���ö�ջ��ַ

//JTAG mode setting definition
//JTAGģʽ���ö���
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	
void JTAG_Set(u8 mode);
#endif
