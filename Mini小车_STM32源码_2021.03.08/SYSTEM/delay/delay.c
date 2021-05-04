#include "delay.h"
#include "sys.h"
////////////////////////////////////////////////////////////////////////////////// 	 

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"	//The header file used by FreeRTOS //FreeRTOSʹ�õ�ͷ�ļ�		  
#include "task.h"     //The header file used by FreeRTOS //FreeRTOSʹ�õ�ͷ�ļ�	
#endif

static u8  fac_us=0; //��s delay multiplier //��s��ʱ������			   
static u16 fac_ms=0; //The ms delay multiple multiplier, under UCOS, represents the ms per beat //ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��
	
static u32 sysTickCnt=0;	
extern void xPortSysTickHandler(void);

/**************************************************************************
Function: Initialize the delay function
Input   : none
Output  : none
�������ܣ���ʼ���ӳٺ���
��ڲ�������
����  ֵ����
**************************************************************************/
void delay_init()
{
	u32 reload;
	
	// Select the external clock HCLK
	//ѡ���ⲿʱ��  HCLK
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	//Fac_us is required whether OS is used or not
	//�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
	fac_us=SystemCoreClock/1000000;
  //The number of counts per second is in M	
  //ÿ���ӵļ������� ��λΪM 	
	reload=SystemCoreClock/1000000;	
	//Set overflow time according to configTICK_RATE_HZ
  //Reload is a 24-bit register with a maximum value of 16777216 at 72M, about 0.233s
	//����configTICK_RATE_HZ�趨���ʱ��
	//reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��0.233s����	
	reload*=1000000/configTICK_RATE_HZ;	
  //Represents the minimum number of units that the OS can delay	
	//����OS������ʱ�����ٵ�λ											
	fac_ms=1000/configTICK_RATE_HZ;				   

	//Enable SysStick interrupt //����SYSTICK�ж�
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;  
	//Interrupt every 1/configTICK_RATE_HZ second
  //ÿ1/configTICK_RATE_HZ���ж�һ�� 	
	SysTick->LOAD=reload; 
  //Open the SYSTICK //����SYSTICK   	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	 
}									    
/**************************************************************************
Function: Delay function of delay n��s
Input   : Duration of delay
Output  : none
�������ܣ���ʱn��s���ӳٺ���
��ڲ�������ʱ��ʱ��
����  ֵ����
**************************************************************************/  								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;	//The value of the LOAD //LOAD��ֵ	    	 
	ticks=nus*fac_us; 				//The number of beats required //��Ҫ�Ľ����� 
	told=SysTick->VAL;        //The value of the counter on entry //�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//SysStick is a decrement counter //����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//If the delay time exceeds or equals the delay time, exit //ʱ�䳬�����ߵ���Ҫ�ӳٵ�ʱ��,���˳�
		}  
	};										    
}  
/**************************************************************************
Function: Delay function of Delay nms
Input   : Duration of delay
Output  : none
�������ܣ���ʱnms���ӳٺ���
��ڲ�������ʱ��ʱ��
����  ֵ����
**************************************************************************/  
void delay_ms(u32 nms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED) //The system is running //ϵͳ�Ѿ�����
	{		
		if(nms>=fac_ms)	//The duration of the delay is greater than the OS minimum duration //��ʱ��ʱ�����OS������ʱ������ 
		{ 
   			vTaskDelay(nms/fac_ms);	//FreeRTOS latency //FreeRTOS��ʱ
		}
		nms%=fac_ms;	//OS is no longer able to provide such a small delay, use normal delay //OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
	}
	delay_us((u32)(nms*1000));	//Normal mode delay	//��ͨ��ʽ��ʱ
}
/**************************************************************************
Function: Delay function of delay ms
Input   : Duration of delay
Output  : none
�������ܣ���ʱms���ӳٺ���
��ڲ�������ʱ��ʱ��
����  ֵ����
**************************************************************************/  
void delay_xms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

/**************************************************************************
Function: Tick timer interrupt service function
Input   : Duration of delay
Output  : none
�������ܣ��δ�ʱ���жϷ�����
��ڲ�������ʱ��ʱ��
����  ֵ����
**************************************************************************/  
void SysTick_Handler(void)
{	
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED) //The system is running //ϵͳ�Ѿ�����
    {
        xPortSysTickHandler();	
    }
		else
		{
		sysTickCnt++;	//Counting before scheduling is enabled //���ȿ���֮ǰ����
	}
}
/**************************************************************************
Function: Get the time of the last execution function
Input   : none
Output  : The last time the program was executed
�������ܣ���ȡ�ϴ�ִ�к�����ʱ��
��ڲ�������
����  ֵ����һ��ִ�г����ʱ��
**************************************************************************/ 
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	//The system is running //ϵͳ�Ѿ�����
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}

