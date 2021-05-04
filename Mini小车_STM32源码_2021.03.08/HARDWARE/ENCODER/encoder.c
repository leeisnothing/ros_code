#include "encoder.h"

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : none
Output  : none
�������ܣ���TIM2��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
�� �� ֵ����
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable the timer 2 clock //ʹ�ܶ�ʱ��2��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//Enable PinA, PinB port clocks //ʹ��PinA��PinB�˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	//Enable reuse //���ø��ù���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
	//Disable JTAG to enable SWD //�ر�JTAG��ʹ��SWD
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	//Pin remapping //������ӳ��
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);	 
	
	//Port configuration, PA15 //�˿����ã�PA15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	
	//Float input //��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//Initialize GPIOA with the set parameters //�����趨������ʼ��GPIOA
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      
  
	//Port configuration, PB3 //�˿����ã�PB3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	
	//Float input //��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //Initialize the gpiob by setting parameters //�����趨������ʼ��GPIOB	
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	//Pre-divider setup //Ԥ��Ƶ������ 
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	//Set the counter to automatically reload//�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; 
	//Select the clock frequency division: no frequency //ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	//Up counting mode //���ϼ���ģʽ  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//Initialize timer 2 //��ʼ����ʱ��2
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  
	//Use encoder mode 3 //ʹ�ñ�����ģʽ3
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	//Fill in each parameter in the TIM_ICInitStruct with the default value
	//��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
	TIM_ICStructInit(&TIM_ICInitStructure);
  //Set the filter length //�����˲�������	
	TIM_ICInitStructure.TIM_ICFilter = 10;
	//Initialize the peripheral TIMX based on the parameter TIM_ICINITSTRUCT
  //���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx  
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
 
  //Clear the update bit for Tim //���TIM�ĸ��±�־λ
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	//Enable the timer to interrupt //ʹ�ܶ�ʱ���ж�
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	//Reset the timer count //���ö�ʱ������
	TIM_SetCounter(TIM2,0); 
	//Enable timer 2 //ʹ�ܶ�ʱ��2
	TIM_Cmd(TIM2, ENABLE); 
}

/**************************************************************************
Function: Initialize TIM3 as the encoder interface mode
Input   : none
Output  : none
�������ܣ���TIM3��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable the timer 3 clock //ʹ�ܶ�ʱ��3��ʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//Enable PA port clock //ʹ��PA�˿�ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//Enable reuse //���ø��ù���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//Port configuration, PA6, PA7 //�˿����ã�PA6��PA7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	
	//Float input //��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //Initialize GPIOA with the set parameters //�����趨������ʼ��GPIOA	
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      
  	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	//Set up the pre-divider //����Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	//Set the counter to automatically reload //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
  //Select the clock frequency division: no frequency //ѡ��ʱ�ӷ�Ƶ������Ƶ	
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	//Up counting mode //���ϼ���ģʽ 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//Initialize timer 3//��ʼ����ʱ��3
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  
	
	//Use encoder mode 3 //ʹ�ñ�����ģʽ3
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	//Fill in each parameter in the TIM_ICInitStruct with the default value
	//��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
  TIM_ICStructInit(&TIM_ICInitStructure); 
	//Set the filter length //�����˲�������
  TIM_ICInitStructure.TIM_ICFilter = 10; 
  //Initialize the peripheral TIMX based on the parameter TIM_ICINITSTRUCT //���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx	
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
 
  //Clear the update bit for Tim //���TIM�ĸ��±�־λ
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	//Enable the timer to interrupt //ʹ�ܶ�ʱ���ж�
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	//Reset the timer count //���ö�ʱ������
	TIM_SetCounter(TIM3,0);
	//Enable timer 3 //ʹ�ܶ�ʱ��3
	TIM_Cmd(TIM3, ENABLE); 
}
/**************************************************************************
Function: Initialize TIM4 as the encoder interface mode
Input   : none
Output  : none
�������ܣ���TIM4��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
�� �� ֵ����
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable timer 4 clock //ʹ�ܶ�ʱ��4��ʱ�� 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//Enable pB port clock //ʹ��PB�˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//Port configuration, PB6, PB7 //�˿����ã�PB6��PB7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  //Float input //��������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//Initialize GPIOB with the specified parameters //�����趨������ʼ��GPIOB
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      
  
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	//Set up the pre-divider //����Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	//Set the counter to automatically reload //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; 
	//Select the clock frequency division: no frequency //ѡ��ʱ�ӷ�Ƶ������Ƶ	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	//Up counting mode //���ϼ���ģʽ 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//Initialize timer 4//��ʼ����ʱ��4
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	//Use encoder mode 3 //ʹ�ñ�����ģʽ3
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	//Fill in each parameter in the TIM_ICInitStruct with the default value
	//��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
	TIM_ICStructInit(&TIM_ICInitStructure);
	//Set the filter length //�����˲�������
	TIM_ICInitStructure.TIM_ICFilter = 10;
	//Initialize the peripheral TIMX based on the parameter TIM_ICINITSTRUCT //���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx	
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	//Clear the update bit for Tim //���TIM�ĸ��±�־λ
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	//Enable the timer to interrupt //ʹ�ܶ�ʱ���ж�
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	//Reset the timer count //���ö�ʱ������
	TIM_SetCounter(TIM4,0);
	//Enable timer 4 //ʹ�ܶ�ʱ��4
	TIM_Cmd(TIM4, ENABLE); 
}
/**************************************************************************
Function: Initialize TIM5 as the encoder interface mode
Input   : none
Output  : none
�������ܣ���TIM5��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM5(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable timer 5 clock //ʹ�ܶ�ʱ��5��ʱ�� 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	//Enable pA port clock //ʹ��PA�˿�ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//Port configuration, PA0, PA1 //�˿����ã�PA0��PA1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	
	//Float input //��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//Initialize GPIOA with the specified parameters //�����趨������ʼ��GPIOA
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	//Set up the pre-divider //����Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	//Set the counter to automatically reload //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
	//Select the clock frequency division: no frequency //ѡ��ʱ�ӷ�Ƶ������Ƶ	
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	//Up counting mode //���ϼ���ģʽ 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//Initialize timer 5//��ʼ����ʱ��5
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	//Use encoder mode 3 //ʹ�ñ�����ģʽ3
  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	//Fill in each parameter in the TIM_ICInitStruct with the default value
	//��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
  TIM_ICStructInit(&TIM_ICInitStructure); 
	//Set the filter length //�����˲�������
  TIM_ICInitStructure.TIM_ICFilter = 10;
	//Initialize the peripheral TIMX based on the parameter TIM_ICINITSTRUCT //���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx	
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
 
  //Clear the update bit for Tim //���TIM�ĸ��±�־λ
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	//Enable the timer to interrupt //ʹ�ܶ�ʱ���ж�
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	//Reset the timer count //���ö�ʱ������
  TIM_SetCounter(TIM5,0);
	//Enable timer 5 //ʹ�ܶ�ʱ��5
  TIM_Cmd(TIM5, ENABLE);  
}

/**************************************************************************
Function: Read the encoder count
Input   : The timer
Output  : Encoder value (representing speed)
�������ܣ���ȡ����������
��ڲ�������ʱ��
����  ֵ����������ֵ(�����ٶ�)
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
 int Encoder_TIM;    
 switch(TIMX)
 {
	case 2:  Encoder_TIM= (short)TIM2 -> CNT;   TIM2 -> CNT=0;  break;
	case 3:  Encoder_TIM= (short)TIM3 -> CNT;   TIM3 -> CNT=0;  break;
	case 4:  Encoder_TIM= (short)TIM4 -> CNT;   TIM4 -> CNT=0;  break;	
	case 5:  Encoder_TIM= (short)TIM5 -> CNT;   TIM5 -> CNT=0;  break;	
	default: Encoder_TIM=0;
 }
	return Encoder_TIM;
}

/**************************************************************************
Function: Tim2 interrupt service function
Input   : none
Output  : none
�������ܣ�TIM2�жϷ�����
��ڲ�������
�� �� ֵ����
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001) //Overflow interrupt //����ж�
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0); //Clear the interrupt flag bit //����жϱ�־λ 	    
}
/**************************************************************************
Function: Tim3 interrupt service function
Input   : none
Output  : none
�������ܣ�TIM3�жϷ�����
��ڲ�������
�� �� ֵ����
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001) //Overflow interrupt //����ж�
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0); //Clear the interrupt flag bit //����жϱ�־λ  	    
}
/**************************************************************************
Function: Tim4 interrupt service function
Input   : none
Output  : none
�������ܣ�TIM4�жϷ�����
��ڲ�������
�� �� ֵ����
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001) //Overflow interrupt //����ж�
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0); //Clear the interrupt flag bit //����жϱ�־λ  	    
}

/**************************************************************************
Function: Tim5 interrupt service function
Input   : none
Output  : none
�������ܣ�TIM5�жϷ�����
��ڲ�������
�� �� ֵ����
**************************************************************************/
void TIM5_IRQHandler(void)
{ 		    		  			    
	if(TIM5->SR&0X0001) //Overflow interrupt //����ж�
	{    				   				     	    	
	}				   
	TIM5->SR&=~(1<<0); //Clear the interrupt flag bit //����жϱ�־λ  	    
}
