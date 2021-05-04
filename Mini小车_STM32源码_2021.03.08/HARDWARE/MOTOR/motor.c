#include "motor.h"

/**************************************************************************
Function: Motor orientation pin initialization
Input   : none
Output  : none
�������ܣ�����������ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MiniBalance_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable port clock //ʹ�ܶ˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_A;
  //Push-pull output //�������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(IN1_PORTA, &GPIO_InitStructure);			
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_A;		
	//Push-pull output //�������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(IN2_PORTA, &GPIO_InitStructure);		
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_B;	
	//Push-pull output //�������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(IN1_PORTB, &GPIO_InitStructure);	
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_B;	
	//Push-pull output //�������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(IN2_PORTB, &GPIO_InitStructure);		
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_C;
	//Push-pull output //�������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(IN1_PORTC, &GPIO_InitStructure);		
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_C;	
	//Push-pull output //�������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(IN2_PORTC, &GPIO_InitStructure);		
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_D;		
	//Push-pull output //�������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(IN1_PORTD, &GPIO_InitStructure);		
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_D;	
	//Push-pull output //�������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(IN2_PORTD, &GPIO_InitStructure);		

  //IO output 0to prevent motor transfer
  //IO���0����ֹ�����ת
	GPIO_ResetBits(IN1_PORTA,IN1_PIN_A);	
	GPIO_ResetBits(IN2_PORTA,IN2_PIN_A);
	GPIO_ResetBits(IN1_PORTB,IN1_PIN_B);
	GPIO_ResetBits(IN2_PORTB,IN2_PIN_B);
	GPIO_ResetBits(IN1_PORTC,IN1_PIN_C);
	GPIO_ResetBits(IN2_PORTC,IN2_PIN_C);
	GPIO_ResetBits(IN1_PORTD,IN1_PIN_D);
	GPIO_ResetBits(IN2_PORTD,IN2_PIN_D);
}
/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
�������ܣ����PWM���ų�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	//Enable timer 8 //ʹ�ܶ�ʱ��8 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  
	//Enable GPIO peripheral clock //ʹ��GPIO����ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_A;
  //Reuse push-pull output //�����������  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(PWM_PORTA, &GPIO_InitStructure);
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_B;  
	//Reuse push-pull output //�����������  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(PWM_PORTB, &GPIO_InitStructure);
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_C; 
	//Reuse push-pull output //�����������  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(PWM_PORTC, &GPIO_InitStructure);
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_D;  
	//Reuse push-pull output //�����������  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(PWM_PORTD, &GPIO_InitStructure);
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	//Up counting mode 
	//���ϼ���ģʽ  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_Pulse = 0; 
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  
	//CH1 is pre-loaded and enabled
	//CH1Ԥװ��ʹ��	 
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	
	//Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ	
	TIM_OCInitStructure.TIM_Pulse = 0;       
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�		
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;    
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx	
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); 
	//CH2 is pre-loaded and enabled
	//CH2Ԥװ��ʹ��	
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	
	//Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ	
	TIM_OCInitStructure.TIM_Pulse = 0;       
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�		
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;    
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
  //CH3 is pre-loaded and enabled
	//CH3Ԥװ��ʹ��	  
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); 
	
	//Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ	
	TIM_OCInitStructure.TIM_Pulse = 0;  
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�		
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  
	//CH4 is pre-loaded and enabled
	//CH4Ԥװ��ʹ��	
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);	

  // Enable the TIMX preloaded register on the ARR
  //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���	
	TIM_ARRPreloadConfig(TIM8, ENABLE); 
	
	//Enable TIM8
	//ʹ��TIM8
	TIM_Cmd(TIM8, ENABLE);  
	
	// Advanced timer output must be enabled
	//�߼���ʱ���������ʹ�����		
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 
} 

/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
�������ܣ�ʹ�ܿ������ų�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void Enable_Pin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //Enable port clock  //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;             //Port configuration //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //Pull up input      //��������
  GPIO_Init(GPIOE, &GPIO_InitStructure);					      //Initialize GPIO with the specified parameters //�����趨������ʼ��GPIO
} 

/**************************************************************************
Function: Model aircraft three channel remote control pin and steering gear PWM pin initialization
Input   : ARR: Automatic reload value  PSC: clock preset frequency
Output  : none
�������ܣ���ģ��ͨ��ң����������PWM���ų�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc)	
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	//Enable reuse //���ø��ù���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//Pin remapping //������ӳ��
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
	//Enable timer 1 //ʹ�ܶ�ʱ��1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
  //Enable GPIO peripheral clock //ʹ��GPIO����ʱ��	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13; //Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //Pull-down input //��������     
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //Reuse push-pull output //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	GPIO_Init(GPIOE, &GPIO_InitStructure);

  /***Initialize timer TIM1***/
	/***��ʼ����ʱ�� TIM1***/
	//Set the counter to automatically reload //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Period = arr;
	//Pre-divider //Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
  //Set the clock split :TDTS = Tck_tim //����ʱ�ӷָ�:TDTS = Tck_tim	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  //TIM up count mode //TIM���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

  /*** initializes TIM1 input capture parameter, channel 1 ***/
	/*** ��ʼ��TIM1���벶�������ͨ��1 *************************/
	//CC1S=01 Select input port
	//CC1S=01   ѡ������� 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	//Rising edge capture
	//�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	//Configure input frequency division, regardless of frequency
  //���������Ƶ,����Ƶ 	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	//IC1F=0000 configuration input filter is not filtered
  //IC1F=0000 ���������˲��� ���˲�	
	TIM_ICInitStructure.TIM_ICFilter = 0x11;    
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

  /*** initializes TIM1 input capture parameter, channel 2 ***/
	/*** ��ʼ��TIM1���벶�������ͨ��2 *************************/
	//CC1S=01 Select input port
	//CC1S=01   ѡ�������
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;  
	//Rising edge capture
	//�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency
  //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	//IC1F=0000 configuration input filter is not filtered
  //IC1F=0000 ���������˲��� ���˲�	
	TIM_ICInitStructure.TIM_ICFilter = 0x11;   
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

  /*** initializes TIM1 input capture parameter, channel 3 ***/
	/*** ��ʼ��TIM1���벶�������ͨ��3 *************************/
	//CC1S=01 Select input port
	//CC1S=01   ѡ�������
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 
	//Rising edge capture
	//�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency
  //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  
	//IC1F=0000 configuration input filter is not filtered
  //IC1F=0000 ���������˲��� ���˲�	
	TIM_ICInitStructure.TIM_ICFilter = 0x00;   
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

  //  /*** initializes TIM1 input capture parameter, channel 4(Ackerman trolleys do not initialize channel 4 for input capture) ***/
	//  /*** ��ʼ��TIM1���벶�������ͨ��4(������С������ʼ��ͨ��4Ϊ���벶��) ************************************************/
	//  //CC1S=01 Select input port
	//  //CC1S=01   ѡ�������
	//  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	//  //Rising edge capture
	//  //�����ز���
	//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
	//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; ��
	//  //Configure input frequency division, regardless of frequency
  //  //���������Ƶ,����Ƶ 
	//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   
	//  //IC1F=0000 configuration input filter is not filtered
  //  //IC1F=0000 ���������˲��� ���˲�	
	//  TIM_ICInitStructure.TIM_ICFilter = 0x11;  
	//  TIM_ICInit(TIM1, &TIM_ICInitStructure); 
  
	/*** Ackerman trolleys do not initialize channel 4 for PWM output ***/
	/*** ������С������ʼ��ͨ��4ΪPWM��� ***/
	//Select Timer mode :TIM Pulse Width Modulation mode 1
	//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_Pulse = 0;  
	//Output polarity :TIM output polarity is higher
  //�������:TIM����Ƚϼ��Ը�	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
	//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); 
	//CH4 is pre-loaded to enable
  //CH4Ԥװ��ʹ��  	
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  

  //  //TIM1 interrupts //TIM1�ж�
	//  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  //  //Preempt priority 0 //��ռ���ȼ�0��  
	//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	//  //Level 0 from priority //�����ȼ�0��
	//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	//  //The IRQ channel is enabled //IRQͨ����ʹ��
	//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//  //Initialize the peripheral NVIC register according to the parameters specified in NVIC_INITSTRUCT
	//  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	//  NVIC_Init(&NVIC_InitStructure);   

	/*** //Interrupt packet initialization //�жϷ����ʼ�� ***/
	//Tim1 input captures interrupts //TIM1���벶���ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  
	//Preempt priority 0 //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	//Level 0 from priority //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	//The IRQ channel is enabled //IRQͨ����ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//Initialize the peripheral NVIC register according to the parameters specified in NVIC_INITSTRUCT 
	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	NVIC_Init(&NVIC_InitStructure);   

  //Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts
  //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�
	TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3,  ENABLE);    
	// Advanced timer output must be enabled
  //�߼���ʱ���������ʹ����� 	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);   
	//Enable timer
  //ʹ�ܶ�ʱ��	
	TIM_Cmd(TIM1, ENABLE);     
}

