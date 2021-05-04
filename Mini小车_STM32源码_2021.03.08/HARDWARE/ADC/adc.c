#include "adc.h"

float Voltage,Voltage_Count,Voltage_All; //Variables related to battery voltage sampling //��ص�ѹ������صı���  
const float Revise=1.03;

/**************************************************************************
Function: ADC initializes battery voltage detection
Input   : none
Output  : none
�������ܣ�ADC��ʼ����ص�ѹ���
��ڲ�������
����  ֵ����
**************************************************************************/
void  Adc_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable ADC1 channel clock
	//ʹ��ADC1ͨ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	 
	//Set ADC frequency dividing factor 6 72M/6=12, and the maximum ADC time shall not exceed 14M
  //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //Power supply voltage analog input pin //��Դ��ѹģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	//Potentiometer analog input pin, used to switch car model //��λ��ģ���������ţ������л�С���ͺ�
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Reset ADC1. Reset all registers of peripheral ADC1 to their default values
	//��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_DeInit(ADC1); 
  //ADC working mode :ADC1 and ADC2 work in standalone mode	
	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	
	//The analog-to-digital conversion works in single-channel mode
	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	
	//A to D conversion works in single conversion mode
	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	
	//The transformation is initiated by software rather than an external trigger
	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	
	//Right aligned ADC data
	//ADC�����Ҷ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//The number of ADC channels that are sequentially converted to a rule
  //˳����й���ת����ADCͨ������Ŀ	
	ADC_InitStructure.ADC_NbrOfChannel = 1;	
	//Initialize the external ADCX register according to the parameters specified in ADC_INITSTRUCT
	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
	ADC_Init(ADC1, &ADC_InitStructure);	
	//Enable the specified ADC1
	//ʹ��ָ����ADC1
	ADC_Cmd(ADC1, ENABLE);
	//Enable reset calibration
  //ʹ�ܸ�λУ׼ 	
	ADC_ResetCalibration(ADC1);	 
  //Wait for the reset calibration to finish	
	//�ȴ���λУ׼����
	while(ADC_GetResetCalibrationStatus(ADC1));	
	//Enable AD calibration
  //����ADУ׼	
	ADC_StartCalibration(ADC1);
	//Wait for the calibration to finish
  //�ȴ�У׼����	
	while(ADC_GetCalibrationStatus(ADC1));	 
}		
/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
�������ܣ�AD����
��ڲ�����ADC��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	
	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����
	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );
  //Enable the specified ADC1 software transformation startup function	
  //ʹ��ָ����ADC1�����ת����������	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
	//Wait for the conversion to finish
  //�ȴ�ת������	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	//Returns the result of the last ADC1 rule group conversion
	//�������һ��ADC1�������ת�����
	return ADC_GetConversionValue(ADC1);	
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
�������ܣ��ɼ����ADCֵ��ƽ��ֵ����
��ڲ�����ADCͨ���Ͳɼ�����
�� �� ֵ��ADת�����
**************************************************************************/
u16 Get_adc_Average(u8 chn, u8 times)
{
  u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(chn);
		delay_ms(5);
	}
	return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in mV
�������ܣ���ȡ��ص�ѹ 
��ڲ�������
����  ֵ����ص�ѹ����λmv
**************************************************************************/
float Get_battery_volt(void)   
{  
	float Volt;
	
	//The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�	
	Volt=Get_Adc(Battery_Ch)*3.3*11.0*Revise/1.0/4096;	
	return Volt;
}




