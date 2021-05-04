#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#include "system.h"
#define Battery_Ch    14 //Battery voltage, ADC channel 14 //��ص�ѹ��ADCͨ��14
#define Potentiometer 8  //Potentiometer, ADC channel 8 //��λ����ADCͨ��8
void Adc_Init(void);
u16 Get_Adc(u8 ch);
float Get_battery_volt(void) ;
u16 Get_adc_Average(u8 chn, u8 times);
extern float Voltage,Voltage_Count,Voltage_All; 	
#endif 


