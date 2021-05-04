#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN 200 //The maximum number of bytes received is 200 //�����������ֽ��� 200
#define EN_USART1_RX 	1		//Enable (1)/Disable (0) Serial port 1 reception //ʹ�ܣ�1��/��ֹ��0������1����
void usart1_init(u32 bound);	  
extern u8  USART_RX_BUF[USART_REC_LEN]; //Receive buffer, maximum USART_REC_LEN bytes.The last byte is a newline character //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;  //Receive status markers //����״̬���	
//If you want serial port interrupt reception, do not comment the following macro definitions
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
#endif


