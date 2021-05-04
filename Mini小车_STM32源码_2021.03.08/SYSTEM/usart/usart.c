#include "usart.h"	
 
//If using UCOS, include the following header file.
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"	//The header file used by FreeRTOS //FreeRTOSʹ�õ�ͷ�ļ�	
#endif

/*****Add the following code to support printf without having to select use MicroLIB****/
/*****�������´���,֧��printf����,������Ҫѡ��use MicroLIB****/	  
#if 1
#pragma import(__use_no_semihosting)  
//Support functions required by the library
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;  
//Define _sys_exit() to avoid using half-host mode
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//Redefine the fputc function
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0); //Loop to send until it is finished //ѭ������,ֱ���������   
    USART2->DR = (u8) ch;      
	return ch;
}
#endif 
/************************************************************/
 
void usart1_init(u32 bound)
{
  //GPIO port setup //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//Enable USART1, GPIOA clock //ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//Reuse push-pull output //�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure); //Initialize GPIOA9 //��ʼ��GPIOA9
   
  //USART1_RX	  GPIOA10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Float input //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure); //Initialize GPIOA10 //��ʼ��GPIOA10 
	
	//USART Initialization Settings
  //USART��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Serial port baud rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word size is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; //No-no parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Non-hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Receipt mode //�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
  USART_Cmd(USART1, ENABLE); //Enable serial port 1 //ʹ�ܴ���1 
}

#if EN_USART1_RX //If enabled to receive //���ʹ���˽���
// Serial port 1 interrupt service routine
// Note that read USARTx- >;SR can avoid inexplicable errors
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN]; //Receive buffer, maximum USART_REC_LEN bytes //���ջ���,���USART_REC_LEN���ֽ�
//Receive status
//bit15, receive complete flag
//bit14, 0x0d received
//bit13~0, the number of valid bytes received
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0; //Receive status markers //����״̬���	  

#endif	

