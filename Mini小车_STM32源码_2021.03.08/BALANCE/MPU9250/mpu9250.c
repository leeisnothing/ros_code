#include "mpu9250.h"
#include "sys.h"
#include "delay.h"  

//Zero drift count
//���Ư�Ƽ���
int Deviation_Count;
//Triaxial IMU data
//����IMU����
short gyro[3], accel[3],magnet[3];
// Gyro static error, raw data
//�����Ǿ��ԭʼ����
short Deviation_gyro[3],Original_gyro[3];    

/**************************************************************************
Function: MPU9250 task
Input   : none
Output  : none
�������ܣ�MPU9250����
��ڲ�������
����  ֵ����
**************************************************************************/
void MPU9250_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
			//This task runs at 100Hz
			//��������100Hz��Ƶ������
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));	
			//Read the gyroscope zero before starting
      //����ǰ����ȡ���������			
		  if(Deviation_Count<CONTROL_DELAY)
		  {	
		  	Deviation_Count++;
			  memcpy(Deviation_gyro,gyro,sizeof(gyro));		
		  }		
			//Get acceleration sensor data //�õ����ٶȴ���������
			MPU_Get_Accelerometer(accel);	
			//Get gyroscope data           //�õ�����������
			MPU_Get_Gyroscope(gyro);
      //Get magnetometer data	       //�õ�����������	
			MPU_Get_Magnetometer(magnet);		  
    }
}  

/**************************************************************************
Function: Initialize MPU9250
Input   : none
Output  : 0: success, others: error code (Errors are usually caused by device addresses, or by misaligned IIC pins)
�������ܣ���ʼ��MPU9250
��ڲ�������
����  ֵ��0:�ɹ�, ����:������� (����һ�㶼��������ַ����ģ�����IIC���Ų��Ե�)
**************************************************************************/
u8 MPU9250_Init(void)
{
  u8 res=0;
  //IIC_Init();  //Initialize the IIC bus //��ʼ��IIC����
  MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80); //Reset MPU9250 //��λMPU9250
  delay_ms(100); //Delay 100 ms //��ʱ100ms
  MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00); //Wake mpu9250 //����MPU9250
	
  MPU_Set_Gyro_Fsr(1);  //Gyroscope sensor              //�����Ǵ�����,��500dps=��500��/s ��32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
	MPU_Set_Accel_Fsr(0);	//Acceleration sensor           //���ٶȴ�����,��2g=��2*9.8m/s^2 ��32768 accel/32768*19.6=accel/1671.84
  MPU_Set_Rate(50);			//Set the sampling rate to 50Hz //���ò�����50Hz

  MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);    //Turn off all interrupts //�ر������ж�
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00); //The I2C main mode is off //I2C��ģʽ�ر�
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	 //Close the FIFO //�ر�FIFO
	//The INT pin is low, enabling bypass mode to read the magnetometer directly
	//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82); 
	//Read the ID of MPU9250 
	//��ȡMPU9250��ID
  res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);   
  if(res==MPU6500_ID) //The device ID is correct, The correct device ID depends on the AD pin //����ID��ȷ, ����ID����ȷȡ����AD����
    {
       MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01); //Set CLKSEL,PLL X axis as reference //����CLKSEL,PLL X��Ϊ�ο�
       MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00); //Acceleration and gyroscope both work //���ٶ��������Ƕ�����
		   MPU_Set_Rate(50);						       	                //Set the sampling rate to 50Hz //���ò�����Ϊ50Hz   
    }
	else return 1;
 
  res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA); //Read AK8963ID //��ȡAK8963ID   
  if(res==AK8963_ID)
    {
       MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //Set AK8963 to single measurement mode //����AK8963Ϊ���β���ģʽ
    }
	else return 1;

  return 0;
}
/**************************************************************************
Function: Set the full range of the MPU9250 gyroscope sensor
Input   : fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����MPU9250�����Ǵ����������̷�Χ
��ڲ�����fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
����  ֵ��0:���óɹ�, ����:����ʧ��
**************************************************************************/
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3); //Set the full scope of the gyroscope//���������������̷�Χ  
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : fsr:0,��2g;1,��4g;2,��8g;3,��16g
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����MPU9250���ٶȴ����������̷�Χ
��ڲ�����fsr:0,��2g;1,��4g;2,��8g;3,��16g
����  ֵ��0:���óɹ�, ����:����ʧ��
**************************************************************************/
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3); //Set the full range of the acceleration sensor//���ü��ٶȴ����������̷�Χ  
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : LPF: Digital low-pass filtering frequency (Hz)
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����MPU9250�����ֵ�ͨ�˲���
��ڲ�����lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
����  ֵ��0:���óɹ�, ����:����ʧ��
**************************************************************************/
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data); //Set the digital lowpass filter//�������ֵ�ͨ�˲���  
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : rate:4~1000(Hz)
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
��ڲ�����rate:4~1000(Hz)
����  ֵ��0:���óɹ�, ����:����ʧ��
**************************************************************************/
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//Set the digital lowpass filter//�������ֵ�ͨ�˲���  
 	return MPU_Set_LPF(rate/2);	//Automatically sets LPF to half of the sampling rate //�Զ�����LPFΪ�����ʵ�һ��
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : none
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����¶�ֵ
��ڲ�������
����  ֵ���¶�ֵ(������100��)
**************************************************************************/
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	  float temp;
	  MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
�������ܣ����������ֵ(ԭʼֵ)
��ڲ�����gx,gy,gz:������x,y,z���ԭʼ����(������)
����  ֵ��0:�ɹ�, ����:�������
**************************************************************************/
u8 MPU_Get_Gyroscope(short *gyro)
{
  u8 buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		if(Deviation_Count<CONTROL_DELAY) // 10 seconds before starting //����ǰ10��
		{
			//Read the gyroscope zero //��ȡ���������
			gyro[0] =(((u16)buf[0]<<8)|buf[1]);  
			gyro[1] =(((u16)buf[2]<<8)|buf[3]);  
			gyro[2]= (((u16)buf[4]<<8)|buf[5]);
			Led_Count=1; //LED high frequency flashing //LED��Ƶ��˸
			Flag_Stop=1; //The software fails to flag location 1 //���ʧ�ܱ�־λ��1		
		}
		else //10 seconds after starting //����10���
		{  
			if(Deviation_Count==CONTROL_DELAY)
				Flag_Stop=0; //The software fails to flag location 0 //���ʧ�ܱ�־λ��0
			Led_Count=300; //The LED returns to normal flicker frequency //LED�ָ�������˸Ƶ��	
			
			//Save the raw data to update zero by clicking the user button
			//����ԭʼ�������ڵ����û������������
			Original_gyro[0] =(((u16)buf[0]<<8)|buf[1]);  
			Original_gyro[1] =(((u16)buf[2]<<8)|buf[3]);  
			Original_gyro[2]= (((u16)buf[4]<<8)|buf[5]);			
			
			//Removes zero drift data
			//ȥ�����Ư�Ƶ�����
			gyro[0] =Original_gyro[0]-Deviation_gyro[0];  
			gyro[1] =Original_gyro[1]-Deviation_gyro[1];  
			gyro[2]= Original_gyro[2]-Deviation_gyro[2];
		}
	} 	
  return res;;
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Ax, Ay, Az: The original reading of the accelerometer on the x,y, and z axes (plus or minus)
Output  : 0: success, others: error code
�������ܣ��õ����ٶ�ֵ(ԭʼֵ)
��ڲ�����ax,ay,az:���ٶȼ�x,y,z���ԭʼ����(������)
����  ֵ��0:�ɹ�, ����:�������
**************************************************************************/
u8 MPU_Get_Accelerometer(short *accel)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
{
		accel[0] =((u16)buf[0]<<8)|buf[1];  
		accel[1] =((u16)buf[2]<<8)|buf[3];  
		accel[2] =((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Mx,my,mz: raw reading (plus or minus) of the x,y, and z axes of the magnetometer
Output  : 0: success, others: error code
�������ܣ��õ�������ֵ(ԭʼֵ)
��ڲ�����mx,my,mz:������x,y,z���ԭʼ����(������)
����  ֵ��0:�ɹ�, ����:�������
**************************************************************************/
u8 MPU_Get_Magnetometer(short *magnet)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		magnet[0]=((u16)buf[1]<<8)|buf[0];  
		magnet[1]=((u16)buf[3]<<8)|buf[2];  
		magnet[2]=((u16)buf[5]<<8)|buf[4];
	} 	
	
	//The AK8963 needs to be reset to single measurement mode after each reading
	//AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
  MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);  
  return res;;
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Addr: device address, reg: register address, len: write length, buf: data area
Output  : 0: success, others: error code
�������ܣ�IIC����д
��ڲ�����addr:������ַ, reg:�Ĵ�����ַ, len:д�볤��, buf:������
����  ֵ��0:�ɹ�, ����:�������
**************************************************************************/
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //Send the device address + write the command //����������ַ+д����
    if(IIC_Wait_Ack())          //wait for an answer //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //Write the register address //д�Ĵ�����ַ
    IIC_Wait_Ack();             //wait for an answer //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //Send data //��������
        if(IIC_Wait_Ack())      //wait for an ACK //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Addr: device address, reg: register address to read, len: length to read, buf: data storage area to read
Output  : 0: success, others: error code
�������ܣ�IIC������
��ڲ�����addr:������ַ, reg:Ҫ��ȡ�ļĴ�����ַ, len:Ҫ��ȡ�ĳ���, buf:��ȡ�������ݴ洢��
����  ֵ��0:�ɹ�, ����:�������
**************************************************************************/
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //Send the device address + write the command //����������ַ+д����
    if(IIC_Wait_Ack())          //wait for an answer //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //Write the register address //д�Ĵ�����ַ
    IIC_Wait_Ack();             //wait for an answer //�ȴ�Ӧ��
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //Send the device address + write the command //����������ַ+д����
    IIC_Wait_Ack();             //wait for an answer //�ȴ�Ӧ��
    while(len)
    {
			if(len==1)*buf=IIC_Read_Byte(0); //Read data and send nACK //������,����nACK 
			else *buf=IIC_Read_Byte(1);	   	 //Read the data and send the ACK //������,����ACK  
			len--;
			buf++;  
    }
    IIC_Stop(); //Generates a stop condition //����һ��ֹͣ����
    return 0;       
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Devaddr: device IIC address, reg: register address, data
Output  : 0: success, others: error code
�������ܣ�IICдһ���ֽ� 
��ڲ�����devaddr:����IIC��ַ, reg:�Ĵ�����ַ, data:����
����  ֵ��0:�ɹ�, ����:�������
**************************************************************************/
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //Send the device address + write the command //����������ַ+д����
    if(IIC_Wait_Ack())          //wait for an answer //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //Write the register address //д�Ĵ�����ַ
    IIC_Wait_Ack();             //wait for an answer //�ȴ�Ӧ��
    IIC_Send_Byte(data);        //Send data //��������
    if(IIC_Wait_Ack())          //wait for an ACK //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Devaddr: device IIC address, reg: register address, data
Output  : The data read
�������ܣ�IIC��һ���ֽ�  
��ڲ�����devaddr:����IIC��ַ, reg:�Ĵ�����ַ, data:����
����  ֵ������������
**************************************************************************/
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //Send the device address + write the command //����������ַ+д����
    IIC_Wait_Ack();             //wait for an answer //�ȴ�Ӧ��
    IIC_Send_Byte(reg);         //Write the register address //д�Ĵ�����ַ
    IIC_Wait_Ack();             //wait for an answer //�ȴ�Ӧ��
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //Send device address + read command //����������ַ+������
    IIC_Wait_Ack();             //wait for an answer //�ȴ�Ӧ��
    res=IIC_Read_Byte(0);		    //Read data and send nACK //������,����nACK  
    IIC_Stop();                 //Generates a stop condition //����һ��ֹͣ����
    return res;  
}

