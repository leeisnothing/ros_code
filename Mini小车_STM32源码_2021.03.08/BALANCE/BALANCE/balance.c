#include "balance.h"

int Time_count=0; //Time variable //��ʱ����

// Robot mode is wrong to detect flag bits
//������ģʽ�Ƿ�������־λ
int robot_mode_check_flag=0; 

//Self-check the relevant variables //�Լ���ر���
int EncoderA_Count=0, EncoderB_Count=0, EncoderC_Count=0, EncoderD_Count=0;                               //�������������ݼ�������������
int MPU9250ErrorCount, MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;//�������������ݼ�������������

Encoder OriginalEncoder; //Encoder raw data //������ԭʼ����     
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //����Ŀ���ٶ��޷�
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //ȫ���ƶ�С���ſ����ٶ�ƽ������
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //�������ٶȽ���ƽ������
  
      //Get the smoothed data 
			//��ȡƽ������������			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //�����ķ��С��
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//ȫ����С��
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //û��ʹ�õ�
		}
		
		//Ackermann structure car
		//������С��
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //������С��ר����ر���
			int K=1000;
			float Ratio=1, Angle;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//���ڰ�����С��Vz����ǰ��ת��Ƕ�
			Angle=Vz;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
			Angle=target_limit_float(Angle,-0.35f,0.35f);
			
			// The software compensates for the front wheel steering Angle due to mechanical structure limitations
			//��е�ṹ���ƣ������ǰ��ת��ǶȽ��в���
			if(Angle<0)Ratio=1.054;
			else if(Angle>0)Ratio=0.838;
			else Ratio=0;
			
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target   = Vx*(1-Wheel_spacing*tan(Angle)/2/Axle_spacing);
			MOTOR_B.Target   = Vx*(1+Wheel_spacing*tan(Angle)/2/Axle_spacing);
			// The PWM value of the servo controls the steering Angle of the front wheel
			//���PWMֵ���������ǰ��ת��Ƕ�
			Servo=(SERVO_INIT-Angle*K*Ratio); 
			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
			Servo=target_limit_int(Servo,900,2000);	//Servo PWM value limit //���PWMֵ�޷�
		}
		
		//Differential car
		//����С��
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
		}
		
		//FourWheel car
		//������
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
					
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
		}
		
		//Tank Car
		//�Ĵ���
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//��������100Hz��Ƶ�����У�10ms����һ�Σ�
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			
			//Time count is no longer needed after 30 seconds
			//ʱ�������30�������Ҫ
			if(Time_count<3000)Time_count++;
			
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
			Get_Velocity_Form_Encoder();   
			
			if(Check==0) //If self-check mode is not enabled //���û�������Լ�ģʽ
			{
				if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //����APPң������
				else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //����ģң������
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //����PS2�ֱ���������
				
				//CAN, Usart 1, Usart 3 control can directly get the three axis target speed, 
				//without additional processing
				//CAN������1������3(ROS)����ֱ�ӵõ�����Ŀ���ٶȣ�������⴦��
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
				
				//Click the user button to update the gyroscope zero
				//�����û������������������
				Key(); 
				
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0
				//�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ���������ʧ�ܱ�־λΪ0
				if(Turn_Off(Voltage)==0) 
				 { 			
           //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //�ٶȱջ����Ƽ�������PWMֵ��PWM������ʵ��ת��
					 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
						 
					 //Set different PWM control polarity according to different car models
					 //���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ���
					 switch(Car_Mode)
					 {
							case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //�����ķ��С��
							case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //ȫ����С��
							case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //������С��
							case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //���ֲ���С��
							case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //������ 
							case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //�Ĵ���
					 }
				 }
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //���Turn_Off(Voltage)����ֵΪ1�����������С�������˶���PWMֵ����Ϊ0
				 else	Set_Pwm(0,0,0,0,0); 
			 }	
			 //If self-check mode is enabled, the run self-check function is executed
			 //����������Լ�ģʽ����ִ�������Լ캯��
       else CheckTask(); 
		 }  
}

/**************************************************************************
Function: Self - check, check the car itself error, and generate the corresponding error code
Input   : none
Output  : none
�������ܣ��Լ죬���С��������󣬲�������Ӧ�������
��ڲ�������
����  ֵ����
**************************************************************************/
int CheckTask(void)
{  
	 // Control forward, actuallyreverse, 0001.Inversion of control, actual forward, 0010.
   // Control forward, actuallystop, 0100.Inversion of Control, Actual Stop, 1000.
	 //Here we define PWM to be positive
	 //������ת��ʵ�ʷ�ת��0001�����Ʒ�ת��ʵ����ת��0010��
	 //������ת��ʵ��ͣת��0100�����Ʒ�ת��ʵ��ͣת��1000�����ﶨ��PWM����ֵΪ��ת
	 static int A,B,C,D; 

	 static float MaxVoltage=0, MinVoltage=20, Temperature, LastTemperature=3000, TemperatureBias; 
	 static int WireWrong=0; //Whether the motor wire is connected to the wrong marker //������Ƿ�Ӵ��־λ
	 
	 if(Check)CheckCount++;  //Countdown to self-check confirmation //�Լ�ȷ�ϵ���ʱ
	 else CheckCount=0;
	
	 //Confirm success, start self-test //ȷ�ϳɹ�����ʼ�Լ�
	 if(Check&&Checking) 
	 {
		 //Test for 2 seconds at each stage, wait for 1 second before the movement becomes stable
		 //ÿ���׶β�2�룬�ȴ�1���˶��ȶ���ʼ���
		 int CheckPeriod=200, WaitPeriod=100; 
		 
		 // Voltage fluctuation detection
		 //��ѹ�������
		 if(Voltage>MaxVoltage) MaxVoltage=Voltage;
		 if(Voltage<MinVoltage) MinVoltage=Voltage;
		 
		 //MPU9250 detection
		 //MPU9250���
		 if(CheckCount<=(1+CheckPeriod*CheckPhrase2))
		 {
			 if(gyro[0]==0||gyro[1]==0||gyro[2]==0||accel[0]==0||accel[1]==0||accel[2]==0||MPU_Get_Temperature()==0)MPU9250ErrorCount++;
		 }
		 
		 //Temperature detection
		 //�¶ȼ��
		 TemperatureBias=MPU_Get_Temperature()-LastTemperature;
		 if(TemperatureBias<10&&TemperatureBias>-10)
		 {
			 Temperature=(MPU_Get_Temperature()+LastTemperature)/2;
			 LastTemperature=Temperature;
		 }
		 
		 //Open loop control motor test //�������Ƶ�����
		 if(0<CheckCount&&CheckCount<(CheckPeriod)) //Test motor A is running //����A�����ת 
		 {
			 if(CheckCount==1)Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(2000, 0, 0, 0, 1500);
			 if(CheckCount>(0+CheckPeriod-WaitPeriod))
			 {
				 static int ZeroACount=0;
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.A<-3)A=A|1;    //Motor reversal //�����ת		
         if(ZeroACount>90)			 A=A|1<<2; //The motor stops //���ͣת
			 }			 
		 }
		 else if(CheckPeriod<CheckCount&&CheckCount<(CheckPeriod*(2))) //Test A motor reversal //����A�����ת
		 {
			 if(CheckCount==(1+CheckPeriod))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(-2000, 0, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*2-WaitPeriod))
			 {
				 static int ZeroACount=0;
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.A>3)A=A|1<<1; //The motor is running //�����ת 	
         if(ZeroACount>90)			A=A|1<<3; //The motor stops //���ͣת				 
			 }		 
		 }
		 
		 else if(CheckPeriod*(2)<CheckCount&&CheckCount<(CheckPeriod*(3))) //Test motor B is running //����B�����ת
		 {
			 if(CheckCount==(1+CheckPeriod*2))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 2000, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*3-WaitPeriod))
			 {
				 static int ZeroBCount=0;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.B<-3)B=B|1;    //Motor reversal //�����ת 
				 if(ZeroBCount>90)			 B=B|1<<2; //The motor stops //���ͣת
			 }			 
		 }
		 else if(CheckPeriod*(3)<CheckCount&&CheckCount<(CheckPeriod*(4))) //Test B motor reversal //����B�����ת
		 {
			 if(CheckCount==(1+CheckPeriod*3))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, -2000, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*4-WaitPeriod))
			 {
				 static int ZeroBCount=0;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.B>3)B=B|1<<1; //The motor is running //�����ת
				 if(ZeroBCount>90)			B=B|1<<3; //The motor stops //���ͣת
			 }			 
		 }
		 
		 else if(CheckPeriod*(4)<CheckCount&&CheckCount<(CheckPeriod*5)&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*4))Set_Pwm(0,0,0,0,1500);    
			 Set_Pwm(0, 0, 2000, 0, 1500);//Test motor C is running //����C�����ת
			 if(CheckCount>(CheckPeriod*5-WaitPeriod))
			 {
				 static int ZeroCCount=0;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.C<-3)C=C|1;    //Motor reversal //�����ת	
         if(ZeroCCount>90)			 C=C|1<<2; //The motor stops //���ͣת				 
			 }				 
		 }
		 else if(CheckPeriod*(5)<CheckCount&&CheckCount<(CheckPeriod*6)&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*5))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, -2000, 0, 1500); //Test C motor reversal //����C�����ת
			 if(CheckCount>(CheckPeriod*6-WaitPeriod))
			 {
				 static int ZeroCCount=0;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.C>3)C=C|1<<1; //The motor is running //�����ת
				 if(ZeroCCount>90)			C=C|1<<3; //The motor stops //���ͣת	
			 }				 
		 }
		 
		 else if(CheckPeriod*(6)<CheckCount&&CheckCount<(CheckPeriod*7)&&(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*6))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, 0, 2000, 1500);//Test motor D is running //����D�����ת
			 if(CheckCount>(CheckPeriod*7-WaitPeriod))
			 {
				 static int ZeroDCount=0;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 if(OriginalEncoder.D<-3)D=D|1;    //Motor reversal //�����ת		
         if(ZeroDCount>90)			 D=D|1<<2; //The motor stops //���ͣת					 
			 }	 
		 }
		 else if(CheckPeriod*(7)<CheckCount&&CheckCount<(CheckPeriod*8)&&(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*7))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, 0, -2000, 1500); //Test D motor reversal //����D�����ת
			 if(CheckCount>(CheckPeriod*8-WaitPeriod))
			 {
				 static int ZeroDCount=0;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 if(OriginalEncoder.D>3)D=D|1<<1; //The motor is running //�����ת
				 if(ZeroDCount>90)			D=D|1<<3; //The motor stops //���ͣת	
			 }				 
		 }
		 //Open loop control motor test //�������Ƶ�����		 
		 
		 //Check the encoder A , four_wheel car will check the encoder AB at the same time
		 //��������A�������������ͬʱ���AB
		 else if(CheckPeriod*(CheckPhrase1)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+1))) 
		 {			 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=-0.3, MOTOR_B.Target=0, MOTOR_C.Target=0, MOTOR_D.Target=0;
			 if(Car_Mode==FourWheel_Car)MOTOR_B.Target=-0.3;
			 
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   
			 if(Car_Mode==FourWheel_Car)
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
			 else 
			 {
				 MOTOR_B.Motor_Pwm=0;   
				 MOTOR_C.Motor_Pwm=0;   
				 MOTOR_D.Motor_Pwm=0;   
			 }
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 1500 ); break;
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break;
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break;
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break;
			 }
		 
			 //Low speed statistics for 9250Z axis (high speed means the car is not being driven), low speed statistics for encoder speed.
			 //9250Z���ٶ�ֵ�ʹ���ͳ��(�������ߴ���û�б�����)���������ٶ�ֵ���ٶ�ͳ�ơ�
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+1)-WaitPeriod))
			 {
				 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountA++;		
				 if(MOTOR_A.Encoder<0.02&&MOTOR_A.Encoder>-0.02)EncoderA_Count++;		
         if(Car_Mode==FourWheel_Car)		
				 {
					 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountB++;		
				   if(MOTOR_B.Encoder<0.02&&MOTOR_B.Encoder>-0.02)EncoderB_Count++;	
				 }					 
			 }				 
		 }
		 
		 //Check the encoder B. four_wheel car wil check CD at the same time, do not check B
		 //��������B��������ͬʱ���CD,�����B
		 else if(CheckPeriod*(CheckPhrase1+1)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+2))) 
		 {					 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1+1)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=0, MOTOR_B.Target=0.3, MOTOR_C.Target=0, MOTOR_D.Target=0;
			 if(Car_Mode==FourWheel_Car)MOTOR_A.Target=0, MOTOR_B.Target=0, MOTOR_C.Target=0.3, MOTOR_D.Target=0.3;
		 		 
			 if(Car_Mode==FourWheel_Car)
			 {
				 MOTOR_A.Motor_Pwm=0;   
				 MOTOR_B.Motor_Pwm=0;  
				 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target),   
				 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);  
			 }
			 else
			 {
				 MOTOR_A.Motor_Pwm=0;   
				 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   
				 MOTOR_C.Motor_Pwm=0;
				 MOTOR_D.Motor_Pwm=0;
			 }
			 
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break;
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 1500 ); break;
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break;
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break;
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break;
			 }
		 
			 //Low speed statistics for 9250Z axis (high speed means the car is not being driven), low speed statistics for encoder speed.
			 //9250Z���ٶ�ֵ�ʹ���ͳ��(�������ߴ���û�б�����)���������ٶ�ֵ���ٶ�ͳ�ơ�
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+2)-WaitPeriod))
			 {
				 if(Car_Mode==FourWheel_Car) 		
				 {
					 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountC++;		
				   if(MOTOR_C.Encoder<0.02&&MOTOR_C.Encoder>-0.02)EncoderC_Count++;	
					 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountD++;		
				   if(MOTOR_D.Encoder<0.02&&MOTOR_D.Encoder>-0.02)EncoderD_Count++;	
				 }
				 else
				 {
					 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountB++;		
				   if(MOTOR_B.Encoder<0.02&&MOTOR_B.Encoder>-0.02)EncoderB_Count++;	
				 }			 			 
			 }				 
		 }
		 
		 //Check the encoder C //��������C
		 else if(CheckPeriod*(CheckPhrase1+2)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+3))&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car)) 
		 {				 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1+2)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=0, MOTOR_B.Target=0, MOTOR_C.Target=-0.3, MOTOR_D.Target=0;
       		 
			 MOTOR_A.Motor_Pwm=0;   
			 MOTOR_B.Motor_Pwm=0;  
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   
			 MOTOR_D.Motor_Pwm=0;   
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; 
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
			 }
			 
			 //Low speed statistics for 9250Z axis (high speed means the car is not being driven), low speed statistics for encoder speed.
			 //9250Z���ٶ�ֵ�ʹ���ͳ��(�������ߴ���û�б�����)���������ٶ�ֵ���ٶ�ͳ�ơ�
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+3)-WaitPeriod))
			 {
				 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountC++;		
				 if(MOTOR_C.Encoder<0.02&&MOTOR_C.Encoder>-0.02)EncoderC_Count++;					 
			 }				 
		 }
		 
		 // Check the encoder D
		 //��������D
		 else if(CheckPeriod*(CheckPhrase1+3)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+4))&&(Car_Mode==Mec_Car)) 
		 {		 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1+3)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=0, MOTOR_B.Target=0, MOTOR_C.Target=0, MOTOR_D.Target=0.3;
			 
			 MOTOR_A.Motor_Pwm=0;  
			 MOTOR_B.Motor_Pwm=0; 
			 MOTOR_C.Motor_Pwm=0; 
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; 
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break;
			 }
			 
			 //Low speed statistics for 9250Z axis (high speed means the car is not being driven), low speed statistics for encoder speed.
			 //9250Z���ٶ�ֵ�ʹ���ͳ��(�������ߴ���û�б�����)���������ٶ�ֵ���ٶ�ͳ�ơ�
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+4)-WaitPeriod))
			 {
				 //The wheat-wheel D motor structure is weak in driving the car rotation, so the test is tolerant
				 //����D����ṹ�ϴ���С��ת�����������������ݴ���		
				 if(gyro[2]<400&&gyro[2]>-400)MPU9250SensorCountD++;	
				 if(MOTOR_D.Encoder<0.02&&MOTOR_D.Encoder>-0.02)EncoderD_Count++;				 
			 }				 
		 }
		 
		 //Check whether the motor wire is connected wrong //��������Ƿ�Ӵ� 
		 else if((CheckPeriod*(CheckPhrase2-2))<CheckCount && CheckCount<(CheckPeriod*(CheckPhrase2-1))&&Car_Mode==Mec_Car) //Test the motor line twice //���ֳ��������ε����
		 {			 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase2-2)))Set_Pwm(0,0,0,0,1500);
			 else if(Car_Mode==Mec_Car)
			 {
				 Drive_Motor(0, 0.3, 0);
			 }
			 			 
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);  
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);  
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);  
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target); 
			 Limit_Pwm(2000);

			 Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    );  
		 
			 if(CheckCount>(CheckPeriod*(CheckPhrase2-1)-WaitPeriod))
			 {
				 static int ZeroACount=0, ZeroBCount=0, ZeroCCount=0, ZeroDCount=0;
				 static int WireWrongCount=0;
				 
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 
				 if(gyro[2]>64000||gyro[2]<-64000)gyro[2]=501;

				 if((gyro[2]>500||gyro[2]<-500)) WireWrongCount++;
				 if(WireWrongCount>80)           WireWrong=1;//ErrorCode=ErrorCode|1<<15; //Motor wire connection is wrong, error code input //����߽Ӵ��������¼��	 
			 }				 
		 }
		 else if((CheckPeriod*(CheckPhrase2-1))<CheckCount && CheckCount<(CheckPeriod*(CheckPhrase2))) 
		 {			 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase2-1)))Set_Pwm(0,0,0,0,1500);
			 if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			 {
				 Drive_Motor(0.15, 0, 0.5);
			 }
			 else if(Car_Mode==FourWheel_Car)
			 {
				 Drive_Motor(0.3,   0, PI/2);
			 }
			 else if(Car_Mode==Omni_Car)
			 {
				 Drive_Motor(0.3,   0, PI/2);
			 }
			 else if(Car_Mode==Mec_Car)
			 {
				 Drive_Motor(0.3,   0, 0);
			 }
			 			 
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target); 
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target); 
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target); 
			 Limit_Pwm(2000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; 
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; 
			 }
		 
			 if(CheckCount>(CheckPeriod*(CheckPhrase2)-WaitPeriod))
			 {
				 static int ZeroACount=0, ZeroBCount=0, ZeroCCount=0, ZeroDCount=0;
				 static int WireWrongCount=0;
				 
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 
				 if(CheckCount==(CheckPeriod*(CheckPhrase2)-2))
				 {
					 //There is no encoder data when the driver is driven alone, 
					 //but there is data when the encoder is moving as a whole, indicating that the driver is damaged
					 //��������ʱ�ޱ��������ݣ������˶�ʱ�����������ݣ�˵��������
					 if((A&12)&&ZeroACount<30)ErrorCode=ErrorCode|1<<3; 
					 if((B&12)&&ZeroBCount<30)ErrorCode=ErrorCode|1<<5;
					 if((C&12)&&ZeroCCount<30)ErrorCode=ErrorCode|1<<7;
					 if((D&12)&&ZeroDCount<30)ErrorCode=ErrorCode|1<<9;
				 }
				 
				 //If the driver and encoder are not normal, it is confirmed that the motor wire connection is wrong
				 //��������������������ȷ�ϵ���߽Ӵ�
				 if(gyro[2]>64000||gyro[2]<-64000)gyro[2]=501;
				 if(Car_Mode==Mec_Car)
				 {
				   if((gyro[2]>500||gyro[2]<-500))   WireWrongCount++;
					 if(WireWrongCount>80)             WireWrong=1;//ErrorCode=ErrorCode|1<<15; //Motor wire connection is wrong, error code input //����߽Ӵ��������¼��
				 }
				 else
				 {
					 if((gyro[2]<100))                 WireWrong=1;//ErrorCode=ErrorCode|1<<14; //Motor wire connection is wrong, error code input //����߽Ӵ��������¼��
				 }			 
			 }				 
		 }
		 //Check whether the motor wire is connected wrong //��������Ƿ�Ӵ�

		 //statistics error code //ͳ�ƴ������
		 else if(CheckCount==(1+CheckPeriod*(CheckPhrase2)))
		 {			 		 
			 if(MPU9250ErrorCount>100*CheckPhrase2/2)     ErrorCode=ErrorCode|1<<13; //MPU9250 damage //MPU9250��
			 if(Temperature>7000)                         ErrorCode=ErrorCode|1<<12; //The temperature is too high //�¶ȹ���
			 if((MaxVoltage-MinVoltage)>5)                ErrorCode=ErrorCode|1<<11; //The voltage fluctuation is too large, the power supply is not stable //��ѹ�������󣬵�Դ���ȶ�
			 if(Voltage<10)                               ErrorCode=ErrorCode|1;     //The voltage is too low //��ѹ����
			 if(EN==0)                                    ErrorCode=ErrorCode|1<<1;  // The switch is off //���ش��ڹر�״̬
			 if(A==16&&B==16)                             ErrorCode=ErrorCode|1<<2;  //The car model is wrong, and the potentiometer in the upper right corner is adjusted //С���ͺ�ѡ�����Ͻǵ�λ�����ͺ�
			 
			 switch(A&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<3; break; //The drive is damaged //������   0001
				case 2:                 ErrorCode=ErrorCode|1<<3; break; //The drive is damaged //������   0010
				case 3:                 ErrorCode=ErrorCode|1<<4; break; //Reverse connection of the drive wire //�����߷��� 0011											
				default:                                          break;
			 }
			 switch(B&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<5; break; //The drive is damaged //������   0001
				case 2:                 ErrorCode=ErrorCode|1<<5; break; //The drive is damaged //������   0010
				case 3:                 ErrorCode=ErrorCode|1<<6; break; //Reverse connection of the drive wire //�����߷��� 0011											
				default:                                          break;
			 }
			 switch(C&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<7; break; //The drive is damaged //������   0001
				case 2:                 ErrorCode=ErrorCode|1<<7; break; //The drive is damaged //������   0010
				case 3:                 ErrorCode=ErrorCode|1<<8; break; //Reverse connection of the drive wire //�����߷��� 0011											
				default:                                          break;
			 }
			 switch(D&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<9; break; //The drive is damaged //������   0001
				case 2:                 ErrorCode=ErrorCode|1<<9; break; //The drive is damaged //������   0010
				case 3:                 ErrorCode=ErrorCode|1<<10;break; //Reverse connection of the drive wire //�����߷��� 0011											
				default:                                          break;
			 }
			 
			 //Four_wheel car driving a single wheel does not move the car, resulting in gyroscope data
			 //�������������������޷�ʹ�����ƶ�������������������
			 if(MPU9250SensorCountA>=80){if(EncoderA_Count>80)                          ErrorCode=ErrorCode|1<<3; } //Drive A is broken   //����A��
			 else                       {if(EncoderA_Count>80&&(!(ErrorCode&1<<3)))     ErrorCode=ErrorCode|1<<16;} //Encoder A is broken //������A�� 
			 if(MPU9250SensorCountB>=80){if(EncoderB_Count>80)                          ErrorCode=ErrorCode|1<<5; } //Drive B is broken   //����B��
			 else                       {if(EncoderB_Count>80&&(!(ErrorCode&1<<5)))     ErrorCode=ErrorCode|1<<17;} //Encoder B is broken //������B�� 
			 if(MPU9250SensorCountC>=80){if(EncoderC_Count>80)                          ErrorCode=ErrorCode|1<<7; } //Drive B is broken   //����C��
			 else                       {if(EncoderC_Count>80&&(!(ErrorCode&1<<7)))     ErrorCode=ErrorCode|1<<18;} //Encoder C is broken //������C�� 
			 if(MPU9250SensorCountD>=90){if(EncoderD_Count>80)                          ErrorCode=ErrorCode|1<<9; } //Drive B is broken   //����D��
			 else                       {if(EncoderD_Count>80&&(!(ErrorCode&1<<9)))     ErrorCode=ErrorCode|1<<19;} //Encoder D is broken //������D�� 
			 //                          1111 0000 0111 1111 1000
			 if(WireWrong==1&&(ErrorCode&0xf07f8)==0)ErrorCode=ErrorCode|1<<14;
			 OLED_Clear();
			 Checked=1;
	   }
		 //Statistics error code //ͳ�ƴ������
			
		 //Test completed, motor set to 0 //�����ɣ������0
		 if(CheckCount>=(1+CheckPeriod*(CheckPhrase2)))
		 {
		  Set_Pwm(0,0,0,0,1500);
		  MOTOR_A.Target=0,   MOTOR_B.Target=0,   MOTOR_C.Target=0,   MOTOR_D.Target=0;
		  MOTOR_A.Motor_Pwm=0,MOTOR_B.Motor_Pwm=0,MOTOR_C.Motor_Pwm=0,MOTOR_D.Motor_Pwm=0;
		  MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   
		  MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);  
		  MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target); 
		  MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target); 
		 }
	 }
	 return 0;
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//�������ת����
  if(motor_a<0)		AIN2=0,		AIN1=1;
	else				    AIN2=1,		AIN1=0;
	//Motor speed control 
	//���ת�ٿ���
	PWMA=abs(motor_a);

	//Forward and reverse control of motor
	//�������ת����	
	if(motor_b<0)		BIN2=1,		BIN1=0;
	else 	          BIN2=0,		BIN1=1;
	//Motor speed control 
	//���ת�ٿ���
	PWMB=abs(motor_b);

	//Forward and reverse control of motor
	//�������ת����	
	if(motor_c>0)		CIN2=0,		CIN1=1;
	else 	          CIN2=1,		CIN1=0;
	//Motor speed control 
	//���ת�ٿ���
  PWMC=abs(motor_c);

	//Forward and reverse control of motor
	//�������ת����
	if(motor_d>0)		DIN2=0,		DIN1=1;
	else 	          DIN2=1,		DIN1=0;
	//Motor speed control 
	//���ת�ٿ���
	PWMD=abs(motor_d);
	
	//Servo control
	//�������
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬�����ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ�������ƣ�1��������0����
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA=0;
				PWMB=0;		
				PWMC=0;	
				PWMD=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //ȫ�����˶�С�����Խ��к����ƶ�
	{
	 switch(Flag_Direction)  //Handle direction control commands //�������������
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //����޷������ָ����ת�����״̬
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //����ת  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //����ת
		 else 		               Move_Z=0;                       //stop           //ֹͣ
	 }
	}	
	else //Non-omnidirectional moving trolley //��ȫ���ƶ�С��
	{
	 switch(Flag_Direction) //Handle direction control commands //�������������
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //����ת 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //����ת	
	}
	
	//Z-axis data conversion //Z������ת��
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//�������ṹС��ת��Ϊǰ��ת��Ƕ�
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //��λת����mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//�õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //����ҡ��С���ȶ���
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//����
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //����	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //��PS2�ֱ�����������д���
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z������ת��
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //�������ṹС��ת��Ϊǰ��ת��Ƕ�
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //�õ�����Ŀ��ֵ�������˶�ѧ����
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���

	  //limiter //�޷�
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //��ҡ��ǰ���򡣿���ǰ�����ˡ�
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //��ҡ�����ҷ��򡣿��������ƶ�������ȫ���ֲŻ�ʹ�õ���ͨ����������С��ʹ�ø�ͨ����ΪPWM������ƶ��
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//��ҡ��ǰ��������/�Ӽ��١�
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//��ҡ�����ҷ��򡣿�����ת��
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //�������
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//�Ժ�ģң�ؿ���������д���
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z������ת��
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //�������ṹС��ת��Ϊǰ��ת��Ƕ�
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		
	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //�õ�����Ŀ��ֵ�������˶�ѧ����
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
�������ܣ������û������������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double_MPU6050(50); 
	if(tmp==2)memcpy(Deviation_gyro,Original_gyro,sizeof(gyro));
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //��ȡ��������ԭʼ����
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	  //Decide the encoder numerical polarity according to different car models
		//���ݲ�ͬС���ͺž�����������ֵ����
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
�������ܣ���ֹ��λ��ѡ��ģʽ�����³�ʼ���������������ת����ֹͣʹ��
��ڲ�������
����  ֵ����
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//�������6�νӽ�����������ж�Ϊ�����ת���õ��ʧ��	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}
