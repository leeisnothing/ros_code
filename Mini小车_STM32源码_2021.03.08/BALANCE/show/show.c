#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;
extern int Time_count;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
函数功能：读取电池电压、蜂鸣器报警、开启自检、向APP发送数据、OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
int Buzzer_count=25;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //此任务以10Hz的频率运行
		
		//开机时蜂鸣器短暂蜂鸣，开机提醒
		//The buzzer will beep briefly when the machine is switched on
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing //蜂鸣器蜂鸣
		else if(Buzzer_count==5)Buzzer=0;
		
		//Read the battery voltage //读取电池电压
		for(i=0;i<100;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/100;
		Voltage_All=0;
		
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if(Voltage>=12.6)Voltage=12.6;
		else if(10<=Voltage && Voltage<10.5 && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery //10.5V，低电量时蜂鸣器第一次报警
		else if(Voltage<10 && LowVoltage_1<2)LowVoltage_2++; //10V, when the car is not allowed to control, the buzzer will alarm the second time //10V，小车禁止控制时蜂鸣器第二次报警
			
		//Self-check related //自检相关
		if(Deviation_Count>=CONTROL_DELAY) //Start the self-check mode after 10 seconds //开机10秒后才允许开启自检模式				
		{
			if(Long_Press()==1) //Long press the user button to start the self-check mode //长按用户按键开启自检模式
			{
				OLED_Clear();
				Check=!Check;		
				if(Check==1)Flag_Stop=1;
				else Flag_Stop=0;					
				Checking=!Checking;
				Checked=0;
				CheckCount=0;
				ErrorCode=0;
				EncoderA_Count=0, EncoderB_Count=0, EncoderC_Count=0, EncoderD_Count=0;
				MPU9250SensorCountA=0, MPU9250SensorCountB=0, MPU9250SensorCountC=0, MPU9250SensorCountD=0;
			}
		}			
		APP_Show();	 //Send data to the APP //向APP发送数据
	  oled_show(); //Tasks are displayed on the screen //显示屏显示任务
   }
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{  
   static int count=0;	 
	 int Car_Mode_Show;
	
	 //Collect the tap information of the potentiometer, 
	 //and display the car model to be fitted when the car starts up in real time
	 //采集电位器档位信息，实时显示小车开机时要适配的小车型号
	 Divisor_Mode=2048/CAR_NUMBER+5;
	 Car_Mode_Show=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); 
	 if(Car_Mode_Show>5)Car_Mode_Show=5;
	
	 Voltage_Show=Voltage*100; 
	 count++;
	
	 if(Check==0)//The car displays normally when the self-check mode is not enabled //没有开启自检模式时小车正常显示
	 {	
		 //The first line of the display displays the content//
		 //显示屏第1行显示内容//
		 switch(Car_Mode_Show)
		 {
			case Mec_Car:       OLED_ShowString(0,0,"Mec "); break; 
			case Omni_Car:      OLED_ShowString(0,0,"Omni"); break; 
			case Akm_Car:       OLED_ShowString(0,0,"Akm "); break; 
			case Diff_Car:      OLED_ShowString(0,0,"Diff"); break; 
			case FourWheel_Car: OLED_ShowString(0,0,"4WD "); break; 
			case Tank_Car:      OLED_ShowString(0,0,"Tank"); break; 
		 }
		 
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		 {
			 //The Mec_car and omni_car show Z-axis angular velocity
			 //麦轮、全向轮小车显示Z轴角速度
			 OLED_ShowString(55,0,"GZ");
			 if( gyro[2]<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-gyro[2],5,12);
			 else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, gyro[2],5,12);		
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==FourWheel_Car||Car_Mode==Tank_Car)
		 {
			 //Akm_Car, Diff_Car, FourWheel_Car and Tank_Car Displays gyroscope zero
			 //阿克曼、差速、四驱、履带车显示陀螺仪零点
			 OLED_ShowString(55,0,"BIAS");
			 if( Deviation_gyro[2]<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-Deviation_gyro[2],3,12);  //Zero-drift data of gyroscope Z axis
			 else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, Deviation_gyro[2],3,12);	//陀螺仪z轴零点漂移数据	
		 }
		 //The first line of the display displays the content//
		 //显示屏第1行显示内容//
		 

		 //The second line of the display displays the content//
		 //显示屏第2行显示内容//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor A
			//麦轮、全向轮、四驱车显示电机A的目标速度和当前实际速度
			OLED_ShowString(0,10,"A");
			if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
														OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
			else                 	OLED_ShowString(15,10,"+"),
														OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
			
			if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
														OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,10,"+"),
														OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //The Akm_Car, Diff_Car and Tank_Car show Z-axis angular velocity
			 //阿克曼、差速、坦克小车显示Z轴角速度
			 OLED_ShowString(00,10,"GYRO_Z:");
			 if( gyro[2]<0)  OLED_ShowString(60,10,"-"),
											 OLED_ShowNumber(75,10,-gyro[2],5,12);
			 else            OLED_ShowString(60,10,"+"),
											 OLED_ShowNumber(75,10, gyro[2],5,12);			
		 }	 
		 //The second line of the display displays the content//
		 //显示屏第2行显示内容//
		 
		 //Lines 3 and 4 of the display screen display content//
		 //显示屏第3、4行显示内容//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor B
			//麦轮、全向轮、四驱车显示电机B的目标速度和当前实际速度
			OLED_ShowString(0,20,"B");		
			if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
														OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
			
			if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
														OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
														OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor C
			//麦轮、全向轮、四驱车显示电机C的目标速度和当前实际速度
			OLED_ShowString(0,30,"C");
			if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
														OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
			else                 	OLED_ShowString(15,30,"+"),
														OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
				
			if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
														OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,30,"+"),
														OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor A
			 //阿克曼、差速、履带车显示电机A的目标速度和当前实际速度
			 OLED_ShowString(0,20,"L:");
			 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
			 else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12);  
			 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
			 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor B
			 //阿克曼、差速、履带车显示电机B的目标速度和当前实际速度
			 OLED_ShowString(0,30,"R:");
			 if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
															OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
			 else                 	OLED_ShowString(15,30,"+"),
															OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12);  
				
			 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);
		 }
		 //Lines 3 and 4 of the display screen display content//
		 //显示屏第3、4行显示内容//
		 
		 //Line 5 of the display displays the content//
		 //显示屏第5行显示内容//
		 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
		 {
			  //Mec_Car Display the target speed and current actual speed of motor D
				//麦轮小车显示电机D的目标速度和当前实际速度
				OLED_ShowString(0,40,"D");
				if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
															OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
				else                 	OLED_ShowString(15,40,"+"),
															OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
				if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
		 }
		 else if(Car_Mode==Omni_Car)
		 {
			  // The Omni_car shows Z-axis angular velocity (1000 times magnification) in rad/s
				//全向轮小车显示Z轴角速度(放大1000倍)，单位rad/s
				OLED_ShowString(0,40,"MOVE_Z"); 			
				if(Send_Data.Sensor_Str.X_speed<0)	OLED_ShowString(60,40,"-"),
																						OLED_ShowNumber(75,40,-Send_Data.Sensor_Str.X_speed,5,12);
				else                              	OLED_ShowString(60,40,"+"),
																						OLED_ShowNumber(75,40, Send_Data.Sensor_Str.X_speed,5,12);
		 }
		 else if(Car_Mode==Akm_Car)
		 {
			  //Akm_Car displays the PWM value of the Servo
				//阿克曼小车显示舵机的PWM的数值
				OLED_ShowString(00,40,"SERVO:");
				if( Servo<0)		      OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(80,40,-Servo,4,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(80,40, Servo,4,12); 
		 }
		 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 // The Diff_Car and Tank_Car displays the PWM values of the left and right motors
			 //差速小车、履带车显示左右电机的PWM的数值
															 OLED_ShowString(00,40,"MA");
			 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
															 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(20,40,"+"),
															 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
															 OLED_ShowString(60,40,"MB");
			 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
															 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(80,40,"+"),
															 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
		 }
		 //Line 5 of the display displays the content//
		 //显示屏第5行显示内容//
			 
		 //Displays the current control mode //显示当前控制模式
		 if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");
		 else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
		 else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
		 else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
		 else if (Usart_ON_Flag==1) OLED_ShowString(0,50,"USART");
		 else                       OLED_ShowString(0,50,"ROS  ");
			
		 //Displays whether controls are allowed in the current car
		 //显示当前小车是否允许控制
		 if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  
		 else                      OLED_ShowString(45,50,"OFF"); 
			
																OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
			                          OLED_ShowString(88,50,".");
																OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
			                          OLED_ShowString(110,50,"V");
		 if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
		}
	 
		/* self-check related */
		/*自检相关*/
		//Display the self-check confirmation screen //显示自检确认界面
		if(Check==1&&!Checking&&Checked==0) 
		{
        OLED_ShowCheckConfirming();
		}
		//Display the interface for self-testing //显示自检进行界面
		if(Check==1&&Checking&&Checked==0) 
		{	
        OLED_ShowChecking();
		}	
		//Show the results of self-test //显示自检结果
		if(Check==1&&Checking&&Checked==1) 
		{		
      OLED_ShowCheckResult();
		}
		/* self-check related */
		/*自检相关*/
		
		//Refresh the screen //刷新屏幕
		if(Check==0)OLED_Refresh_Gram();
		//The screen refresh rate in self-check mode is reduced by 10 times
		//自检模式下屏幕刷新频率降低10倍
    else {if(count>10)OLED_Refresh_Gram(),count=0;}	
}

/**************************************************************************
Function: The OLED screen displays the self-check confirmation interface
Input   : none
Output  : none
函数功能：OLED屏显示自检确认界面
入口参数：无
返回  值：无
**************************************************************************/
void OLED_ShowCheckConfirming(void)
{
	//Press the user button to start the self-check
	//按下用户按键开始自检
  for(i=0;i<8;i++)	
	{
	 OLED_ShowCHinese(i*15, 5, i, CNSizeWidth, CNSizeHeight);
	}
	for(i=8;i<12;i++)	
	{
	 OLED_ShowCHinese((i-8)*15, 3, i, CNSizeWidth, CNSizeHeight);
	}
	
	// No operation Exit after * seconds
	//无操作*秒后退出
	for(i=12;i<15;i++)	
	{
	 OLED_ShowCHinese((i-12)*15, 1, i, CNSizeWidth, CNSizeHeight);
	}
	OLED_ShowNumber(48,45,(5-CheckCount/100),1,12);
	for(i=15;i<19;i++)	
	{
	 OLED_ShowCHinese((i-12)*15+10, 1, i, CNSizeWidth, CNSizeHeight);
	}
}
/**************************************************************************
Function: The OLED screen displays the interface for self-checking
Input   : none
Output  : none
函数功能：OLED屏显示自检进行界面
入口参数：无
返回  值：无
**************************************************************************/
void OLED_ShowChecking(void)
{
	//Press the user button to start the self-check
	//按下用户按键开始自检
  for(i=19;i<24;i++)	
	{
	 OLED_ShowCHinese((i-19)*15, 5, i, CNSizeWidth, CNSizeHeight);
	}
	OLED_ShowNumber(5,28,1,1,12);
	for(i=24;i<28;i++)	
	{
	 OLED_ShowCHinese((i-24)*15+18, 3, i, CNSizeWidth, CNSizeHeight);
	}
	OLED_ShowNumber(5,28,(200*CheckPhrase2-CheckCount)/100,2,12);
}
/**************************************************************************
Function: The OLED screen displays the interface of completion of self-inspection
Input   : none
Output  : none
函数功能：OLED屏显示自检完成界面
入口参数：无
返回  值：无
**************************************************************************/
void OLED_ShowCheckResult(void)
{
	int row=5;
//  for(i=32;i<36;i++)	
//	{
//	 OLED_ShowCHinese((i-32)*15, 5, i, CNSizeWidth, CNSizeHeight);
//	}
//	for(i=36;i<40;i++)	
//	{
//	 OLED_ShowCHinese((i-36)*15, 3, i, CNSizeWidth, CNSizeHeight);
//	}
//	
//	OLED_ShowString(65, 26,":");
//	OLED_ShowNumber(10,42,ErrorCode,9,16);
//	
//	OLED_ShowString( 0,00,"A");OLED_ShowNumber(20,00,EncoderA_Count,5,12);
//  OLED_ShowString(60,00,"B");OLED_ShowNumber(80,00,EncoderB_Count,5,12);
//  OLED_ShowString( 0,10,"C");OLED_ShowNumber(20,10,EncoderC_Count,5,12);
//  OLED_ShowString(60,10,"D");OLED_ShowNumber(80,10,EncoderD_Count,5,12);
//	OLED_ShowString( 0,20,"MPU");OLED_ShowNumber(80,20,MPU9250SensorCount,5,12);
//	OLED_ShowString( 0,30,"MPUErr");OLED_ShowNumber(80,30,MPU9250ErrorCount,5,12);
	
	if(ErrorCode&1) //The voltage is too low //电压过低
	{
	  for(i=40;i<44;i++)	
		{
		 OLED_ShowCHinese((i-40)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<1) //The enable switch is off //使能开关关闭
	{
	  for(i=44;i<50;i++)	
		{
		 OLED_ShowCHinese((i-44)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}

	
	if(ErrorCode&1<<3) //A drive is damaged //A驱动损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<4) //A drive line back connect //A驱动线反接
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<5) //B drive is damaged //B驱动损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<6) //B drive line back connect //B驱动线反接
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<7) //C drive is damaged //C驱动损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<8) //C drive line back connect //C驱动线反接
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<9) //D drive is damaged //D驱动损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<10) //D drive line back connect //D驱动线反接
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	if(ErrorCode&1<<11) //The power supply is unstable //电源不稳定
	{
	  for(i=64;i<69;i++)	
		{
		 OLED_ShowCHinese((i-64)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<12) //The temperature is too high //温度过高
	{
	  for(i=69;i<73;i++)	
		{
		 OLED_ShowCHinese((i-69)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<14||ErrorCode&1<<15) //The motor wire connection is wrong //电机线接错
	{
	  for(i=73;i<78;i++)	
		{
		 OLED_ShowCHinese((i-73)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	if(ErrorCode&1<<16) //The A encoder is broken //A编码器损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<17) //The B encoder is broken //B编码器损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<18) //The C encoder is broken //C编码器损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<19) //The D encoder is broken //D编码器损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	//MPU9250 has a lower corruption priority and is displayed later
	//MPU9250损坏 优先级较低，放在后面显示
	if(ErrorCode&1<<13)
	{
	  for(i=57;i<59;i++)	
		{
		 OLED_ShowCHinese((i-57)*15+80, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;	
		     if(row<1)OLED_ShowString( 20,44,"MPU9250");
		else if(row<3)OLED_ShowString( 20,27,"MPU9250");
		else if(row<5)OLED_ShowString( 20,10,"MPU9250");
	}
	
	//The error code is 0, the car is normal
	//错误代码为0，小车正常
	if(ErrorCode==0)
	{
	  for(i=78;i<82;i++)	
		{
		 OLED_ShowCHinese((i-78)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;	
	}
	
	if(row<5)OLED_ShowNumber(0, 7,1,1,16),OLED_ShowString( 10,10,".");
	if(row<3)OLED_ShowNumber(0,24,2,1,16),OLED_ShowString( 10,27,".");
	if(row<1)OLED_ShowNumber(0,41,3,1,16),OLED_ShowString( 10,44,".");

}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //对电池电压处理成百分比形式
	 Voltage_Show=(Voltage*1000-10000)/27;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //车轮速度单位转换为0.01m/s，方便在APP显示
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //用于交替打印APP数据和显示波形
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //发送参数到APP，APP在调试界面显示
		 printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);
		 PID_Send=0;	
	 }	
	 else	if(flag_show==0) 
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //发送参数到APP，APP在首页显示
		 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[2]);
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //发送参数到APP，APP在波形界面显示
		 printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);
	 }
}


