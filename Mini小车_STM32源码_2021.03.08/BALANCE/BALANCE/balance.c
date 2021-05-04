#include "balance.h"

int Time_count=0; //Time variable //计时变量

// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0; 

//Self-check the relevant variables //自检相关变量
int EncoderA_Count=0, EncoderB_Count=0, EncoderC_Count=0, EncoderD_Count=0;                               //联合陀螺仪数据检测编码器与驱动
int MPU9250ErrorCount, MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;//联合陀螺仪数据检测编码器与驱动

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据     
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //全向移动小车才开启速度平滑处理
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
  
      //Get the smoothed data 
			//获取平滑处理后的数据			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //麦克纳姆轮小车
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//全向轮小车
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //没有使用到
		}
		
		//Ackermann structure car
		//阿克曼小车
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			int K=1000;
			float Ratio=1, Angle;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表前轮转向角度
			Angle=Vz;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			Angle=target_limit_float(Angle,-0.35f,0.35f);
			
			// The software compensates for the front wheel steering Angle due to mechanical structure limitations
			//机械结构限制，软件对前轮转向角度进行补偿
			if(Angle<0)Ratio=1.054;
			else if(Angle>0)Ratio=0.838;
			else Ratio=0;
			
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = Vx*(1-Wheel_spacing*tan(Angle)/2/Axle_spacing);
			MOTOR_B.Target   = Vx*(1+Wheel_spacing*tan(Angle)/2/Axle_spacing);
			// The PWM value of the servo controls the steering Angle of the front wheel
			//舵机PWM值，舵机控制前轮转向角度
			Servo=(SERVO_INIT-Angle*K*Ratio); 
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
			Servo=target_limit_int(Servo,900,2000);	//Servo PWM value limit //舵机PWM值限幅
		}
		
		//Differential car
		//差速小车
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
		
		//FourWheel car
		//四驱车
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
					
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
		}
		
		//Tank Car
		//履带车
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//此任务以100Hz的频率运行（10ms控制一次）
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			
			//Time count is no longer needed after 30 seconds
			//时间计数，30秒后不再需要
			if(Time_count<3000)Time_count++;
			
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//获取编码器数据，即车轮实时速度，并转换位国际单位
			Get_Velocity_Form_Encoder();   
			
			if(Check==0) //If self-check mode is not enabled //如果没有启动自检模式
			{
				if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //处理APP遥控命令
				else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //处理航模遥控命令
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄控制命令
				
				//CAN, Usart 1, Usart 3 control can directly get the three axis target speed, 
				//without additional processing
				//CAN、串口1、串口3(ROS)控制直接得到三轴目标速度，无须额外处理
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
				
				//Click the user button to update the gyroscope zero
				//单击用户按键更新陀螺仪零点
				Key(); 
				
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0
				//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
				if(Turn_Off(Voltage)==0) 
				 { 			
           //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
					 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
						 
					 //Set different PWM control polarity according to different car models
					 //根据不同小车型号设置不同的PWM控制极性
					 switch(Car_Mode)
					 {
							case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //麦克纳姆轮小车
							case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //全向轮小车
							case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //阿克曼小车
							case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //两轮差速小车
							case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //四驱车 
							case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //履带车
					 }
				 }
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
				 else	Set_Pwm(0,0,0,0,0); 
			 }	
			 //If self-check mode is enabled, the run self-check function is executed
			 //如果开启了自检模式，则执行运行自检函数
       else CheckTask(); 
		 }  
}

/**************************************************************************
Function: Self - check, check the car itself error, and generate the corresponding error code
Input   : none
Output  : none
函数功能：自检，检查小车自身错误，并产生对应错误代码
入口参数：无
返回  值：无
**************************************************************************/
int CheckTask(void)
{  
	 // Control forward, actuallyreverse, 0001.Inversion of control, actual forward, 0010.
   // Control forward, actuallystop, 0100.Inversion of Control, Actual Stop, 1000.
	 //Here we define PWM to be positive
	 //控制正转，实际反转，0001。控制反转，实际正转，0010。
	 //控制正转，实际停转，0100。控制反转，实际停转，1000。这里定义PWM给正值为正转
	 static int A,B,C,D; 

	 static float MaxVoltage=0, MinVoltage=20, Temperature, LastTemperature=3000, TemperatureBias; 
	 static int WireWrong=0; //Whether the motor wire is connected to the wrong marker //电机线是否接错标志位
	 
	 if(Check)CheckCount++;  //Countdown to self-check confirmation //自检确认倒计时
	 else CheckCount=0;
	
	 //Confirm success, start self-test //确认成功，开始自检
	 if(Check&&Checking) 
	 {
		 //Test for 2 seconds at each stage, wait for 1 second before the movement becomes stable
		 //每个阶段测2秒，等待1秒运动稳定后开始检测
		 int CheckPeriod=200, WaitPeriod=100; 
		 
		 // Voltage fluctuation detection
		 //电压波动检测
		 if(Voltage>MaxVoltage) MaxVoltage=Voltage;
		 if(Voltage<MinVoltage) MinVoltage=Voltage;
		 
		 //MPU9250 detection
		 //MPU9250检测
		 if(CheckCount<=(1+CheckPeriod*CheckPhrase2))
		 {
			 if(gyro[0]==0||gyro[1]==0||gyro[2]==0||accel[0]==0||accel[1]==0||accel[2]==0||MPU_Get_Temperature()==0)MPU9250ErrorCount++;
		 }
		 
		 //Temperature detection
		 //温度检测
		 TemperatureBias=MPU_Get_Temperature()-LastTemperature;
		 if(TemperatureBias<10&&TemperatureBias>-10)
		 {
			 Temperature=(MPU_Get_Temperature()+LastTemperature)/2;
			 LastTemperature=Temperature;
		 }
		 
		 //Open loop control motor test //开环控制电机检测
		 if(0<CheckCount&&CheckCount<(CheckPeriod)) //Test motor A is running //测试A电机正转 
		 {
			 if(CheckCount==1)Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(2000, 0, 0, 0, 1500);
			 if(CheckCount>(0+CheckPeriod-WaitPeriod))
			 {
				 static int ZeroACount=0;
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.A<-3)A=A|1;    //Motor reversal //电机反转		
         if(ZeroACount>90)			 A=A|1<<2; //The motor stops //电机停转
			 }			 
		 }
		 else if(CheckPeriod<CheckCount&&CheckCount<(CheckPeriod*(2))) //Test A motor reversal //测试A电机反转
		 {
			 if(CheckCount==(1+CheckPeriod))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(-2000, 0, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*2-WaitPeriod))
			 {
				 static int ZeroACount=0;
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.A>3)A=A|1<<1; //The motor is running //电机正转 	
         if(ZeroACount>90)			A=A|1<<3; //The motor stops //电机停转				 
			 }		 
		 }
		 
		 else if(CheckPeriod*(2)<CheckCount&&CheckCount<(CheckPeriod*(3))) //Test motor B is running //测试B电机正转
		 {
			 if(CheckCount==(1+CheckPeriod*2))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 2000, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*3-WaitPeriod))
			 {
				 static int ZeroBCount=0;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.B<-3)B=B|1;    //Motor reversal //电机反转 
				 if(ZeroBCount>90)			 B=B|1<<2; //The motor stops //电机停转
			 }			 
		 }
		 else if(CheckPeriod*(3)<CheckCount&&CheckCount<(CheckPeriod*(4))) //Test B motor reversal //测试B电机反转
		 {
			 if(CheckCount==(1+CheckPeriod*3))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, -2000, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*4-WaitPeriod))
			 {
				 static int ZeroBCount=0;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.B>3)B=B|1<<1; //The motor is running //电机正转
				 if(ZeroBCount>90)			B=B|1<<3; //The motor stops //电机停转
			 }			 
		 }
		 
		 else if(CheckPeriod*(4)<CheckCount&&CheckCount<(CheckPeriod*5)&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*4))Set_Pwm(0,0,0,0,1500);    
			 Set_Pwm(0, 0, 2000, 0, 1500);//Test motor C is running //测试C电机正转
			 if(CheckCount>(CheckPeriod*5-WaitPeriod))
			 {
				 static int ZeroCCount=0;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.C<-3)C=C|1;    //Motor reversal //电机反转	
         if(ZeroCCount>90)			 C=C|1<<2; //The motor stops //电机停转				 
			 }				 
		 }
		 else if(CheckPeriod*(5)<CheckCount&&CheckCount<(CheckPeriod*6)&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*5))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, -2000, 0, 1500); //Test C motor reversal //测试C电机反转
			 if(CheckCount>(CheckPeriod*6-WaitPeriod))
			 {
				 static int ZeroCCount=0;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.C>3)C=C|1<<1; //The motor is running //电机正转
				 if(ZeroCCount>90)			C=C|1<<3; //The motor stops //电机停转	
			 }				 
		 }
		 
		 else if(CheckPeriod*(6)<CheckCount&&CheckCount<(CheckPeriod*7)&&(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*6))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, 0, 2000, 1500);//Test motor D is running //测试D电机正转
			 if(CheckCount>(CheckPeriod*7-WaitPeriod))
			 {
				 static int ZeroDCount=0;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 if(OriginalEncoder.D<-3)D=D|1;    //Motor reversal //电机反转		
         if(ZeroDCount>90)			 D=D|1<<2; //The motor stops //电机停转					 
			 }	 
		 }
		 else if(CheckPeriod*(7)<CheckCount&&CheckCount<(CheckPeriod*8)&&(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*7))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, 0, -2000, 1500); //Test D motor reversal //测试D电机反转
			 if(CheckCount>(CheckPeriod*8-WaitPeriod))
			 {
				 static int ZeroDCount=0;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 if(OriginalEncoder.D>3)D=D|1<<1; //The motor is running //电机正转
				 if(ZeroDCount>90)			D=D|1<<3; //The motor stops //电机停转	
			 }				 
		 }
		 //Open loop control motor test //开环控制电机检测		 
		 
		 //Check the encoder A , four_wheel car will check the encoder AB at the same time
		 //检测编码器A，如果是四驱车同时检测AB
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
			 //9250Z轴速度值低次数统计(次数过高代表车没有被驱动)，编码器速度值低速度统计。
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
		 //检测编码器B，四驱车同时检查CD,不检查B
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
			 //9250Z轴速度值低次数统计(次数过高代表车没有被驱动)，编码器速度值低速度统计。
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
		 
		 //Check the encoder C //检测编码器C
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
			 //9250Z轴速度值低次数统计(次数过高代表车没有被驱动)，编码器速度值低速度统计。
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+3)-WaitPeriod))
			 {
				 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountC++;		
				 if(MOTOR_C.Encoder<0.02&&MOTOR_C.Encoder>-0.02)EncoderC_Count++;					 
			 }				 
		 }
		 
		 // Check the encoder D
		 //检测编码器D
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
			 //9250Z轴速度值低次数统计(次数过高代表车没有被驱动)，编码器速度值低速度统计。
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+4)-WaitPeriod))
			 {
				 //The wheat-wheel D motor structure is weak in driving the car rotation, so the test is tolerant
				 //麦轮D电机结构上带动小车转动的能力弱，检测宽容处理		
				 if(gyro[2]<400&&gyro[2]>-400)MPU9250SensorCountD++;	
				 if(MOTOR_D.Encoder<0.02&&MOTOR_D.Encoder>-0.02)EncoderD_Count++;				 
			 }				 
		 }
		 
		 //Check whether the motor wire is connected wrong //检测电机线是否接错 
		 else if((CheckPeriod*(CheckPhrase2-2))<CheckCount && CheckCount<(CheckPeriod*(CheckPhrase2-1))&&Car_Mode==Mec_Car) //Test the motor line twice //麦轮车测试两次电机线
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
				 if(WireWrongCount>80)           WireWrong=1;//ErrorCode=ErrorCode|1<<15; //Motor wire connection is wrong, error code input //电机线接错，错误代码录入	 
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
					 //单独驱动时无编码器数据，整体运动时编码器有数据，说明驱动损坏
					 if((A&12)&&ZeroACount<30)ErrorCode=ErrorCode|1<<3; 
					 if((B&12)&&ZeroBCount<30)ErrorCode=ErrorCode|1<<5;
					 if((C&12)&&ZeroCCount<30)ErrorCode=ErrorCode|1<<7;
					 if((D&12)&&ZeroDCount<30)ErrorCode=ErrorCode|1<<9;
				 }
				 
				 //If the driver and encoder are not normal, it is confirmed that the motor wire connection is wrong
				 //驱动、编码器不正常则确认电机线接错
				 if(gyro[2]>64000||gyro[2]<-64000)gyro[2]=501;
				 if(Car_Mode==Mec_Car)
				 {
				   if((gyro[2]>500||gyro[2]<-500))   WireWrongCount++;
					 if(WireWrongCount>80)             WireWrong=1;//ErrorCode=ErrorCode|1<<15; //Motor wire connection is wrong, error code input //电机线接错，错误代码录入
				 }
				 else
				 {
					 if((gyro[2]<100))                 WireWrong=1;//ErrorCode=ErrorCode|1<<14; //Motor wire connection is wrong, error code input //电机线接错，错误代码录入
				 }			 
			 }				 
		 }
		 //Check whether the motor wire is connected wrong //检测电机线是否接错

		 //statistics error code //统计错误代码
		 else if(CheckCount==(1+CheckPeriod*(CheckPhrase2)))
		 {			 		 
			 if(MPU9250ErrorCount>100*CheckPhrase2/2)     ErrorCode=ErrorCode|1<<13; //MPU9250 damage //MPU9250损坏
			 if(Temperature>7000)                         ErrorCode=ErrorCode|1<<12; //The temperature is too high //温度过高
			 if((MaxVoltage-MinVoltage)>5)                ErrorCode=ErrorCode|1<<11; //The voltage fluctuation is too large, the power supply is not stable //电压波动过大，电源不稳定
			 if(Voltage<10)                               ErrorCode=ErrorCode|1;     //The voltage is too low //电压过低
			 if(EN==0)                                    ErrorCode=ErrorCode|1<<1;  // The switch is off //开关处于关闭状态
			 if(A==16&&B==16)                             ErrorCode=ErrorCode|1<<2;  //The car model is wrong, and the potentiometer in the upper right corner is adjusted //小车型号选错，右上角电位器调型号
			 
			 switch(A&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<3; break; //The drive is damaged //驱动损坏   0001
				case 2:                 ErrorCode=ErrorCode|1<<3; break; //The drive is damaged //驱动损坏   0010
				case 3:                 ErrorCode=ErrorCode|1<<4; break; //Reverse connection of the drive wire //驱动线反接 0011											
				default:                                          break;
			 }
			 switch(B&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<5; break; //The drive is damaged //驱动损坏   0001
				case 2:                 ErrorCode=ErrorCode|1<<5; break; //The drive is damaged //驱动损坏   0010
				case 3:                 ErrorCode=ErrorCode|1<<6; break; //Reverse connection of the drive wire //驱动线反接 0011											
				default:                                          break;
			 }
			 switch(C&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<7; break; //The drive is damaged //驱动损坏   0001
				case 2:                 ErrorCode=ErrorCode|1<<7; break; //The drive is damaged //驱动损坏   0010
				case 3:                 ErrorCode=ErrorCode|1<<8; break; //Reverse connection of the drive wire //驱动线反接 0011											
				default:                                          break;
			 }
			 switch(D&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<9; break; //The drive is damaged //驱动损坏   0001
				case 2:                 ErrorCode=ErrorCode|1<<9; break; //The drive is damaged //驱动损坏   0010
				case 3:                 ErrorCode=ErrorCode|1<<10;break; //Reverse connection of the drive wire //驱动线反接 0011											
				default:                                          break;
			 }
			 
			 //Four_wheel car driving a single wheel does not move the car, resulting in gyroscope data
			 //四驱车驱动单个轮子无法使车子移动，导致陀螺仪无数据
			 if(MPU9250SensorCountA>=80){if(EncoderA_Count>80)                          ErrorCode=ErrorCode|1<<3; } //Drive A is broken   //驱动A损坏
			 else                       {if(EncoderA_Count>80&&(!(ErrorCode&1<<3)))     ErrorCode=ErrorCode|1<<16;} //Encoder A is broken //编码器A损坏 
			 if(MPU9250SensorCountB>=80){if(EncoderB_Count>80)                          ErrorCode=ErrorCode|1<<5; } //Drive B is broken   //驱动B损坏
			 else                       {if(EncoderB_Count>80&&(!(ErrorCode&1<<5)))     ErrorCode=ErrorCode|1<<17;} //Encoder B is broken //编码器B损坏 
			 if(MPU9250SensorCountC>=80){if(EncoderC_Count>80)                          ErrorCode=ErrorCode|1<<7; } //Drive B is broken   //驱动C损坏
			 else                       {if(EncoderC_Count>80&&(!(ErrorCode&1<<7)))     ErrorCode=ErrorCode|1<<18;} //Encoder C is broken //编码器C损坏 
			 if(MPU9250SensorCountD>=90){if(EncoderD_Count>80)                          ErrorCode=ErrorCode|1<<9; } //Drive B is broken   //驱动D损坏
			 else                       {if(EncoderD_Count>80&&(!(ErrorCode&1<<9)))     ErrorCode=ErrorCode|1<<19;} //Encoder D is broken //编码器D损坏 
			 //                          1111 0000 0111 1111 1000
			 if(WireWrong==1&&(ErrorCode&0xf07f8)==0)ErrorCode=ErrorCode|1<<14;
			 OLED_Clear();
			 Checked=1;
	   }
		 //Statistics error code //统计错误代码
			
		 //Test completed, motor set to 0 //检测完成，电机置0
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
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
  if(motor_a<0)		AIN2=0,		AIN1=1;
	else				    AIN2=1,		AIN1=0;
	//Motor speed control 
	//电机转速控制
	PWMA=abs(motor_a);

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)		BIN2=1,		BIN1=0;
	else 	          BIN2=0,		BIN1=1;
	//Motor speed control 
	//电机转速控制
	PWMB=abs(motor_b);

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c>0)		CIN2=0,		CIN1=1;
	else 	          CIN2=1,		CIN1=0;
	//Motor speed control 
	//电机转速控制
  PWMC=abs(motor_c);

	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d>0)		DIN2=0,		DIN1=1;
	else 	          DIN2=1,		DIN1=0;
	//Motor speed control 
	//电机转速控制
	PWMD=abs(motor_d);
	
	//Servo control
	//舵机控制
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
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
函数功能：限幅函数
入口参数：幅值
返回  值：无
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
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
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
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
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
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
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
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
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
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//加速
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //减速	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //对PS2手柄控制命令进行处理
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作

	  //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //左摇杆左右方向。控制左右移动。麦轮全向轮才会使用到改通道。阿克曼小车使用该通道作为PWM输出控制舵机
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//右摇杆前后方向。油门/加减速。
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //油门相关
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
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
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
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
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
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
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
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
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。已停止使用
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//如果连续6次接近满幅输出，判断为电机乱转，让电机失能	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}
