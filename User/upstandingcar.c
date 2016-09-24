
#include "upstandingcar.h"
#include "I2C_MPU6050.h"
#include "MOTOR.h"
#include "led.h"
#include "USART.H"
#include "MPU6050.H"

u8 g_u8LEDCount; 
u8 g_u8MainEventCount;
u8 g_u8SpeedControlCount;
u8 g_u8SpeedControlPeriod;
u8 g_u8DirectionControlPeriod;
u8 g_u8DirectionControlCount;
//s32 g_liAccAdd;
//s32 g_liGyroAdd;
/******电机控制参数******/
float g_fSpeedControlOut;
float g_fSpeedControlOutOld;
float g_fSpeedControlOutNew;
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;

float g_fCarAngle;

/******速度控制参数******/
//s32   g_s32LeftMotorPulse;
//s32   g_s32RightMotorPulse;
s16   g_s16LeftMotorPulse;
s16	  g_s16RightMotorPulse;

s32   g_s32LeftMotorPulseOld;
s32   g_s32RightMotorPulseOld;
s32   g_s32LeftMotorPulseSigma;
s32   g_s32RightMotorPulseSigma;

float g_fCarSpeed;
float g_fCarSpeedOld;
//float g_fCarSpeedOld;
float g_fCarPosition;
//float fSpeedCtrlPeriod=100.0;
//以下参考为重点调试参考，同电池电压有关，建议充好电再调试
/*-----角度环和速度环PID控制参数-----*/
float  g_fCarAngle_P = 29;//10.0  14	10 //调大小时会左右摆，调大时会振动  请调到基本能够站立
float  g_fCarAngle_D = 0.0389;	// 0.008  0.01	 调小时反应慢，调大时会干扰
float  g_fCarSpeed_P = 5.1;// 10.2  4.0	4	  5 -10		9  //调小一点稳定
float  g_fCarSpeed_I = 0.0898;//0.1  0.018  0.01		0.0025- 0.015  0.1	 电压低时调小，调大，蓝牙控制不稳，会倒
/******蓝牙控制参数******/
float g_fBluetoothSpeed;
float g_fBluetoothDirection;
float g_fBluetoothDirectionOld;
float g_fBluetoothDirectionNew;
float g_fBluetoothDirectionOut;


void delay_nms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}

/***************************************************************
** 函数名称: CarUpstandInit
** 功能描述: 全局变量初始化函数
** 输　入:   
** 输　出:   
***************************************************************/
void CarUpstandInit(void)
{
	//g_iAccelInputVoltage_X_Axis = g_iGyroInputVoltage_Y_Axis = 0;
	g_s16LeftMotorPulse = g_s16RightMotorPulse = 0;
	g_s32LeftMotorPulseOld = g_s32RightMotorPulseOld = 0;
	g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;

	g_fCarSpeed = g_fCarSpeedOld = 0;
	g_fCarPosition = 0;
	g_fCarAngle    = 0;
	//g_fGyroAngleSpeed = 0;
	//g_fGravityAngle   = 0;
	//g_fGyroscopeAngleIntegral = 0;
	g_fAngleControlOut = g_fSpeedControlOut = g_fBluetoothDirectionOut = 0;
	g_fLeftMotorOut    = g_fRightMotorOut   = 0;
	g_fBluetoothSpeed  = g_fBluetoothDirection = 0;
	g_fBluetoothDirectionNew = g_fBluetoothDirectionOld = 0;

  	g_u8MainEventCount=0;
	g_u8SpeedControlCount=0;
    g_u8SpeedControlPeriod=0;
}

/***************************************************************
** 函数名称: SampleInputVoltage
** 功能描述: 采样函数             
** 输　入:   
** 输　出:   
** 全局变量: 
***************************************************************/

/***************************************************************
** 函数名称: AngleControl
** 功能描述: 角度环控制函数
** 输　入:   
** 输　出:   
** 全局变量: 
***************************************************************/
void AngleControl(void)	 
{
/*
	float fDeltaValue;
	g_iGyroInputVoltage_Y_Axis = g_iGyroInputVoltage_Y_Axis / 40;
	g_fGravityAngle = (g_iAccelInputVoltage_X_Axis - GRAVITY_OFFSET) * GRAVITY_ANGLE_RATIO ;
	g_fGyroAngleSpeed = (g_iGyroInputVoltage_Y_Axis * 40 - GYRO_OFFSET) * GYROSCOPE_ANGLE_RATIO  ;
	
	g_fCarAngle = g_fGyroscopeAngleIntegral ;
	
	fDeltaValue = (g_fGravityAngle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;
	g_fGyroscopeAngleIntegral += (g_fGyroAngleSpeed + fDeltaValue) / GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
*/
	g_fCarAngle = Roll - CAR_ZERO_ANGLE;
	g_fAngleControlOut =  g_fCarAngle * g_fCarAngle_P + \
	  gyro[0] * g_fCarAngle_D ;

}
/***************************************************************
** 函数名称: SetMotorVoltageAndDirection
** 功能描述: 电机转速及方向控制函数             
** 输　入:   
** 输　出:   
** 全局变量: 
***************************************************************/
void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	u16 u16LeftMotorValue;
	u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)
    {	
	  GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else
    {
	GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );	
      
      s16RightVoltage = s16RightVoltage;
    }
	 u16RightMotorValue= (u16)s16RightVoltage;
	 u16LeftMotorValue = (u16)s16LeftVoltage;

	if(u16RightMotorValue>255)  
	{
		u16RightMotorValue=255;
	}
	if(u16LeftMotorValue>255)
	{
	   u16LeftMotorValue=255;
	}
    //TIM2_PWM_CHANGE(u16RighttMotorValue,u16LeftMotorValue) ;
	//TIM2->CCR3 = 	u16RightMotorValue ; 
   	//TIM2->CCR4 =    u16LeftMotorValue; 
	TIM_SetCompare3(TIM2,u16RightMotorValue);
	TIM_SetCompare4(TIM2,u16LeftMotorValue);

#if 1	 /*判断车辆是否跌倒，调试用*/

	if(g_fCarAngle > 180 || g_fCarAngle < (-180))
	{
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);  
	}

#endif
}

/***************************************************************
** 函数名称: MotorOutput
** 功能描述: 电机输出函数
             将直立控制、速度控制、方向控制的输出量进行叠加,并加
			 入死区常量，对输出饱和作出处理。
** 输　入:   
** 输　出:   
** 全局变量: 
***************************************************************/
void MotorOutput(void)
{


	g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut - g_fBluetoothDirectionOut  ;
	
	
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirectionOut ;

	//g_fLeftMotorOut  = g_fAngleControlOut ;
	
	
	//g_fRightMotorOut = g_fAngleControlOut ;


	/*增加死区常数*/
	if(g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
	else if(g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
	if(g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if(g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*输出饱和处理，防止超出PWM范围*/
	/*	
	if((s16)g_fLeftMotorOut  > MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)g_fLeftMotorOut  < MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)g_fRightMotorOut > MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)g_fRightMotorOut < MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;
	*/
    SetMotorVoltageAndDirection((s16)g_fLeftMotorOut,(s16)g_fRightMotorOut);
}

void GetMotorPulse(void)  //采集电机速度脉冲
{ 
  uint16_t u16TempLeft;
  uint16_t u16TempRight;
  //int16_t  s16TempLeft;
  //int16_t  s16TempRight;
  u16TempLeft = TIM_GetCounter(TIM3);     
  u16TempRight= TIM_GetCounter(TIM4);
  TIM3->CNT = 0;
  TIM4->CNT = 0;   //清零
  g_s16LeftMotorPulse=u16TempLeft;
  g_s16RightMotorPulse=u16TempRight;
  //s16TempLeft =	u16TempLeft;
  //s16TempRight = u16TempRight;
  /*
  //------------低通滤波---------------//
  g_s32LeftMotorPulse  = 0.7 * g_s32LeftMotorPulseOld  + 0.3 * g_s32LeftMotorPulse ;
  g_s32RightMotorPulse = 0.7 * g_s32RightMotorPulseOld + 0.3 * g_s32RightMotorPulse;
  g_s32LeftMotorPulseOld  = g_s32LeftMotorPulse ;
  g_s32RightMotorPulseOld = g_s32RightMotorPulse;
  //-----------------------------------//
  */
  if(!MOTOR_LEFT_SPEED_POSITIVE)  
  {
  	g_s16LeftMotorPulse  = (-g_s16LeftMotorPulse) ; 
  }
  if(!MOTOR_RIGHT_SPEED_POSITIVE) 
  {
  	g_s16RightMotorPulse = (-g_s16RightMotorPulse);
  }
  #if 0
		//g_s32LeftMotorPulse=TIM_GetCounter(TIM3);
		
		//g_s32RightMotorPulse=TIM_GetCounter(TIM4);
		
		//printf("\r\n this is a printf demo \r\n");
	    //printf("\r\n 欢迎使用两轮自平衡小车BasicBalance主控板:) \r\n");		
		printf("\r\n---编码器1:%d \r\n",g_s16LeftMotorPulse);
		printf("\r\n---编码器2:%d \r\n",g_s16RightMotorPulse);
#endif	 
  g_s32LeftMotorPulseSigma +=  g_s16LeftMotorPulse;
  g_s32RightMotorPulseSigma += g_s16RightMotorPulse; 
}
/***************************************************************
** 函数名称: SpeedControl
** 功能描述: 速度环控制函数
** 输　入:   
** 输　出:   
** 全局变量: 
***************************************************************/

void SpeedControl(void)
{
    //float fP,fI;
//	float fDelta;
	
	
	g_fCarSpeed = (g_s32LeftMotorPulseSigma  + g_s32RightMotorPulseSigma ) * 0.5 ;
    g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;	  //全局变量 注意及时清零
    	
	/*
	g_fCarSpeed *= CAR_SPEED_CONSTANT;	 //单位：转/秒
	g_fCarSpeed = 0.6 * g_fCarSpeedOld + 0.4 * g_fCarSpeed ;
	g_fCarSpeedOld = g_fCarSpeed;

	fDelta = CAR_SPEED_SET;
	fDelta -= g_fCarSpeed;   
	
	fP = fDelta * g_fCarSpeed_P;
  	fI = fDelta * g_fCarSpeed_I;
  	g_fCarPosition += fI;
	g_fCarPosition += g_fBluetoothSpeed;	  
  //积分上限设限
	if(g_fCarPosition > CAR_POSITION_MAX)    g_fCarPosition = CAR_POSITION_MAX;
	if(g_fCarPosition < CAR_POSITION_MIN)    g_fCarPosition = CAR_POSITION_MIN;
#if 0
	g_fSpeedControlOutOld = g_fSpeedControlOutNew;
	g_fSpeedControlOutNew = fP + g_fCarPosition;
#endif
#if 1
	g_fSpeedControlOut =  fP + g_fCarPosition;
#endif							   */

	g_fCarSpeed = 0.7 * g_fCarSpeedOld + 0.3 * g_fCarSpeed ;
	g_fCarSpeedOld = g_fCarSpeed;

	g_fCarSpeed *= CAR_SPEED_CONSTANT;	 //单位：转/秒

	g_fCarPosition += g_fCarSpeed; 		 //路程  即速度积分
	
	g_fCarPosition += g_fBluetoothSpeed;
	//积分上限设限//
	if((s32)g_fCarPosition > CAR_POSITION_MAX)    g_fCarPosition = CAR_POSITION_MAX;
	if((s32)g_fCarPosition < CAR_POSITION_MIN)    g_fCarPosition = CAR_POSITION_MIN;
	
	g_fSpeedControlOutOld = g_fSpeedControlOutNew;
	g_fSpeedControlOutNew = (CAR_SPEED_SET - g_fCarSpeed) * g_fCarSpeed_P +\
		(CAR_POSITION_SET - g_fCarPosition) * g_fCarSpeed_I; 		  

}
void SpeedControlOutput(void)
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld ;
  g_fSpeedControlOut = fValue * (g_u8SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld; 
  
}


/***************************************************************
** 函数名称: BluetoothControl
** 功能描述: 蓝牙控制函数
             手机发送内容
			 上：00000010    下：00000001
             左：00000011    右：00000100
             停止：00000000
             功能键（可自编下位机程序扩展）：
             A:00000101      B:00000110
             C:00000111      D:00001000
** 输　入:   
** 输　出:   
***************************************************************/
void BluetoothControl(void)	 
{
	u8 u8BluetoothValue;
	
	while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE)!=SET);	//等待USART3的接受区不为空
	//{ 
		u8BluetoothValue =USART3->DR;
		//ucBluetoothValue = USART_ReceiveData(USART3);
		USART_ClearFlag(USART3,USART_FLAG_RXNE); 
		//USART1_Send_Byte(u8BluetoothValue);
	
	//}
		
	switch (u8BluetoothValue)
	{
	  case 0x02 : g_fBluetoothSpeed =   600 ; break;	   //前进数字大时速度快 70		   300
	  case 0x01 : g_fBluetoothSpeed = (-300);  break;	   //后退		  -300
	  case 0x03 : g_fBluetoothDirection =   50 ;  break;//左转
	  case 0x04 : g_fBluetoothDirection = (-50);  break;//右转
	  case 0x05 : g_fBluetoothSpeed =   100 ; break ;//500
	  case 0x06 : g_fBluetoothSpeed = (-100); break ;//-500
	  case 0x07 : g_fCarSpeed_P = 20 ; g_fCarSpeed_I = 0.1; break;
	  case 0x08 : g_fCarSpeed_P = 20 ; g_fCarSpeed_I = 0.1;  break;
	  //case 0x07 : g_fBluetoothDirection =   70 ;  break;
	  //case 0x08 : g_fBluetoothDirection = (-70);  break;
	  default : g_fBluetoothSpeed = 0; g_fBluetoothDirection = 0;break;
	}
	
}
void DirectionControl(void)
{
#if 1
	g_fBluetoothDirectionOld = g_fBluetoothDirectionNew;
	g_fBluetoothDirectionNew = g_fBluetoothDirection;
#endif
#if 0
	g_fBluetoothDirectionOut = g_fBluetoothDirection;
#endif
}
void DirectionControlOutput(void)
{
  float fValue;
  
  fValue = g_fBluetoothDirectionNew - g_fBluetoothDirectionOld;
  g_fBluetoothDirectionOut = fValue *(g_u8DirectionControlPeriod + 1) / 200 + g_fBluetoothDirectionOld; //  
}
