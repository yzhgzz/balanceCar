/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * �ļ���  ��main.c
**********************************************************************************/
#include "stm32f10x.h"
#include "led.h"
#include "usart.h"
#include "mpu6050.h"
#include "i2c_mpu6050.h"
#include "i2c.h"
#include "motor.h"
#include "SysTick.h"
#include "upstandingcar.h"
#include "outputdata.h"
/*
u16 encoder_num1;
u16 encoder_num2;

void delay_us(u32 n)
{
	u8 j;
	while(n--)
	for(j=0;j<10;j++);
}
void delay_ms(u32 n)
{
	while(n--)
	delay_us(1000);
}
void Delay(__IO u32 nCount);
 */
/*
 * ��������main
 * ����  ��������
 * ����  ����
 * ���  ����
 */
int main(void)
{	
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();
	
	//delay_nms(100);
	
	USART1_Config();
	USART3_Config();
	//NVIC_Configuration();
	TIM2_PWM_Init();
	MOTOR_GPIO_Config();
	//TIM3_Encoder_Init();
	//TIM4_Encoder_Init();
	TIM3_External_Clock_CountingMode();
	TIM4_External_Clock_CountingMode();
	//I2C_Config();
	i2cInit();
	delay_nms(10);
	MPU6050_Init();

	SysTick_Init();
	
	CarUpstandInit();
		GPIO_ResetBits(GPIOB, GPIO_Pin_4);
	// ʹ�ܵδ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while (1)
	{
	BluetoothControl();
#if 0 /*������ Ԥ��������*/
	MPU6050_Pose();
	//printf("Pitch/Roll/Yaw:%f,%f,%f\n",Pitch,Roll,Yaw);   //ROLL
	//printf("�����ǣ�%d,%d,%d\n",gyro[0],gyro[1],gyro[2]);  //gyro[0]
	//printf("���ٶȣ�%d,%d,%d\n",accel[0],accel[1],accel[2]);  //accel[1]

	  
	  
   OutData[0] = Roll;
   OutData[1] = gyro[0];
   OutData[2] = accel[1] ;
   //OutData[3] = g_iAccelInputVoltage_X_Axis;
   
   OutPut_Data();
#endif	  
	  //USART_SendData(USART3, 0XDD);
	   		
	//	LED1( ON );			  // ��
		//Delay(0x0FFFEF);
	//	LED1( OFF );		  // ��
	  /* 
		LED2( ON );
		Delay(0x0FFFEF);
		LED2( OFF );   	   */
#if 0
		encoder_num1=TIM_GetCounter(TIM3);
		TIM3->CNT = 0;
		encoder_num2=TIM_GetCounter(TIM4);
		TIM4->CNT = 0;
		//printf("\r\n this is a printf demo \r\n");
	    //printf("\r\n ��ӭʹ��������ƽ��С��BasicBalance���ذ�:) \r\n");		
		printf("\r\n---------������1---------%d \r\n",encoder_num1);
		printf("\r\n---------������2---------%d \r\n",encoder_num2);
#endif	  
		/*
		printf("\r\n---------���ٶ�X��ԭʼ����---------%d \r\n",GetData(ACCEL_XOUT_H));
		printf("\r\n---------���ٶ�Y��ԭʼ����---------%d \r\n",GetData(ACCEL_YOUT_H));	
		printf("\r\n---------���ٶ�Z��ԭʼ����---------%d \r\n",GetData(ACCEL_ZOUT_H));	
		printf("\r\n---------������X��ԭʼ����---------%d \r\n",GetData(GYRO_XOUT_H));	
		printf("\r\n---------������Y��ԭʼ����---------%d \r\n",GetData(GYRO_YOUT_H));	
		printf("\r\n---------������Z��ԭʼ����---------%d \r\n",GetData(GYRO_ZOUT_H));
		delay_ms(10);     									   */
	}
}
/*
void Delay(__IO u32 nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
} 
*/

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
