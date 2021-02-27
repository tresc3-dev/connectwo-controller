/*
 * ros_main.cpp
 *
 *  Created on: Feb 20, 2021
 *      Author: colson
 */

#include <Pid.h>
#include <ros_main.h>
#include <periphGpio.h>

#include "ros.h"
#include "ros/time.h"

#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <Nonholonomic.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
sensor_msgs::Imu imu_msg;

ros::Publisher pub_str("/tresc3/chatter", &str_msg);
ros::Publisher pub_imu("/imu", &imu_msg);

char hello[] = "hello world!";

void cmdVelCallback(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

PeriphGPIO __ledState(STATE_LED_GPIO_Port, STATE_LED_Pin, 1000);

PeriphGPIO __led0(LED0_GPIO_Port, LED0_Pin, 100);
PeriphGPIO __led1(LED1_GPIO_Port, LED1_Pin, 100);
PeriphGPIO __led2(LED2_GPIO_Port, LED2_Pin, 100);
PeriphGPIO __led3(LED3_GPIO_Port, LED3_Pin, 100);



tresc3::pidProperty<long> pidSetting = {5000, 100, 0,
										0,
										0, 0,
										0, 0,
										0, 200, 0,
										0, 0, 999,
										1000};
tresc3::Pid<long> pid[4] = {pidSetting, pidSetting, pidSetting, pidSetting};
tresc3::pidProperty<long> properties[4];
uint16_t g_NowEncoder[4] = {0,};
uint16_t g_PastEncoder[4] = {0,};
GPIO_TypeDef* g_MotorGpio[4] = {GPIOB, GPIOC, GPIOB, GPIOB};
uint16_t g_MotorGpioPin[4] = {GPIO_PIN_0, GPIO_PIN_5, GPIO_PIN_1, GPIO_PIN_2};
uint32_t* g_MotorPwmAddr[4] = {(uint32_t *)&TIM8->CCR4, (uint32_t *)&TIM8->CCR3, (uint32_t *)&TIM8->CCR2, (uint32_t *)&TIM8->CCR1};
uint32_t* g_MotorEncAddr[4] = {(uint32_t *)&TIM4->CNT, (uint32_t *)&TIM5->CNT, (uint32_t *)&TIM3->CNT, (uint32_t *)&TIM1->CNT};
int g_DeltaEncoder[4] 	= {0,};
long g_EncoderCnt[4] 	= {0,};
long g_targetEncoder[4] = {-50,};
long g_nowOutput[4] 	= {0,};




void ros_init(void)
{
	nh.initNode();
	nh.advertise(pub_str);
	nh.advertise(pub_imu);
	nh.subscribe(cmdVelSub);

	for(int i = 0; i < 4; i ++)
		properties[i] = pid[i].getProperty();


	  TIM8->CCR1 = 0;
	  TIM8->CCR2 = 0;
	  TIM8->CCR3 = 0;
	  TIM8->CCR4 = 0;
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_4);
	  HAL_Delay(1000);


	  HAL_TIM_Base_Start_IT(&htim6);
	  HAL_TIM_Base_Start_IT(&htim7);
	  HAL_TIM_Base_Start_IT(&htim14);

	  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
}

uint32_t nowTick = 0;
uint32_t pastTick = 0;

void ros_run(void)
{
	__ledState.run();
	__led0.run();
	__led1.run();
	__led2.run();
	__led3.run();

	nowTick = HAL_GetTick();
	if(nowTick - pastTick > 1000)
	{
		str_msg.data = hello;
		imu_msg.orientation.x = 10;
		imu_msg.orientation.y = 105;
		imu_msg.orientation.z = 100;
		pub_str.publish(&str_msg);
		pub_imu.publish(&imu_msg);

		pastTick = nowTick;
	}
	nh.spinOnce();
}

void uart3TxCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->flush();
}
void uart3RxCallbcak(UART_HandleTypeDef *huart)
{
	nh.getHardware()->reset_rbuf();
}



void timer10ms(void)
{
	for(int i = 0; i < 4; i++){
//		pid[i].setGain(properties[i].kP, properties[i].kI, properties[i].kD);
//		pid[i].setErrorSumLimit(properties[i].errorSumLimit);
		g_NowEncoder[i] = *g_MotorEncAddr[i];
		if (g_NowEncoder[i] > 30000)
			g_DeltaEncoder[i] =(long)g_NowEncoder[i] - 65535;
		else
			g_DeltaEncoder[i] = g_NowEncoder[i];
		*g_MotorEncAddr[i] = 0;
		g_nowOutput[i] = pid[i].runPid(g_targetEncoder[i], g_DeltaEncoder[i]);
		g_PastEncoder[i] = g_NowEncoder[i];
		if(g_nowOutput[i] < 0)
		{
			HAL_GPIO_WritePin(g_MotorGpio[i], g_MotorGpioPin[i], GPIO_PIN_RESET);
			*g_MotorPwmAddr[i] = -g_nowOutput[i];
		}
		else
		{
			HAL_GPIO_WritePin(g_MotorGpio[i], g_MotorGpioPin[i], GPIO_PIN_SET);
			*g_MotorPwmAddr[i] = g_nowOutput[i];
		}
	}
}
void timer15us(void)
{

}
void timer1s(void)
{

}

void cmdVelCallback(const geometry_msgs::Twist& msg)
{
	__led0.setPeriod(1000);
}
