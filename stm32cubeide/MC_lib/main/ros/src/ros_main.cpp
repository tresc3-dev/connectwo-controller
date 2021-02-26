/*
 * ros_main.cpp
 *
 *  Created on: Feb 20, 2021
 *      Author: colson
 */

#include <ros_main.h>
#include <periphGpio.h>

#include "ros.h"
#include "ros/time.h"

#include "std_msgs/String.h"

ros::NodeHandle nh;

std_msgs::String str_msg;

ros::Publisher pub_str("/tresc3/chatter", &str_msg);

char hello[] = "hello world!";


PeriphGPIO __led0(LED0_GPIO_Port, LED0_Pin, 100);
PeriphGPIO __led1(LED1_GPIO_Port, LED1_Pin, 100);
PeriphGPIO __led2(LED2_GPIO_Port, LED2_Pin, 100);
PeriphGPIO __led3(LED3_GPIO_Port, LED3_Pin, 100);








void ros_init(void)
{

	nh.initNode();
	nh.advertise(pub_str);
	TIM8->CCR1 = 0;
	TIM8->CCR2 = 0;
	TIM8->CCR3 = 0;
	TIM8->CCR4 = 0;


}

uint32_t nowTick = 0;
uint32_t pastTick = 0;
void ros_run(void)
{
	__led0.run();
	__led1.run();
	__led2.run();
	__led3.run();

	nowTick = HAL_GetTick();
	if(nowTick - pastTick > 1000)
	{
		TIM8->CCR1 = 100;
		TIM8->CCR2 = 200;
		TIM8->CCR3 = 300;
		TIM8->CCR4 = 400;
		str_msg.data = hello;
		pub_str.publish(&str_msg);

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

uint16_t gNowEncoder[4] = {0,};
uint16_t gPastEncoder[4] = {0,};
int gDeltaEncoder[4] = {0,};
long pEncoderCnt[4] = {0,4};
void timer10ms(void)
{
	gNowEncoder[0] = TIM4->CNT;
	gNowEncoder[1] = TIM5->CNT;
	gNowEncoder[2] = TIM3->CNT;
	gNowEncoder[3] = TIM1->CNT;

	for(int i = 0; i < 4; i++)
	{
		gDeltaEncoder[i] = gNowEncoder[i] - gPastEncoder[i];
	}




	for(int i = 0; i < 4; i++)
	{
		gPastEncoder[i] = gNowEncoder[i];
	}
}
void timer15us(void)
{

}
void timer1s(void)
{

}

