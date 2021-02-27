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

#include <Motor.h>
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

tresc3::pidProperty<long> pidSetting = { 5000, 100, 0, 0, 0, 0, 0, 0, 0, 200, 0,
		0, 0, 999, 1000 };
long g_targetEncoder[4] = { -50, };

tresc3::Motor<long> motor[4] = { { &htim8, &htim4, (uint32_t) TIM_CHANNEL_4,
		(uint32_t *) &TIM8->CCR4, (uint32_t *) &TIM4->CNT, GPIOB, GPIO_PIN_0,
		pidSetting }, { &htim8, &htim5, (uint32_t) TIM_CHANNEL_3,
		(uint32_t *) &TIM8->CCR3, (uint32_t *) &TIM5->CNT, GPIOC, GPIO_PIN_5,
		pidSetting }, { &htim8, &htim3, (uint32_t) TIM_CHANNEL_2,
		(uint32_t *) &TIM8->CCR2, (uint32_t *) &TIM3->CNT, GPIOB, GPIO_PIN_1,
		pidSetting }, { &htim8, &htim1, (uint32_t) TIM_CHANNEL_1,
		(uint32_t *) &TIM8->CCR1, (uint32_t *) &TIM1->CNT, GPIOB, GPIO_PIN_2,
		pidSetting } };

void ros_init(void) {
	nh.initNode();
	nh.advertise(pub_str);
	nh.advertise(pub_imu);
	nh.subscribe(cmdVelSub);

	for (int i = 0; i < 4; i++)
		motor[i].reset();
	HAL_Delay(1000);
	for (int i = 0; i < 4; i++)
		motor[i].start();

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim14);
}

uint32_t nowTick = 0;
uint32_t pastTick = 0;

void ros_run(void) {
	__ledState.run();
	__led0.run();
	__led1.run();
	__led2.run();
	__led3.run();

	nowTick = HAL_GetTick();
	if (nowTick - pastTick > 500) {
		for (int i = 0; i < 4; i++) {
			g_targetEncoder[i] -= 10;
			if (g_targetEncoder[i] < -120)
				g_targetEncoder[i] = 0;
		}
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

void uart3TxCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void uart3RxCallbcak(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

void timer10ms(void) {
	for (int i = 0; i < 4; i++)
		motor[i].motorControl(g_targetEncoder[i]);
}

void timer15us(void) {

}

void timer1s(void) {

}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
	__led0.setPeriod(1000);
}
