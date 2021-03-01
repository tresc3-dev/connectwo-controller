/*
 * ros_main.cpp
 *
 *  Created on: Feb 20, 2021
 *      Author: colson
 */

#include <Pid.h>
#include <ros_main.h>
#include <periphGpio.h>
#include <periphCAN.h>
//#include <MW-AHRSv1.h>
#include <Ahrsv1.h>

#include "ros.h"
#include "ros/time.h"

#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <Motor.h>
#include <Nonholonomic.h>

extern CAN_HandleTypeDef hcan1;

ros::NodeHandle nh;

std_msgs::String str_msg;
sensor_msgs::Imu imu_msg;

ros::Publisher pub_str("/tresc3/chatter", &str_msg);
ros::Publisher pub_imu("/imu", &imu_msg);

char hello[] = "hello world!";

void cmdVelCallback(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

//can_tx_hedder.StdId = 0x01;
//can_tx_hedder.ExtId = 0x01;
//can_tx_hedder.RTR = CAN_RTR_DATA;
//can_tx_hedder.IDE = CAN_ID_STD;
//can_tx_hedder.DLC = 8;
PeriphGPIO __ledState(STATE_LED_GPIO_Port, STATE_LED_Pin, 1000);
PeriphGPIO __led0(LED0_GPIO_Port, LED0_Pin, 100);
PeriphGPIO __led1(LED1_GPIO_Port, LED1_Pin, 100);
PeriphGPIO __led2(LED2_GPIO_Port, LED2_Pin, 100);
PeriphGPIO __led3(LED3_GPIO_Port, LED3_Pin, 100);
extern CAN_HandleTypeDef hcan1;
Ahrsv1 __imu(&hcan1,
		(CAN_Handler_t ) { (uint32_t) 0, (CAN_FilterTypeDef ) { 0x0000
								<< 5, 0x0000, 0x0000 << 5, 0x0000,
						CAN_FILTER_FIFO0, 0,
						CAN_FILTERMODE_IDMASK,
						CAN_FILTERSCALE_32BIT,
						CAN_FILTER_ENABLE, 0 }, (CAN_TxHeaderTypeDef ) { 0x01,

								0x01, CAN_ID_STD, CAN_RTR_DATA, 8,
										(FunctionalState) 0 },
						(CAN_RxHeaderTypeDef ) { 0, }, (uint8_t ) { 0, },
						(uint8_t ) { 0, } });

pidProperty<long> pidSetting = { 5000, 100, 0, 0, 0, 0, 0, 0, 0, 200, 0,
		0, 0, 999, 1000 };

Motor<long> motor[4] = { { &htim8, &htim4, (uint32_t) TIM_CHANNEL_4,
		(uint32_t *) &TIM8->CCR4, (uint32_t *) &TIM4->CNT, GPIOB, GPIO_PIN_0,
		pidSetting }, { &htim8, &htim5, (uint32_t) TIM_CHANNEL_3,
		(uint32_t *) &TIM8->CCR3, (uint32_t *) &TIM5->CNT, GPIOC, GPIO_PIN_5,
		pidSetting }, { &htim8, &htim3, (uint32_t) TIM_CHANNEL_2,
		(uint32_t *) &TIM8->CCR2, (uint32_t *) &TIM3->CNT, GPIOB, GPIO_PIN_1,
		pidSetting }, { &htim8, &htim1, (uint32_t) TIM_CHANNEL_1,
		(uint32_t *) &TIM8->CCR1, (uint32_t *) &TIM1->CNT, GPIOB, GPIO_PIN_2,
		pidSetting } };

Nonholonomic dynamics(0.13, 0.125, 1664, 0.02);

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
	HAL_Delay(1000);

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim14);

	__imu.init();
	__imu.setDataType(1, 1, 1, 1);
	__imu.setPeriod(10);

}

const int imu_index = 0;
const int chat_index = 1;
uint32_t nowTick[10] = { 0, };
uint32_t pastTick[10] = { 0, };

void ros_run(void) {
	__ledState.run();
	__led0.run();
	__led1.run();
	__led2.run();
	__led3.run();

	nowTick[chat_index] = HAL_GetTick();
	if (nowTick[chat_index] - pastTick[chat_index] > 500) {
		str_msg.data = hello;
		pub_str.publish(&str_msg);
		pastTick[chat_index] = nowTick[chat_index];
	}
	nowTick[imu_index] = HAL_GetTick();
	if (nowTick[imu_index] - pastTick[imu_index] > 100) {
		imu_msg.orientation.x = __imu.data.e_roll;
		imu_msg.orientation.y = __imu.data.e_pitch;
		imu_msg.orientation.z = __imu.data.e_yaw;
		pub_imu.publish(&imu_msg);
		pastTick[imu_index] = nowTick[imu_index];
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
		motor[i].motorControl(50);
}

void timer15us(void) {

}

void timer1s(void) {

}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
	__led0.setPeriod(1000);
	auto ret = dynamics.calc(msg.linear.x, msg.angular.z);
	long left_v = static_cast<long>(ret.leftValue);
	long right_v = static_cast<long>(ret.rightValue);
	motor[0].motorControl(left_v);
	motor[1].motorControl(left_v);
	motor[2].motorControl(right_v);
	motor[3].motorControl(right_v);
}

void canRxCallback(CAN_HandleTypeDef *huart) {
	uint8_t* rxData = __imu.getRxDataAddr();
	CAN_RxHeaderTypeDef* rxHeader = __imu.getRxHedderAddr();
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, rxHeader, rxData);
	for (int i = 0; i < 8; i++)
		__imu.data.can_read_data[i] = rxData[i];
	mw_ahrs_input_data(&__imu.data);
}
































// now not used
MW_AHRS ahrs_obj = { 0, };
CAN_TxHeaderTypeDef can_tx_hedder = { 0, };
CAN_RxHeaderTypeDef can_rx_hedder = { 0, };
uint8_t can_tx_data[8] = { 0, };
uint8_t can_rx_data[8] = { 0, };
void can_init(void) {
	CAN_FilterTypeDef canFilter;
	canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilter.FilterIdHigh = 0x0000 << 5;
	canFilter.FilterIdLow = 0x0000;
	canFilter.FilterMaskIdHigh = 0x0000 << 5;
	canFilter.FilterMaskIdLow = 0x0000;
	canFilter.SlaveStartFilterBank = 0;
	canFilter.FilterBank = 0;
	canFilter.FilterActivation = CAN_FILTER_ENABLE;

	mw_ahrs_set_period(&ahrs_obj, 10);
	mw_ahrs_set_data_type(&ahrs_obj, 1, 1, 1, 1);
	can_tx_hedder.StdId = 0x01;
	can_tx_hedder.ExtId = 0x01;
	can_tx_hedder.RTR = CAN_RTR_DATA;
	can_tx_hedder.IDE = CAN_ID_STD;
	can_tx_hedder.DLC = 8;

	for (int i = 0; i < 8; i++) {
		can_tx_data[i] = ahrs_obj.can_write_data[i];
	}

	HAL_CAN_ConfigFilter(&hcan1, &canFilter);
	HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING
					| CAN_IT_RX_FIFO0_FULL);
	HAL_CAN_Start(&hcan1);
	uint32_t canTxMailbox;
	HAL_CAN_AddTxMessage(&hcan1, &can_tx_hedder, can_tx_data, &canTxMailbox);
}
