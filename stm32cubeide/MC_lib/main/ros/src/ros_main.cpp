/*
 * ros_main.cpp
 *
 *  Created on: Feb 20, 2021
 *      Author: colson
 */

#include <list>
#include <algorithm>
#include <array>

#include <Pid.h>
#include <ros_main.h>
#include <periphGpio.h>
#include <periphCAN.h>
#include <periphusart.h>

#include <Ahrsv1.h>

#include <Motor.h>
#include <Nonholonomic.h>

#include "ros.h"
#include "ros/time.h"

#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include "StateMachine.h"

#define ROS_MODE 1
#define ARDUINO_MODE 2

int nowMode = 0;

extern CAN_HandleTypeDef hcan1;

ros::NodeHandle nh;

std_msgs::String str_msg;
sensor_msgs::Imu imu_msg;

ros::Publisher pub_str("/tresc3/chatter", &str_msg);
ros::Publisher pub_imu("/imu", &imu_msg);

char hello[] = "hello world!";

void
cmdVelCallback(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

PeriphUsart __usart2(&huart2);
PeriphUsart __usart3(&huart3);
PeriphUsart __usart5(&huart5);

PeriphGPIO __ledState(STATE_LED_GPIO_Port, STATE_LED_Pin, 1000);
PeriphGPIO __led0(LED0_GPIO_Port, LED0_Pin, 100);
PeriphGPIO __led1(LED1_GPIO_Port, LED1_Pin, 100);
PeriphGPIO __led2(LED2_GPIO_Port, LED2_Pin, 100);
PeriphGPIO __led3(LED3_GPIO_Port, LED3_Pin, 100);

PeriphGPIO __id0(ID0_GPIO_Port, ID0_Pin);
PeriphGPIO __id1(ID1_GPIO_Port, ID1_Pin);
PeriphGPIO __id2(ID2_GPIO_Port, ID2_Pin);
PeriphGPIO __id3(ID3_GPIO_Port, ID3_Pin);

std::list<PeriphGPIO> __ids{PeriphGPIO(ID0_GPIO_Port, ID0_Pin), PeriphGPIO(ID1_GPIO_Port, ID1_Pin), PeriphGPIO(ID2_GPIO_Port, ID2_Pin), PeriphGPIO(ID3_GPIO_Port, ID3_Pin)};

extern CAN_HandleTypeDef hcan1;
Ahrsv1 __imu(&hcan1, (CAN_Handler_t )
        { (uint32_t) 0, (CAN_FilterTypeDef )
                { 0x0000 << 5, 0x0000, 0x0000 << 5, 0x0000,
                CAN_FILTER_FIFO0, 0, CAN_FILTERMODE_IDMASK,
                CAN_FILTERSCALE_32BIT, CAN_FILTER_ENABLE, 0 },
                (CAN_TxHeaderTypeDef )
                        { 0x01, 0x01, CAN_ID_STD,
                        CAN_RTR_DATA, 8, (FunctionalState) 0 },
                (CAN_RxHeaderTypeDef )
                        { 0, }, (uint8_t )
                                { 0, }, (uint8_t )
                                        { 0, } });

pidProperty<long> pidSetting = { 5000, 100, 0, 0, 0, 0, 0, 0, 0, 200, 0, 0, 0,
        999, 1000 };

Motor<long> motor[4] = { { &htim8, &htim4, (uint32_t) TIM_CHANNEL_4,
        (uint32_t *) &TIM8->CCR4, (uint32_t *) &TIM4->CNT, GPIOB, GPIO_PIN_0,
        pidSetting }, { &htim8, &htim5, (uint32_t) TIM_CHANNEL_3,
        (uint32_t *) &TIM8->CCR3, (uint32_t *) &TIM5->CNT, GPIOC, GPIO_PIN_5,
        pidSetting }, { &htim8, &htim3, (uint32_t) TIM_CHANNEL_2,
        (uint32_t *) &TIM8->CCR2, (uint32_t *) &TIM3->CNT, GPIOB, GPIO_PIN_1,
        pidSetting }, { &htim8, &htim1, (uint32_t) TIM_CHANNEL_1,
        (uint32_t *) &TIM8->CCR1, (uint32_t *) &TIM1->CNT, GPIOB, GPIO_PIN_2,
        pidSetting } };

Nonholonomic dynamics(0.065, 0.125, 1664, 0.02);

tresc3::StateMachine machine;

void systemReset() {
    HAL_NVIC_SystemReset();
}

const int imu_index = 0;
const int chat_index = 1;
uint32_t nowTick[10] = { 0, };
uint32_t pastTick[10] = { 0, };

long target_l = 0;
long target_r = 0;


std::array<int, 4> read_ids()
{
	std::array<int, 4> state;
	int i = 0;
	std::for_each(__ids.begin(), __ids.end(), [&](auto &periphGpio){
		state[i++] = periphGpio.read();
	});
	return state;
}

void ros_init(void) {
    auto idState = read_ids();
    /* Check arduino state */
    if(idState[0])
    {
    	nowMode = ARDUINO_MODE;

        HAL_TIM_Base_Start_IT(&htim6);
        HAL_TIM_Base_Start_IT(&htim7);
        HAL_TIM_Base_Start_IT(&htim14);

        for(int i = 0; i < 4; i++) {
            motor[i].reset();
            motor[i].start();
        }
        HAL_Delay(4000);

        __imu.init();
        __imu.setDataType(1, 1, 1, 1);
        __imu.setPeriod(10);
    }
    else
    {
    	nowMode = ROS_MODE;
        nh.initNode();
        nh.advertise(pub_str);
        nh.advertise(pub_imu);
        nh.subscribe(cmdVelSub);

        for(int i = 0; i < 4; i++) {
            motor[i].reset();
            motor[i].start();
        }
        HAL_Delay(4000);

        HAL_TIM_Base_Start_IT(&htim6);
        HAL_TIM_Base_Start_IT(&htim7);
        HAL_TIM_Base_Start_IT(&htim14);

        __imu.init();
        __imu.setDataType(1, 1, 1, 1);
        __imu.setPeriod(10);

        auto ret = dynamics.calc(0, 5.0);
        target_l = static_cast<long>(ret.leftValue);
        target_r = -static_cast<long>(ret.rightValue);
        __led0.setPeriod(target_l);
        __led1.setPeriod(target_l);
        __led2.setPeriod(target_r);
        __led3.setPeriod(target_r);
    }
    machine.resetPacket();
    __usart3.init();
    __usart5.init();
}


uint8_t tempData[100];
uint8_t cnt = 0;

uint32_t makeImuPacket(uint16_t cmd, uint8_t *target, double roll, double pitch, double yaw)
{
	int rollData = (int)(roll * 1000);
	int pitchData = (int)(pitch * 1000);
	int yawData = (int)(yaw * 1000);

	uint8_t checkSum = 0;
	uint16_t length = 12;

	target[0] = 0xFF;
	target[1] = 0xFF;
	target[2] = (uint8_t)cmd;
	target[3] = (uint8_t)(cmd >> 8);
	target[4] = (uint8_t)length;
	target[5] = (uint8_t)(length >> 8);

	target[6] = (uint8_t)rollData;
	target[7] = (uint8_t)(rollData >> 8);
	target[8] = (uint8_t)(rollData >> 16);
	target[9] = (uint8_t)(rollData >> 24);

	target[10] = (uint8_t)pitchData;
	target[11] = (uint8_t)(pitchData >> 8);
	target[12] = (uint8_t)(pitchData >> 16);
	target[13] = (uint8_t)(pitchData >> 24);

	target[14] = (uint8_t)yawData;
	target[15] = (uint8_t)(yawData >> 8);
	target[16] = (uint8_t)(yawData >> 16);
	target[17] = (uint8_t)(yawData >> 24);

	for(int i = 6; i < 18; i ++)
	{
		checkSum += target[i];
	}
	target[18] = checkSum;

	return 19;
}

void ros_run(void) {
    __ledState.run();
    __led0.run();
    __led1.run();
    __led2.run();
    __led3.run();

    if(nowMode == ARDUINO_MODE)
    {
    	while(true)
    	{
    		int data = __usart3.read();
    		if(data == -1)
    		{
    			break;
    		}
    		if(machine.run(data))
    		{
    			printf("machine complete\r\n");
    			tresc3::packet result = machine.getPacket();
    			if(result.cmd == 3)
    			{
    				int linearX = result.data[0];
    				linearX |= result.data[1] << 8;
    				linearX |= result.data[2] << 16;
    				linearX |= result.data[3] << 24;
    				int angularZ = result.data[20];
    				angularZ |= result.data[21] << 8;
    				angularZ |= result.data[22] << 16;
    				angularZ |= result.data[23] << 24;

    				double dlinearX = (double)linearX / 1000.0;
    				double dAngularZ = (double)angularZ / 1000.0;


    		        auto ret = dynamics.calc(dlinearX, dAngularZ);
    		        target_l = static_cast<long>(ret.leftValue);
    		        target_r = -static_cast<long>(ret.rightValue);
    		        __led0.setPeriod(target_l);
    		        __led1.setPeriod(target_l);
    		        __led2.setPeriod(target_r);
    		        __led3.setPeriod(target_r);
    			}
    		}
    	}


		nowTick[chat_index] = HAL_GetTick();
		if(nowTick[chat_index] - pastTick[chat_index] > 100) {
    		printf("Arduino mode chat \n\r");
			pastTick[chat_index] = nowTick[chat_index];


			uint8_t makeData[255] = {0,};
			int size = makeImuPacket(2, makeData, __imu.data.e_roll, __imu.data.e_pitch, __imu.data.e_yaw);
			__usart3.write(makeData, size);
		}
    }
    else if(nowMode == ROS_MODE)
    {
		nowTick[chat_index] = HAL_GetTick();
		if(nowTick[chat_index] - pastTick[chat_index] > 500) {
			str_msg.data = hello;
			pub_str.publish(&str_msg);
			pastTick[chat_index] = nowTick[chat_index];
		}
		nowTick[imu_index] = HAL_GetTick();
		if(nowTick[imu_index] - pastTick[imu_index] > 100) {
			imu_msg.orientation.x = __imu.data.e_roll;
			imu_msg.orientation.y = __imu.data.e_pitch;
			imu_msg.orientation.z = __imu.data.e_yaw;
			pub_imu.publish(&imu_msg);
			pastTick[imu_index] = nowTick[imu_index];
		}
		nh.spinOnce();
    }
}

void uart3TxCallback(UART_HandleTypeDef *huart) {
//	nh.getHardware()->flush();
    __usart3.flush();
}

void uart3RxCallbcak(UART_HandleTypeDef *huart) {
//	nh.getHardware()->reset_rbuf();
    __usart3.reset_rbuf();
}

void uart2TxCallback(UART_HandleTypeDef *huart) {
    nh.getHardware()->flush();
}

void uart2RxCallbcak(UART_HandleTypeDef *huart) {
    nh.getHardware()->reset_rbuf();
}

void uart5TxCallback(UART_HandleTypeDef *huart) {
	__usart5.flush();
}
void uart5RxCallbcak(UART_HandleTypeDef *huart) {
	__usart5.reset_rbuf();
}

void timer10ms(void) {
    for(int i = 0; i < 4; i++) {
        motor[0].motorControl(target_l);
        motor[1].motorControl(target_l);
        motor[2].motorControl(target_r);
        motor[3].motorControl(target_r);
    }
}

void timer15us(void) {

}

void timer1s(void) {
	printf("timer1s \n\r");
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
    auto ret = dynamics.calc(msg.linear.x, msg.angular.z);
    target_l = static_cast<long>(ret.leftValue);
    target_r = -static_cast<long>(ret.rightValue);
    printf("hello? \n\r");
    printf("target_l: %d\n\r", static_cast<int>(target_l));
    printf("target_r: %d\n\r", static_cast<int>(target_r));

    __led0.setPeriod(target_l);
    __led1.setPeriod(target_l);
    __led2.setPeriod(target_r);
    __led3.setPeriod(target_r);
}

void canRxCallback(CAN_HandleTypeDef *huart) {
    uint8_t* rxData = __imu.getRxDataAddr();
    CAN_RxHeaderTypeDef* rxHeader = __imu.getRxHedderAddr();
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, rxHeader, rxData);
    for(int i = 0; i < 8; i++)
        __imu.data.can_read_data[i] = rxData[i];
    mw_ahrs_input_data(&__imu.data);
}

int __printf__io__putchar(int ch) {
    uint8_t data = ch;

//	TODO change MAX485 or CAN line
    __usart5.write(&data, 1);

    return ch;
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

    for(int i = 0; i < 8; i++) {
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
