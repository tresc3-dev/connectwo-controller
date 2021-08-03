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
#include <connectwo_config.h>
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
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"


#include "StateMachine.h"

#define ROS_MODE 1
#define ARDUINO_MODE 2

int nowMode = 0;

extern CAN_HandleTypeDef hcan1;

std_msgs::String str_msg;

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

pidProperty<long> pidSetting = { 10000, 100, 0, 0, 0, 0, 0, 0, 0, 200, 0, 0, 0,
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
		printf("Arduino mode init complete.\r\n");
    }
    else
    {
    	nowMode = ROS_MODE;
        nh.initNode();
        nh.advertise(pub_str);
//        nh.advertise(pub_imu);
        nh.advertise(imu_pub);
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

        initOdom();
        initJointStates();
        tf_broadcaster.init(nh);
		printf("ROS mode init complete.\r\n");
    }
    machine.resetPacket();
    __usart3.init();
    __usart5.init();
	printf("Init process complete.\r\n");
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

    updateVariable(nh.connected());
    updateTFPrefix(nh.connected());

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
    			printf("Complete machine.\r\n");
    			tresc3::packet result = machine.getPacket();
    			if(result.cmd == 3)
    			{
    				printf("Incoming task: set cmd_vel data.\r\n");
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
    				printf("Complete led setup.\r\n");
    			}
    		}
    	}


		nowTick[chat_index] = HAL_GetTick();
		if(nowTick[chat_index] - pastTick[chat_index] > 100) {
    		printf("Make imu packet(100ms).\n\r");
			pastTick[chat_index] = nowTick[chat_index];


			uint8_t makeData[255] = {0,};
			int size = makeImuPacket(2, makeData, __imu.data.e_roll, __imu.data.e_pitch, __imu.data.e_yaw);
    		printf("Complete make imu packet.\n\r");
			__usart3.write(makeData, size);
    		printf("Send packet __uart3(Arduino).\n\r");
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
//			imu_msg.orientation.x = __imu.data.e_roll;
//			imu_msg.orientation.y = __imu.data.e_pitch;
//			imu_msg.orientation.z = __imu.data.e_yaw;
//			pub_imu.publish(&imu_msg);
			publishImuMsg();
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
	printf("imu_x %d \n\r", (int)(__imu.data.e_roll * 1000));
	printf("imu_y %d \n\r", (int)(__imu.data.e_pitch * 1000));
	printf("imu_z %d \n\r", (int)(__imu.data.e_yaw * 1000));
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

void initOdom()
{
	init_encoder = true;

	for (int index = 0; index < 3; index++)
	{
		odom_pose[index] = 0.0;
		odom_vel[index] = 0.0;
	}

	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;

	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 0.0;

	odom.twist.twist.linear.x = 0.0;
	odom.twist.twist.angular.z = 0.0;
}
void initJointStates()
{
	  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

	  joint_states.header.frame_id = joint_state_header_frame_id;
	  joint_states.name = joint_states_name;

	  joint_states.name_length = WHEEL_NUM;
	  joint_states.position_length = WHEEL_NUM;
	  joint_states.velocity_length = WHEEL_NUM;
	  joint_states.effort_length = WHEEL_NUM;
}



void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{

}
void resetCallback(const std_msgs::Empty& reset_msg)
{

}

void publishImuMsg(void)
{
	imu_msg.header.stamp = rosNow();
	imu_msg.header.frame_id = imu_frame_id;
	imu_msg.orientation = tf::createQuaternionFromYaw(__imu.data.e_yaw);

	imu_msg.angular_velocity.x = __imu.data.g_x;
	imu_msg.angular_velocity.y = __imu.data.g_y;
	imu_msg.angular_velocity.z = __imu.data.g_z;

	imu_msg.linear_acceleration.x = __imu.data.a_x;
	imu_msg.linear_acceleration.y = __imu.data.a_y;
	imu_msg.linear_acceleration.z = __imu.data.a_z;

	imu_pub.publish(&imu_msg);
}
void publishMagMsg(void)
{

}
void publishSensorStateMsg(void)
{

}
void publishDriveInformation(void)
{
	unsigned long time_now = HAL_GetTick();
	unsigned long step_time = time_now - prev_update_time;

	prev_update_time = time_now;
    ros::Time stamp_now = rosNow();

	// calculate odometry
	calcOdometry((double)(step_time * 0.001));

	// odometry
	updateOdometry();
	odom.header.stamp = stamp_now;
	odom_pub.publish(&odom);

	// odometry tf
	updateTF(odom_tf);
	odom_tf.header.stamp = stamp_now;
	tf_broadcaster.sendTransform(odom_tf);

	// joint state
	updateJointStates();
	joint_states.header.stamp =stamp_now;
	joint_states_pub.publish(&joint_states);
}

ros::Time rosNow(void)
{
	  return nh.now();
}
ros::Time addMicros(ros::Time & t, uint32_t _micros) // deprecated
{
	  uint32_t sec, nsec;

	  sec  = _micros / 1000 + t.sec;
	  nsec = _micros % 1000000000 + t.nsec;

	  return ros::Time(sec, nsec);
}

void updateVariable(bool isConnected)
{
	static bool variable_flag = false;

	if (isConnected)
	{
		if (variable_flag == false)
		{
			initOdom();
			variable_flag = true;
		}
	}
	else
	{
		variable_flag = false;
	}
}
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
	int32_t current_tick = 0;
	static int32_t last_tick[WHEEL_NUM] = {0, 0};

	if (init_encoder)
	{
		for (int index = 0; index < WHEEL_NUM; index++)
		{
		  last_diff_tick[index] = 0;
		  last_tick[index]      = 0;
		  last_rad[index]       = 0.0;

		  last_velocity[index]  = 0.0;
		}

		last_tick[LEFT] = left_tick;
		last_tick[RIGHT] = right_tick;

		init_encoder = false;
		return;
	}

	current_tick = left_tick;

	last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
	last_tick[LEFT]      = current_tick;
	last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

	current_tick = right_tick;

	last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
	last_tick[RIGHT]      = current_tick;
	last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}
void updateTime(void)
{
	current_offset = HAL_GetTick();
	current_time = nh.now();
}
void updateOdometry(void)
{
	  odom.header.frame_id = odom_header_frame_id;
	  odom.child_frame_id  = odom_child_frame_id;

	  odom.pose.pose.position.x = odom_pose[0];
	  odom.pose.pose.position.y = odom_pose[1];
	  odom.pose.pose.position.z = 0;
	  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

	  odom.twist.twist.linear.x  = odom_vel[0];
	  odom.twist.twist.angular.z = odom_vel[2];
}
void updateJoint(void)
{

}
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
	odom_tf.header = odom.header;
	odom_tf.child_frame_id = odom.child_frame_id;
	odom_tf.transform.translation.x = odom.pose.pose.position.x;
	odom_tf.transform.translation.y = odom.pose.pose.position.y;
	odom_tf.transform.translation.z = odom.pose.pose.position.z;
	odom_tf.transform.rotation      = odom.pose.pose.orientation;
}
void updateGoalVelocity(void)
{

}
void updateTFPrefix(bool isConnected)
{
	static bool isChecked = false;
	char log_msg[50];

	if (isConnected)
	{
		if (isChecked == false)
		{
			nh.getParam("~tf_prefix", &get_tf_prefix);

			if(!strcmp(get_tf_prefix, ""))
			{
				sprintf(odom_header_frame_id, "odom");
				sprintf(odom_child_frame_id, "base_footprint");

				sprintf(imu_frame_id, "imu_link");
				sprintf(mag_frame_id, "mag_link");
				sprintf(joint_state_header_frame_id, "base_link");
			}
			else
			{
		        strcpy(odom_header_frame_id, get_tf_prefix);
		        strcpy(odom_child_frame_id, get_tf_prefix);

		        strcpy(imu_frame_id, get_tf_prefix);
		        strcpy(mag_frame_id, get_tf_prefix);
		        strcpy(joint_state_header_frame_id, get_tf_prefix);

		        strcat(odom_header_frame_id, "/odom");
		        strcat(odom_child_frame_id, "/base_footprint");

		        strcat(imu_frame_id, "/imu_link");
		        strcat(mag_frame_id, "/mag_link");
		        strcat(joint_state_header_frame_id, "/base_link");
			}

			sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
			nh.loginfo(log_msg);

			sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
			nh.loginfo(log_msg);

			sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
			nh.loginfo(log_msg);

			sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
			nh.loginfo(log_msg);

			isChecked = true;
		}
	}
	else
	{
		isChecked = false;
	}
}

void updateJointStates(void)
{
	  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
	  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
	  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

	  joint_states_pos[LEFT]  = last_rad[LEFT];
	  joint_states_pos[RIGHT] = last_rad[RIGHT];

	  joint_states_vel[LEFT]  = last_velocity[LEFT];
	  joint_states_vel[RIGHT] = last_velocity[RIGHT];

	  joint_states.position = joint_states_pos;
	  joint_states.velocity = joint_states_vel;
}

bool calcOdometry(double diff_time)
{
	float orientation[4];
	double wheel_l, wheel_r;
	double delta_s, theta, delta_theta;
	static double last_theta = 0.0;
	double v, w;
	double step_time;

	wheel_l = wheel_r = 0.0;
	delta_s = delta_theta = theta = 0.0;
	v= w = 0;
	step_time = 0.0;

	step_time = diff_time;

	if (step_time == 0)
		return false;

	wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
	wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

	if(isnan(wheel_l))
		wheel_l = 0.0;
	if(isnan(wheel_r))
		wheel_r = 0.0;

	delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
	theta = __imu.data.e_yaw;

	delta_theta = theta - last_theta;

	// compute odometric pose
	odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
	odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
	odom_pose[2] += delta_theta;

	// compute odometric instantaneouse velocity

	v = delta_s / step_time;
	w = delta_theta / step_time;

	odom_vel[0] = v;
	odom_vel[1] = 0.0;
	odom_vel[2] = w;

	last_velocity[LEFT]  = wheel_l / step_time;
	last_velocity[RIGHT] = wheel_r / step_time;
	last_theta = theta;

	return true;
}


