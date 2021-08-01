/*
 * connectwo_config.h
 *
 *  Created on: 2021. 8. 1.
 *      Author: colson
 */

#ifndef CONNECTWO_CONFIG_H_
#define CONNECTWO_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.0037759  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define WHEEL_RADIUS					0.065


void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

void publishImuMsg(void);
void publishMagMsg(void);
void publishSensorStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);
void updateJointStates(void);

void initOdom();
void initJointStates();

bool calcOdometry(double diff_time);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Subscriber
*******************************************************************************/


/*******************************************************************************
* Publisher
*******************************************************************************/

// IMU of Connectwo
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Odometry of Connectwo
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(DNJ) state of Connectwo
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Connectwo
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];


#endif /* CONNECTWO_CONFIG_H_ */
