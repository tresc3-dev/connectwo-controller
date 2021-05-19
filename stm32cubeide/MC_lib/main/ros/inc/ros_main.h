/*
 * ros_main.h
 *
 *  Created on: Feb 20, 2021
 *      Author: colson
 */
#ifndef INC_ROS_MAIN_H_
#define INC_ROS_MAIN_H_


#include <main.h>
#include <tim.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

void systemReset();

void ros_init();
void ros_run();

void can_init();


void uart3TxCallback(UART_HandleTypeDef *huart);
void uart3RxCallbcak(UART_HandleTypeDef *huart);
void uart2TxCallback(UART_HandleTypeDef *huart);
void uart2RxCallbcak(UART_HandleTypeDef *huart);
void uart5TxCallback(UART_HandleTypeDef *huart);
void uart5RxCallbcak(UART_HandleTypeDef *huart);
void canRxCallback(CAN_HandleTypeDef *huart);
void timer10ms();
void timer15us();
void timer1s();

int __printf__io__putchar(int ch);
#ifdef __cplusplus
}
#endif
#endif /* INC_ROS_MAIN_H_ */
