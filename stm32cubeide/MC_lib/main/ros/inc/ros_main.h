/*
 * ros_main.h
 *
 *  Created on: Feb 20, 2021
 *      Author: colson
 */
#ifndef INC_ROS_MAIN_H_
#define INC_ROS_MAIN_H_
#include <main.h>

#ifdef __cplusplus
extern "C" {
#endif


void ros_init(void);
void ros_run(void);

void uart3TxCallback(UART_HandleTypeDef *huart);
void uart3RxCallbcak(UART_HandleTypeDef *huart);
void timer10ms(void);
void timer15us(void);
void timer1s(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_ROS_MAIN_H_ */
