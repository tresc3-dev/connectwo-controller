/*
 * stateMachine.c
 *
 *  Created on: Feb 20, 2021
 *      Author: colson
 */

#include "main.h"
#include "time.h"
#include "usart.h"
#include "gpio.h"
#include "ros_main.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14)
	{
		timer15us();
	}
	else if(htim->Instance == TIM7)
	{
		timer10ms();
	}
	else if(htim->Instance == TIM6)
	{
		timer1s();
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		uart3TxCallback(huart);
	}
	else if(huart->Instance == USART2)
	{
		uart2TxCallback(huart);
	}
	else if(huart->Instance == UART5)
	{
		uart5TxCallback(huart);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		uart3RxCallbcak(huart);
	}
	else if(huart->Instance == USART2)
	{
		uart2RxCallbcak(huart);
	}
	else if(huart->Instance == UART5)
	{
		uart5RxCallbcak(huart);
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		canRxCallback(hcan);

	}
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1) {
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1) {

	}
}

