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
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		uart3RxCallbcak(huart);
	}
}
