/*
 * queue.c
 *
 *  Created on: Jun 22, 2019
 *      Author: colson
 */

#include "queue.h"



void u8_init_queue(QUEUE_ST *str)
{
	str->front = 0;
	str->rear = 0;
	str->count = 0;
	str->max_count = 0;
}

BOOL u8_is_empty(QUEUE_ST * str) {
	return str->front == str->rear ? true : false;
}

BOOL u8_is_full(QUEUE_ST * str) {
	return (str->rear + 1) % UART_RX_MAX_QUEUE == str->front ? true : false;
}

BOOL u8_enqueue(QUEUE_ST *str, uint8_t value)
{
	if (!u8_is_full(str))
	{
		int preIndex = str->rear;
		str->rear = (str->rear + 1) % UART_RX_MAX_QUEUE;
		str->queue[preIndex] = value;
		str->count++;
		if (str->count > str->max_count)
			str->max_count = str->count;

		return true;
	}
	return false;
}

BOOL u8_dequeue(QUEUE_ST *str, uint8_t* value)
{
	if (!str || !value)
	{
		return false;
	}

	if (!u8_is_empty(str))
	{
		int preIndex = str->front;
		str->front = (str->front + 1) % UART_RX_MAX_QUEUE;
		str->count--;

		*value = str->queue[preIndex];

		return true;
	}

	return false;
}
