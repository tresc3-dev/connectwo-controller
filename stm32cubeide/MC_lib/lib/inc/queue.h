/*
 * queue.h
 *
 *  Created on: Jun 22, 2019
 *      Author: colson
 */

#ifndef QUEUE_H_
#define QUEUE_H_


#ifdef __cplusplus
extern "C"{
#endif


#include "main.h"

#ifndef BOOL
#define BOOL int
#endif

#define UART_RX_MAX_QUEUE 256
#define UART_TX_MAX_QUEUE 256
#define CAN_RX_MAX_QUEUE 30
#define CAN_TX_MAX_QUEUE 30


#define true 1
#define false 0

typedef struct _u_queue {
	uint8_t queue[UART_RX_MAX_QUEUE];
	int front;
	int rear;
	int count;
	int max_count;
} QUEUE_ST;

typedef struct _st_machine_queue {

} QUEUE_MACHINE;

/*
 * for uint8_t arr
 *
 */

void u8_init_queue(QUEUE_ST *str);
BOOL u8_is_empty(QUEUE_ST * str);
BOOL u8_is_full(QUEUE_ST * str);
BOOL u8_enqueue(QUEUE_ST *str, uint8_t value);
BOOL u8_dequeue(QUEUE_ST *str, uint8_t* value);




#ifdef __cplusplus
}
#endif
#endif /* QUEUE_H_ */
