/*
 * data_struct.h
 *
 *  Created on: Jun 21, 2019
 *      Author: colson
 */

#ifndef DATA_STRUCT_H_
#define DATA_STRUCT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct _counters {
	uint32_t usart1TxCounter;
	uint32_t usart1RxCounter;
	uint32_t usart2TxCounter;
	uint32_t usart2RxCounter;
	uint32_t usart3TxCounter;
	uint32_t usart3RxCounter;
	uint32_t usart4TxCounter;
	uint32_t usart4RxCounter;
	uint32_t usart5TxCounter;
	uint32_t usart5RxCounter;
	uint32_t usart6TxCounter;
	uint32_t usart6RxCounter;

	uint32_t usart1ErrorCounter;
	uint32_t usart2ErrorCounter;
	uint32_t usart3ErrorCounter;
	uint32_t usart4ErrorCounter;
	uint32_t usart5ErrorCounter;
	uint32_t usart6ErrorCounter;

	uint32_t can1TxCounter;
	uint32_t can1RxCounter;
	uint32_t can1RxFullCounter;
	uint32_t can1ErrorCounter;

	uint32_t stateMachineCpltCounter[3];
} COUNTERS;

typedef struct _flags{

}FLAGS;



#ifdef __cplusplus
}
#endif
#endif /* DATA_STRUCT_H_ */
