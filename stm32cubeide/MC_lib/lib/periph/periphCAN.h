/*
 * hardware.h
 *
 *  Created on: Jun 22, 2019
 *      Author: colson
 */

#ifndef INC_PERIPHCAN_H_
#define INC_PERIPHCAN_H_

#include "main.h"

typedef struct _CAN_Handler {
	uint32_t flags;
	CAN_FilterTypeDef filter;
	CAN_TxHeaderTypeDef txHeader;
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t txData[8];
	uint8_t rxData[8];
} CAN_Handler_t;

class PeriphCAN {
public:
	uint32_t canTxMailBox;

protected:
	CAN_HandleTypeDef *hcan;
	CAN_Handler_t handle;
	uint32_t notification;

public:
	PeriphCAN() {

	}
	PeriphCAN(CAN_HandleTypeDef *_hcan, CAN_Handler_t _handle,
			uint32_t _notification = CAN_IT_TX_MAILBOX_EMPTY
					| CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL) :
			hcan(_hcan), handle(_handle), notification(_notification) {
	}
	~PeriphCAN() {

	}

	void periphCanInit(void)
	{
		filterInit(handle.filter);
		notificationInit(notification);
		start();
	}

	void filterInit(CAN_FilterTypeDef type) {
		if (HAL_CAN_ConfigFilter(hcan, &type) != HAL_OK)
			Error_Handler();
	}
	void notificationInit(uint32_t flags) {
		if (HAL_CAN_ActivateNotification(hcan, flags) != HAL_OK)
			Error_Handler();
	}
	void start(void) {
		if (HAL_CAN_Start(hcan) != HAL_OK)
			Error_Handler();
	}
protected:
	void transmitData(CAN_TxHeaderTypeDef *hedder, uint8_t *txData) {
		if (HAL_CAN_AddTxMessage(hcan, hedder, txData, &canTxMailBox) != HAL_OK)
			Error_Handler();
	}
};

#endif /* INC_PERIPHUSART_H_ */
