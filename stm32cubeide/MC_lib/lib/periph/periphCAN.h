/*
 * hardware.h
 *
 *  Created on: Jun 22, 2019
 *      Author: colson
 */

#ifndef INC_PERIPHCAN_H_
#define INC_PERIPHCAN_H_

#include "main.h"
#include "string.h"

class PeriphCAN {
protected:
	CAN_HandleTypeDef *hcan;

public:
	PeriphCAN() {

	}
	PeriphCAN(CAN_HandleTypeDef *hcan_):
		hcan(hcan_){
	}
	~PeriphCAN() {

	}
	void can_filter_init(CAN_FilterTypeDef type)
	{
		if(HAL_CAN_ConfigFilter(hcan, &type) != HAL_OK)
			Error_Handler();
	}
	void can_transmit_data(CAN_TxHeaderTypeDef *hedder, uint8_t *txData)
	{
		if (HAL_CAN_AddTxMessage(hcan, hedder, txData, &canTxMailBox) != HAL_OK)
			Error_Handler();
	}
	void can_notification_init(uint32_t flags)
	{
		if (HAL_CAN_ActivateNotification(hcan, flags) != HAL_OK)
			Error_Handler();
	}
	void can_start(void)
	{
		if (HAL_CAN_Start(hcan) != HAL_OK)
			Error_Handler();
	}

public:
	uint32_t canTxMailBox;
protected:
};

typedef struct _CAN_Handler{
	PeriphCAN *CAN;
	uint32_t flags;
	CAN_FilterTypeDef filter;
	CAN_TxHeaderTypeDef txHeader;
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t txData[8];
	uint8_t rxData[8];
}CAN_Handler_t;

#endif /* INC_PERIPHUSART_H_ */
