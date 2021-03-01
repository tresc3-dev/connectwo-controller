/*
 * Ahrsv1.h
 *
 *  Created on: Mar 1, 2021
 *      Author: dud37
 */

#ifndef AHRSV1_H_
#define AHRSV1_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <main.h>
#include <math.h>
#include <MW-AHRSv1.h>
#include <periphCAN.h>

class Ahrsv1 : public PeriphCAN {
public:
	Ahrsv1(CAN_HandleTypeDef *_hcan, CAN_Handler_t _init);
	~Ahrsv1();
public:

	MW_AHRS data;
private:
	CAN_TxHeaderTypeDef* txHeader;
	CAN_RxHeaderTypeDef* rxHeader;
	uint8_t* txData;
	uint8_t* rxData;

	float Azimuth = 0;
	float value = 0;

public:
	void init();

	void getInputData();
	/*
	 * @param acc		: acceleration flag
	 * @param gyro		: gtroscope flag
	 * @param angle		: euler angle flag
	 * @param magnetic	: magnetic flag
	 * */
	void setDataType(uint8_t acc, uint8_t gyro, uint8_t angle, uint8_t magnetic);
	/*
	 * @param time		: data stream interval (ms)
	 * */
	void setPeriod(uint32_t time);

	void writeData(CAN_TxHeaderTypeDef *hedder, uint8_t *txData);

	void checkStatus();

	CAN_TxHeaderTypeDef* getTxHedderAddr()
	{
		return txHeader;
	}

	CAN_RxHeaderTypeDef* getRxHedderAddr()
	{
		return rxHeader;
	}

	uint8_t* getTxDataAddr()
	{
		return txData;
	}

	uint8_t* getRxDataAddr()
	{
		return rxData;
	}
};

#ifdef __cplusplus
}
#endif
#endif /* AHRSV1_H_ */
