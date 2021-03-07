/*
 * Ahrsv1.cpp
 *
 *  Created on: Mar 1, 2021
 *      Author: dud37
 */

#include <Ahrsv1.h>
#include <string.h>

Ahrsv1::Ahrsv1(CAN_HandleTypeDef *_hcan, CAN_Handler_t _init) :
		PeriphCAN(_hcan, _init) {
	data = {0,};
	txHeader = &handle.txHeader;
	rxHeader = &handle.rxHeader;
	txData = handle.txData;
	rxData = handle.rxData;
}

Ahrsv1::~Ahrsv1() {

}

void Ahrsv1::init()
{
	periphCanInit();
}

void Ahrsv1::getInputData() {
	if (data.can_read_data[0] == 0xF0) {
		switch (data.can_read_data[1]) {
		case ACCELERATION:
			memcpy(data.a_raw_data, &data.can_read_data[2], 6);
			data.a_x = GET_S16_BYTE(data.a_raw_data[0], data.a_raw_data[1])
					/ 1000.0f;
			data.a_y = GET_S16_BYTE(data.a_raw_data[2], data.a_raw_data[3])
					/ 1000.0f;
			data.a_z = GET_S16_BYTE(data.a_raw_data[4], data.a_raw_data[5])
					/ 1000.0f;

			data.ai_x += data.a_x;
			data.ai_y += data.a_y;
			data.ai_z += data.a_z;
			break;
		case GYROSCOPE:
			memcpy(data.g_raw_data, &data.can_read_data[2], 6);
			data.g_x = GET_S16_BYTE(data.g_raw_data[0], data.g_raw_data[1])
					/ 10.0f;
			data.g_y = GET_S16_BYTE(data.g_raw_data[2], data.g_raw_data[3])
					/ 10.0f;
			data.g_z = GET_S16_BYTE(data.g_raw_data[4], data.g_raw_data[5])
					/ 10.0f;
			break;
		case ANGLE:
			memcpy(data.e_raw_data, &data.can_read_data[2], 6);
			data.e_roll = GET_S16_BYTE(data.e_raw_data[0], data.e_raw_data[1])
					/ 100.0f;
			data.e_pitch = GET_S16_BYTE(data.e_raw_data[2], data.e_raw_data[3])
					/ 100.0f;
			data.e_yaw = GET_S16_BYTE(data.e_raw_data[4], data.e_raw_data[5])
					/ 100.0f;
			break;
		case MAGNETIC:
			memcpy(data.m_raw_data, &data.can_read_data[2], 6);
			data.m_x = GET_S16_BYTE(data.m_raw_data[0], data.m_raw_data[1])
					/ 10.0f;
			data.m_y = GET_S16_BYTE(data.m_raw_data[2], data.m_raw_data[3])
					/ 10.0f;
			data.m_z = GET_S16_BYTE(data.m_raw_data[4], data.m_raw_data[5])
					/ 10.0f;
			break;
		}

		value = data.m_x / data.m_y;
		Azimuth = 90 - atanf(value);
	}
}
void Ahrsv1::setDataType(uint8_t acc, uint8_t gyro, uint8_t angle,
		uint8_t magnetic) {
	// 18 16 00 00    xx 00 00 00

	handle.txData[0] = AC_OBJECT_WRITE_REQ + OT_INT32;
	handle.txData[1] = SET_CAN_DATA;
	handle.txData[2] = 0;
	handle.txData[3] = 0;

	handle.txData[4] = (acc << DT_ACC) + (gyro << DT_GYRO) + (angle << DT_ANGLE)
			+ (magnetic << DT_MAGNETIC);
	handle.txData[5] = 0;
	handle.txData[6] = 0;
	handle.txData[7] = 0;

	transmitData(&handle.txHeader, handle.txData);
}
void Ahrsv1::setPeriod(uint32_t time) {
	handle.txData[0] = AC_OBJECT_WRITE_REQ + OT_INT32;
	handle.txData[1] = SET_PERIOD;
	handle.txData[2] = 0;
	handle.txData[3] = 0;

	handle.txData[4] = GET_LOWBYTE_16(GET_LOWWORD_32(time));
	handle.txData[5] = GET_HIGHBYTE_16(GET_LOWWORD_32(time));
	handle.txData[6] = GET_LOWBYTE_16(GET_HIGHWORD_32(time));
	handle.txData[7] = GET_HIGHBYTE_16(GET_HIGHWORD_32(time));

	transmitData(&handle.txHeader, handle.txData);
}
void Ahrsv1::writeData(CAN_TxHeaderTypeDef* hedder, uint8_t* txData) {

}
void Ahrsv1::checkStatus() {

}

