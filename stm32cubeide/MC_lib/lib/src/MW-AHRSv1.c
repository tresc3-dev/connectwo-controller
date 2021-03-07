#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "MW-AHRSv1.h"

char direction[] = "i";
float Azimuth = 0;
float value = 0;

void mw_ahrs_input_data(MW_AHRS *dst) {
	if (dst->can_read_data[0] == 0xF0) {
		switch (dst->can_read_data[1]) {
		case ACCELERATION:
			memcpy(dst->a_raw_data, &dst->can_read_data[2], 6);
			dst->a_x = GET_S16_BYTE(dst->a_raw_data[0], dst->a_raw_data[1])
					/ 1000.0f;
			dst->a_y = GET_S16_BYTE(dst->a_raw_data[2], dst->a_raw_data[3])
					/ 1000.0f;
			dst->a_z = GET_S16_BYTE(dst->a_raw_data[4], dst->a_raw_data[5])
					/ 1000.0f;

			dst->ai_x += dst->a_x;
			dst->ai_y += dst->a_y;
			dst->ai_z += dst->a_z;
			break;
		case GYROSCOPE:
			memcpy(dst->g_raw_data, &dst->can_read_data[2], 6);
			dst->g_x = GET_S16_BYTE(dst->g_raw_data[0], dst->g_raw_data[1])
					/ 10.0f;
			dst->g_y = GET_S16_BYTE(dst->g_raw_data[2], dst->g_raw_data[3])
					/ 10.0f;
			dst->g_z = GET_S16_BYTE(dst->g_raw_data[4], dst->g_raw_data[5])
					/ 10.0f;
			break;
		case ANGLE:
			memcpy(dst->e_raw_data, &dst->can_read_data[2], 6);
			dst->e_roll = GET_S16_BYTE(dst->e_raw_data[0], dst->e_raw_data[1])
					/ 100.0f;
			dst->e_pitch = GET_S16_BYTE(dst->e_raw_data[2], dst->e_raw_data[3])
					/ 100.0f;
			dst->e_yaw = GET_S16_BYTE(dst->e_raw_data[4], dst->e_raw_data[5])
					/ 100.0f;
			break;
		case MAGNETIC:
			memcpy(dst->m_raw_data, &dst->can_read_data[2], 6);
			dst->m_x = GET_S16_BYTE(dst->m_raw_data[0], dst->m_raw_data[1])
					/ 10.0f;
			dst->m_y = GET_S16_BYTE(dst->m_raw_data[2], dst->m_raw_data[3])
					/ 10.0f;
			dst->m_z = GET_S16_BYTE(dst->m_raw_data[4], dst->m_raw_data[5])
					/ 10.0f;
			break;
		}

		value = dst->m_x / dst->m_y;
		Azimuth = 90 - atanf(value);

		/*
		 if(dst->m_x <= -0.4 &&dst->m_x >= -1.5   &&dst->m_y <= -4.5 &&dst->m_y >= -5.0 )
		 {
		 strcpy(direction, "N");   //North
		 }
		 else if(dst->m_x <= -1.0 &&dst->m_x >= -2.2   &&dst->m_y <= -7.3)
		 {
		 strcpy(direction, "S");  //South
		 }
		 else if(dst->m_x >= -0.3   &&dst->m_y <= -6.3 &&dst->m_y >= -7.3)
		 {
		 strcpy(direction, "W");  //West
		 }
		 else if(dst->m_x <= -2.2 &&dst->m_x >= -2.7  &&dst->m_y <= -5.0 &&dst->m_y >= -6.4)
		 {
		 strcpy(direction, "E");   //East
		 }

		 else if(dst->m_x <= -1.5 &&dst->m_x >= -2.2   &&dst->m_y <= -4.5 &&dst->m_y >= -5.0)
		 {
		 strcpy(direction, "NE");  //North East
		 }
		 else if(dst->m_x >= -0.4   &&dst->m_y <= -5.0 &&dst->m_y >= -6.3)
		 {
		 strcpy(direction, "NW");  //North West
		 }
		 else if(dst->m_x <= -2.2 &&dst->m_x >= -2.7   &&dst->m_y <= -6.4 &&dst->m_y >= -7.1)
		 {
		 strcpy(direction, "SE");  //South East
		 }
		 else if(dst->m_x <= -0.2 &&dst->m_x >= -1.0   &&dst->m_y <= -7.1 &&dst->m_y >= -7.5)
		 {
		 strcpy(direction, "SW"); //South West
		 }
		 */

	}
}

void mw_ahrs_set_data_type(MW_AHRS *dst, uint8_t acc, uint8_t gyro,
		uint8_t angle, uint8_t mangetic) {
	// 18 16 00 00    xx 00 00 00
	dst->can_write_data[0] = AC_OBJECT_WRITE_REQ + OT_INT32;
	dst->can_write_data[1] = SET_CAN_DATA;
	dst->can_write_data[2] = 0;
	dst->can_write_data[3] = 0;

	dst->can_write_data[4] = (acc << DT_ACC) + (gyro << DT_GYRO)
			+ (angle << DT_ANGLE) + (mangetic << DT_MAGNETIC);
	dst->can_write_data[5] = 0;
	dst->can_write_data[6] = 0;
	dst->can_write_data[7] = 0;
}

void mw_ahrs_set_period(MW_AHRS *dst, uint32_t time) {
	dst->can_write_data[0] = AC_OBJECT_WRITE_REQ + OT_INT32;
	dst->can_write_data[1] = SET_PERIOD;
	dst->can_write_data[2] = 0;
	dst->can_write_data[3] = 0;

	dst->can_write_data[4] = GET_LOWBYTE_16(GET_LOWWORD_32(time));
	dst->can_write_data[5] = GET_HIGHBYTE_16(GET_LOWWORD_32(time));
	dst->can_write_data[6] = GET_LOWBYTE_16(GET_HIGHWORD_32(time));
	dst->can_write_data[7] = GET_HIGHBYTE_16(GET_HIGHWORD_32(time));
}
