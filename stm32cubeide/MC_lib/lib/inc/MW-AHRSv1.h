#ifndef SOURCES_MW_AHRSV1_H_
#define SOURCES_MW_AHRSV1_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

// Access Code
#define AC_OBJECT_WRITE_REQ		0x10
#define AC_OBJECT_WRITE_RES		0x20
#define AC_OBJECT_READ_REQ		0x30
#define AC_OBJECT_READ_RES		0x40
#define AC_OBJECT_ERROR_RES		0x80

// Object Type
#define OT_INT8					0x00
#define OT_INT16				0x04
#define OT_INT32				0x08
#define OT_FLOAT				0x0C

// Error Code
#define UNDEFINED_NAME			1		// Index�� Name���� ������ ������Ʈ�� �������� ����
#define PACKET_FORMAT_ERROR		2		// ��Ŷ�� ������ �߸� �Ǿ���
#define OBJECT_ACCESS_ERROR		3		// �б� ���� ������Ʈ�� ���ų�, ���� ���� ������Ʈ�� �б⸦ �õ���
#define WRONG_VALUE_ASSIGNMENT	        4		// ���� ������Ʈ�� ������ ����� ���� ���� �õ���. ex) id=5000

#define GET_LOWBYTE_16(X)		(X & 0xFF)
#define GET_HIGHBYTE_16(X)		((X >> 8) & 0xFF)

#define GET_LOWWORD_32(X)		(X & 0xFFFF)
#define GET_HIGHWORD_32(X)		((X >> 16) & 0xFFFF)

#define GET_S16_BYTE(LOW,HIGH)	((int16_t)(LOW + ((uint16_t)(HIGH) << 8)))
#define GET_U16_BYTE(LOW,HIGH)	((uint16_t)(LOW + ((uint16_t)(HIGH) << 8)))
#define GET_S32_WORD(LOW,HIGH)	((int32_t)(LOW + ((uint32_t)(HIGH) << 16)))
#define GET_U32_WORD(LOW,HIGH)	((uint32_t)(LOW + ((uint32_t)(HIGH) << 16)))


// PROTOCOL
// |     1	|       2	|       3	|       4	|       5	|       6	|       7	|       8	|
// |    CMD 	|             INDEX	        |    SUB-IDX    |                            VALUE				|	����
// |    CMD     |            ERR CODE       	|                                NULL(0x00) * 6  				|	����
// |   (0xF0)	|     INDEX	|         Value1(INT16)         |         Value2(INT16)         |         Value3(INT16)	        |	����ȭ ������ ����


enum MW_AHRS_INDEX {
	// INT32
	PROVIDER_ID = 1, PRODUCT_ID, SOFTWARE_VERSION, HARDWARE_VERSION,	// RO
	COMMAND=7,	                                                        // WO
	DEVICE_ID=11, CAN_BUS_SPEED, SERIAL_BUS_SPEED,                          // RW
	MAGNETIC_VARIANCE=19, ACCELEROMETER_VARIANCE,                           // RW
	SET_SERIAL_DATA=21, SET_CAN_DATA,                                       // RW
	SET_PERIOD=24,                                                          // RW

	// FLOAT
	ACCELERATION=51, GYROSCOPE, ANGLE, MAGNETIC,                            // RO
	TEMPERATURE=57
};


// ex) sc = (1<<DT_ACC) + (1<<DT_GYRO) + (1<<DT_ANGLE) + (0<<DT_MAGNETIC);
// ACC ON, GYRO ON, ANGLE ON, MAGNETIC OFF
//      0       1       1       1
//     MAG     ANG     GYR     ACC

enum MW_AHRS_DATA_TYPE_LIST {
	DT_ACC=0, DT_GYRO, DT_ANGLE, DT_MAGNETIC
};



typedef struct
{
	// Acceleration
	float a_x;
	float a_y;
	float a_z;

	// Angular velocity
	float g_x;
	float g_y;
	float g_z;

	// Euler
	float e_yaw;
	float e_pitch;
	float e_roll;

	// Magnetic
	float m_x;
	float m_y;
	float m_z;

	// acc interal
	float ai_x;
	float ai_y;
	float ai_z;


	uint8_t a_raw_data[6];
	uint8_t g_raw_data[6];
	uint8_t e_raw_data[6];
	uint8_t m_raw_data[6];

	uint8_t can_write_data[8];
	uint8_t can_read_data[8];

}MW_AHRS;

void mw_ahrs_input_data(MW_AHRS *dst);
void mw_ahrs_set_data_type(MW_AHRS *dst, uint8_t acc, uint8_t gyro, uint8_t angle, uint8_t mangetic);
void mw_ahrs_set_period(MW_AHRS *dst, uint32_t time); // ms


#ifdef __cplusplus
}
#endif
#endif /* SOURCES_MW_AHRSV1_H_ */
