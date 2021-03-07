/*
 * absoluteEncoder.c
 *
 *  Created on: Jun 23, 2019
 *      Author: colson
 */

#include "absoluteEncoder.h"
#include "conf.h"

typedef uint8_t byte;
typedef unsigned int word;

word absStep = 0;
word absCnt = 0;
word absDelayCnt = 0;
word absResetCnt[4] = {0,};
word absReadData[4] = {0,};
word absDataTemp[4] = {0,};
word absNowData[4] = {0,};
const word ENC_HEX[12] = {
		  0x0800, 0x0400, 0x0200, 0x0100,
		  0x0080, 0x0040, 0x0020, 0x0010,
		  0x0008, 0x0004, 0x0002, 0x0001
};


void abs_encoder_step_calculator(void)
{
	byte i = 0;
	switch(absStep)
	{
	case 0:
		absCnt = 0;
		abs_temp_clear();
		abs_clock_clear();
		absStep = 1;
		break;
	case 1:
		abs_clock_set();
		absStep = 2;
		break;
	case 2:
		abs_read_data();
		abs_clock_clear();
		absStep = 3;
		break;
	case 3:
		for(i = 0 ; i < 4; i ++)
			if(absReadData[i] != 0)
				absDataTemp[i] += ENC_HEX[absCnt];
		abs_clock_set();
		absCnt++;

		if(absCnt < 12)
			absStep = 2;
		else
			absStep = 4;
		break;
	case 4:
		abs_get_data();

		if(absDelayCnt++ > 100)
		{
			absDelayCnt = 0;
			absStep = 0;
		}
		break;


	case 100:
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
		if(absResetCnt[0] ++ > 10000)
		{
			absResetCnt[0] = 0;
			absStep = 101;
		}
		break;
	case 101:
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
		if(absResetCnt[0]++ > 15000)
		{
			absResetCnt[0] = 0;
			absStep = 102;
		}
		break;
	case 102:
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
		if(absResetCnt[0]++ > 100)
		{
			absResetCnt[0] = 0;
			absStep = 0;
		}
		break;


	case 200:
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		if(absResetCnt[1] ++ > 10000)
		{
			absResetCnt[1] = 0;
			absStep = 201;
		}
		break;
	case 201:
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
		if(absResetCnt[1]++ > 15000)
		{
			absResetCnt[1] = 0;
			absStep = 202;
		}
		break;
	case 202:
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		if(absResetCnt[1]++ > 100)
		{
			absResetCnt[1] = 0;
			absStep = 0;
		}
		break;

	case 300:
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
		if(absResetCnt[2] ++ > 10000)
		{
			absResetCnt[2] = 0;
			absStep = 301;
		}
		break;
	case 301:
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
		if(absResetCnt[2]++ > 15000)
		{
			absResetCnt[2] = 0;
			absStep = 302;
		}
		break;
	case 302:
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
		if(absResetCnt[2]++ > 100)
		{
			absResetCnt[2] = 0;
			absStep = 0;
		}
		break;

	case 400:
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
		if(absResetCnt[3] ++ > 10000)
		{
			absResetCnt[3] = 0;
			absStep = 401;
		}
		break;
	case 401:
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
		if(absResetCnt[3]++ > 15000)
		{
			absResetCnt[3] = 0;
			absStep = 402;
		}
		break;
	case 402:
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
		if(absResetCnt[3]++ > 100)
		{
			absResetCnt[3] = 0;
			absStep = 0;
		}
		break;
	}
}
void abs_temp_clear(void)
{
	absDataTemp[0] = 0;
	absDataTemp[1] = 0;
	absDataTemp[2] = 0;
	absDataTemp[3] = 0;
}
void abs_clock_clear(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
}
void abs_clock_set(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
}
void abs_read_data(void)
{
	absReadData[0] = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
	absReadData[1] = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5);
	absReadData[2] = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
	absReadData[3] = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
}
void abs_get_data(void)
{
	absNowData[0] = absDataTemp[0];
	absNowData[1] = absDataTemp[1];
	absNowData[2] = absDataTemp[2];
	absNowData[3] = absDataTemp[3];
}
void abs_zero_clear(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
}
void abs_zero_set(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
}
