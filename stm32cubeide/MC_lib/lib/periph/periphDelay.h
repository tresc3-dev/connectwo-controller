/*
 * periphMotor.h
 *
 *  Created on: 2019. 6. 24.
 *      Author: colson
 */

#ifndef INC_PERIPHDELAY_H_
#define INC_PERIPHDELAY_H_


#include "main.h"


class PeriphDelay {
protected:
	uint32_t nowtick;
	uint32_t pasttick;
public:
	PeriphDelay(void){
		nowtick = 0;
		pasttick = 0;
	}

	void delay_ms(uint32_t count)
	{
		pasttick = HAL_GetTick();
		while(1)
		{
			//TODO limitation
			nowtick = HAL_GetTick();
			if(nowtick - pasttick > count)
				break;
		}
	}
	void delay_s(uint32_t count)
	{
		pasttick = HAL_GetTick();
		while(1)
		{
			//TODO limitation
			nowtick = HAL_GetTick();
			if(nowtick - pasttick > count * 1000)
				break;
		}
	}
};


#endif /* INC_PERIPHDELAY_H_ */
