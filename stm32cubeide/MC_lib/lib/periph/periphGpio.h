/*
 * stateLed.h
 *
 *  Created on: 2019. 6. 23.
 *      Author: colson
 */

#ifndef INC_PERIPHGPIO_H_
#define INC_PERIPHGPIO_H_

#include "main.h"

class PeriphGPIO {
protected:
	GPIO_TypeDef *GPIOx;			/**< GPIOx*/
	uint16_t GPIO_Pin;				/**< GPIOx_Pin*/
	GPIO_PinState PinState;			/**< GPIOx_State*/
	uint16_t period;				/**< GPIOx_period (uint 1ms)*/
	uint32_t nowtick;				/**< GPIOx_time control*/
	uint32_t pasttick;				/**< GPIOx_time control*/

public:

	PeriphGPIO(){	}

	PeriphGPIO(GPIO_TypeDef *GPIOx_, uint16_t GPIO_Pin_, uint16_t period_ = 1000):
		GPIOx(GPIOx_), GPIO_Pin(GPIO_Pin_), PinState(GPIO_PIN_RESET), period(period_){
	}

	void init(){reset();}

	void re(void){period = 1000;}

	void setPeriod(uint16_t period_){period = period_;}

	void toggle(void){HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);}

	void set(void){HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);}

	void reset(void){HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);}

	GPIO_PinState read(void){return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);}

	void run(void){
		nowtick = HAL_GetTick();
		if(period == 0)
		{
		}
		else if(nowtick - pasttick > period)
		{
			toggle();
			pasttick = nowtick;
		}
	}
};




#endif /* INC_PERIPHGPIO_H_ */
