/*
 * periphMotor.h
 *
 *  Created on: 2019. 6. 24.
 *      Author: colson
 */

#ifndef INC_PERIPHMOTOR_H_
#define INC_PERIPHMOTOR_H_


#include "main.h"

typedef enum{
	MOTOR_DIT_RESET = 0,
	MOTOR_DIR_SET,
}MOTOR_DIR;

class PeriphMotor {
protected:
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
	MOTOR_DIR dir;
	TIM_TypeDef *TIMx;
	uint32_t prescaler;
	uint32_t counter;
	uint32_t duty;

public:
	PeriphMotor(GPIO_TypeDef *GPIOx_, uint16_t GPIO_Pin_, TIM_TypeDef *TIMx_):
		GPIOx(GPIOx_), GPIO_Pin(GPIO_Pin_), TIMx(TIMx_){

	}

	void init(){
		//TODO INIT TIMER PWM


		reset();
	};
	void reset(void){
		duty = 0;
	}
	void setPWM(uint16_t _duty){

	}
	void setDIR(uint16_t _dir){

	}
	void start(void){

	}

};


#endif /* INC_PERIPHMOTOR_H_ */
