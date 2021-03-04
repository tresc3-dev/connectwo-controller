/*
 * Motor.h
 *
 *  Created on: Feb 27, 2021
 *      Author: colson
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"
#include "pid.h"
#include "gpio.h"


using pwmModule = TIM_HandleTypeDef;
using gpioModule = GPIO_TypeDef;

template <typename T>
class Motor{
private:
	uint32_t nowEncoder;
	uint32_t pastEncoder;
	T encoderCnt;
	T deltaEncoder;
	T targetEncoder;
	T nowOutput;

	Pid<T> pid;

	pwmModule* TIMxP;
	pwmModule* TIMxE;
	uint32_t* CCRx = NULL;
	uint32_t* CNTx;
	gpioModule* GPIOx;
	uint16_t GPIO_Pinx;
	uint32_t channel;

public:
	/*
	 * @param _TIMxP 		: pwm generation timer module addr
	 * @param _TIMxE 		: encoder timer module addr
	 * @param _CCRx  		: pwm Channel addr (uint32_t*)&TIMx->CCRx
	 * @param _GPIOx 		: gpio module of dir pin
	 * @param _GPIO_Pinx 	: gpio pin number of dir pin
	 * @param _channel		: PWM channel
	 * @param _property		: pid setting
	 * */
	Motor(pwmModule* _TIMxP,pwmModule* _TIMxE,
			uint32_t _channel,
			uint32_t* _CCRx, uint32_t* _CNTx,
			gpioModule* _GPIOx, uint16_t _GPIO_Pinx,
			pidProperty<T> _property)
	: TIMxP(_TIMxP), TIMxE(_TIMxE), channel(_channel), CCRx(_CCRx), CNTx(_CNTx), GPIOx(_GPIOx), GPIO_Pinx(_GPIO_Pinx)
	{
		encoderCnt = 0;
		deltaEncoder = 0;
		pastEncoder = 0;
		targetEncoder = 0;

		nowEncoder = 0;
		pastEncoder = 0;

		pid.setProperty(_property);
	}
	~Motor()
	{
		free(TIMxP);
		free(TIMxE);
		free(CCRx);
		free(CNTx);
		free(GPIOx);
	}

	void reset()
	{
		*CCRx = 0;
		*CNTx = 0;
	}

	void start()
	{
		HAL_TIM_PWM_Start(TIMxP, channel);
		HAL_TIMEx_PWMN_Start(TIMxP, channel);
		HAL_TIM_Encoder_Start(TIMxE, TIM_CHANNEL_ALL);
	}

	void setPwm(uint32_t _value)
	{
		*CCRx = _value;
	}

	void setDir(GPIO_PinState _dir)
	{
		HAL_GPIO_WritePin(GPIOx, GPIO_Pinx, _dir);
	}

	void getDeltaEncoder()
	{
		nowEncoder = *CNTx;
		if(nowEncoder > 30000)
			deltaEncoder = (long)nowEncoder - 65535;
		else
			deltaEncoder = nowEncoder;
		*CNTx = 0;
		encoderCnt += deltaEncoder;
	}

	T getTargetEncoder()
	{
		return targetEncoder;
	}

	void motorControl(T _target)
	{
		getDeltaEncoder();
		nowOutput = pid.run(_target, deltaEncoder);
		pastEncoder = nowEncoder;
		if (nowOutput < 0)
		{
			setDir(GPIO_PIN_RESET);
			*CCRx = -nowOutput;
		}
		else
		{
			setDir(GPIO_PIN_SET);
			*CCRx = nowOutput;
		}
	}
};





#endif /* MOTOR_H_ */
