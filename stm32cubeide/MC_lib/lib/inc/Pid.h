/*
 * Pid.h
 *
 *  Created on: 2021. 2. 22.
 *      Author: colson
 */

#ifndef PID_H_
#define PID_H_

#include "main.h"

namespace tresc3 {

template<typename T>
struct pidProperty {
	T kP;
	T kI;
	T kD;

	T target;

	T nowValue;
	T pastValue;

	T nowError;
	T pastError;

	T errorSum;
	T errorSumLimit;
	T errorDiff;

	T nowOutput;
	T pastOutput;
	T outputLimit;

	T underOfPoint;
};

template <typename T>
class Pid {
public:

private:
	pidProperty<T> m_pid;

public:
	Pid(){};
	Pid(pidProperty<T> _pidProperty): m_pid(_pidProperty){};
	~Pid(){};

public:
	T run(T _target, T _input)
	{

		m_pid.nowValue = (T)_input;
		m_pid.target = (T)_target;

		m_pid.nowError = m_pid.nowValue -  m_pid.target;
		m_pid.errorSum += m_pid.nowError;
		m_pid.errorDiff = m_pid.nowError - m_pid.pastError;

		if (m_pid.errorSumLimit != 0)
		{
			if(m_pid.errorSum > m_pid.errorSumLimit)
				m_pid.errorSum = m_pid.errorSumLimit;
			else if(m_pid.errorSum < -m_pid.errorSumLimit)
				m_pid.errorSum = -m_pid.errorSumLimit;
		}

		m_pid.nowOutput =
				m_pid.kP * m_pid.nowError +
				m_pid.kI * m_pid.errorSum +
				m_pid.kD * m_pid.errorDiff;

		if(m_pid.underOfPoint == 0) return 0;	// Escape Error

		m_pid.nowOutput /= m_pid.underOfPoint;
		m_pid.pastError = m_pid.nowError;

		if(m_pid.outputLimit != 0)
		{
			if(m_pid.nowOutput > m_pid.outputLimit) m_pid.nowOutput = m_pid.outputLimit;
			else if(m_pid.nowOutput < -m_pid.outputLimit) m_pid.nowOutput = -m_pid.outputLimit;
		}

		return m_pid.nowOutput;
	}
	void setGain(T _kP, T _kI, T _kD)
	{
		m_pid.kP = _kP;
		m_pid.kI = _kI;
		m_pid.kD = _kD;
	}

	void setErrorSumLimit(T _limit)
	{
		m_pid.errorSumLimit = _limit;
	}

	void setProperty(pidProperty<T> _property)
	{
		m_pid = _property;
	}

	pidProperty<T> getProperty(void)
	{
		return m_pid;
	}
};
} // namespace tresc3

#endif /* PID_H_ */
