/*
 * Nonholonomic.cpp
 *
 *  Created on: 2021. 2. 24.
 *      Author: colson
 */

#ifndef NONHOLONOMIC_CPP_
#define NONHOLONOMIC_CPP_

/*      _____________
 *     |             |
 *  ___|             |___
 * |   |             |   |
 * |   |<----(2l)---->|   |
 * |___|             |___|
 *     |             |
 *     |_____________|
 *
 *	wheel diameter = (2r)
 *
 * */

namespace tresc3 {
typedef struct motorValue_t {
	double leftValue;
	double rightValue;
} motorValueTypeDef;
class Nonholonomic {
private:
	double r;
	double l;
	double p;
	double t;

	double factor;
public:
	/*
	 * @param _r	: wheel radius
	 * @param _l	: wheel distance / 2
	 * @param _p	: pulses within 1 rev.
	 * @param _t	: control cycle
	 * */
	Nonholonomic(double _r, double _l, double _p, double _t) :
			r(_r), l(_l), p(_p), t(_t) {
		factor = p * t / 6.28 / r;
	}
	;
	~Nonholonomic() {
	}
	;

	motorValueTypeDef dynamics(double _linearVelocity,
			double _angularVelocity) {
		motorValueTypeDef ret;
		double l_value = 0;
		double a_value = 0;
		if (_linearVelocity)
			l_value = _linearVelocity * factor;
		if(_angularVelocity)
			a_value = _angularVelocity * l * factor;
		ret.rightValue = l_value + a_value;
		ret.leftValue = l_value - a_value;
		return ret;
	}
private:

};
}

#endif /* NONHOLONOMIC_CPP_ */
