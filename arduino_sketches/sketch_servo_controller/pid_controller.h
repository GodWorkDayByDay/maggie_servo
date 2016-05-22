#ifndef PID_CONTROLLER_h
#define PID_CONTROLLER_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "controller_globals.h"

#define MAX_PID_OUTPUT	11.5
#define MIN_PID_OUTPUT	-11.5

#define PROPORTIONAL_PARAM			0.1
#define INTEGRAL_PARAM				0.01
#define DERIVATIVE_PARAM			0	

class PIDController {
public:
	PIDController();

	double CalculatePID(const double &input, const double &targetVelocity);

private:
	volatile double _lastVelocity;
	volatile double _integralTerm;
};

#endif