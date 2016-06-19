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

#define PROPORTIONAL_PARAM			0 //(double)(1 / MOTOR_PWM_VELOCITY_FACTOR)
#define INTEGRAL_PARAM				0.005
#define DERIVATIVE_PARAM			0

class PIDController {
public:
	PIDController();

	double calculatePID(const double &input, const double &targetVelocity);
	//double calculatePID(const double velocityError);
	// void reset();

private:
	// volatile double _lastVelocity;
	volatile double _integralTerm;
};

#endif