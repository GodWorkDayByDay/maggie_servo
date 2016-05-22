#ifndef POSITION_CONTROLLER_h
#define POSITION_CONTROLLER_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "controller_globals.h"
#include "pid_controller.h"
#include <math.h>

class PositionController {
public:
	PositionController();
	
	void setTargets(double &targetPosition, double &targetVelocity, uint8_t &feedforwardPWM);
	void moveToGoal(	const volatile long &sensorCount,
						const volatile long &lastSensorCount,
						void (*motorCB)(const uint8_t pwm, bool directionCCW)
						#ifdef SERIAL_DEBUGGING
							,MotorDebug &motorDebug
						#endif
						);
	
	uint8_t calculateFeedforwardPWM(const double &targetVelocity);
	double calculateVelocity(const volatile long &sensorCount, const volatile long &lastSensorCount, const double &interval);
	double calculateAcceleration(const double &velocity, const volatile double &lastVelocity, const double &interval);

private:
	PIDController _pidController;

	volatile double _targetPosition;
	volatile double _targetVelocity;
	uint8_t _feedforwardPWM;

	volatile double _lastVelocity;
	double _integralTerm;
};

#endif