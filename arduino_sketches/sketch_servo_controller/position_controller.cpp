#include "position_controller.h"

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

PositionController::PositionController() {}

// PositionController::PositionController(double maxRotation, double maxSpeed)
// {
// 	_maxRotation = fmin(maxRotation, MOTOR_MAX_ROTATION);
// 	_maxSpeed = fmin(maxSpeed, MOTOR_MAX_SPEED);
// }

/*
 * Calculate the interval seconds
 */ 

///
/// Calculate the velocity in radians/s
double PositionController::calculateVelocity(const long &sensorCount, const long &lastSensorCount, const double &interval)
{
	return interval == 0 ? 0 : (sensorCount - lastSensorCount) * RADIANS_PER_SENSOR_TICK / interval;
}

///
/// Calculate the acceleration in radians/s^2
double PositionController::calculateAcceleration(const double &velocity, const double &lastVelocity, const double &interval)
{
	// the acceleration is just the derivative of the velocity!
	return interval == 0 ? 0 : (velocity - lastVelocity) / interval;
}

// void PositionController::calculatePID()
// {
	
// }