#include "position_controller.h"

PositionController::PositionController() : 	_targetPosition(0), 
											_targetVelocity(0), 
											_lastVelocity(0)
{

}

void PositionController::setTargets(double &targetPosition, double &targetVelocity)
{
	_targetPosition = targetPosition;
	_targetVelocity = targetVelocity;
	_adjustedTargetVelocity = targetVelocity;
}

void PositionController::moveToGoal(const volatile long &sensorCount,
									const volatile long &lastSensorCount,
									void (*motorCB)(const uint8_t pwm, bool directionCCW)
									#ifdef SERIAL_DEBUGGING
										,MotorDebug &motorDebug
									#endif
									)
{	 
	double position = sensorCount * RADIANS_PER_SENSOR_TICK;
  	double velocity = calculateVelocity(sensorCount, lastSensorCount, TIMER1_MICRO_SECONDS / 1E6);
  	double acceleration = calculateAcceleration(velocity, _lastVelocity, TIMER1_MICRO_SECONDS / 1E6);
  	_lastVelocity = velocity;

	double positionError = fabs(_targetPosition - position);

	bool directionCCW = position > _targetPosition;

	setAdjustedTargetVelocity(	velocity,
								position
								#ifdef SERIAL_DEBUGGING
									,motorDebug
								#endif 
								);

	double pidTerm = _pidController.calculatePID(velocity, (double)_adjustedTargetVelocity, positionError <= POSITION_TOLERANCE); 
	
	uint8_t volatile pwm = calculatePWM(pidTerm); 

#ifdef SERIAL_DEBUGGING
	motorDebug.position = position;
	motorDebug.velocity = velocity;
	motorDebug.acceleration = acceleration;
	motorDebug.positionError = positionError;
	motorDebug.directionCCW = directionCCW;
	motorDebug.pidTerm = pidTerm;
	motorDebug.adjustedTargetVelocity = _adjustedTargetVelocity;
#endif	

	motorCB(positionError <= POSITION_TOLERANCE ? 0 : pwm, directionCCW);
}

///
/// Scale the target velocity when within the proportional band.
void PositionController::setAdjustedTargetVelocity( double &velocity,
													double &position
													#ifdef SERIAL_DEBUGGING
														,MotorDebug &motorDebug
													#endif 
													)
{
	double proportionalBand;
	if (_targetPosition < 0)
	{
		proportionalBand = _targetPosition + (PROPORTIONAL_BAND_FACTOR * velocity);
		if (position < proportionalBand)
		{
		 	_adjustedTargetVelocity *= proportionalBand / position; 
		}
	}
	else
	{
		proportionalBand = _targetPosition - (PROPORTIONAL_BAND_FACTOR * velocity);
		if (position > proportionalBand)
		{
		 	_adjustedTargetVelocity *= proportionalBand / position; 
		}
	} 
	#ifdef SERIAL_DEBUGGING
		motorDebug.proportionalBand = proportionalBand;
	#endif
}

///
/// Calculate PWM based on angle to goal and target velocity
uint8_t PositionController::calculatePWM(double adjustedTargetVelocity)
{
	// no load tests of Lynxmotion 12V 90 rpm motor reveal a deadbank of 16, and a velocity gradient of approximately 0.05 rad/s per PWM pulse.
	return (fmin(adjustedTargetVelocity, MOTOR_MAX_VELOCITY) / MOTOR_PWM_VELOCITY_FACTOR) + MOTOR_PWM_DEADBAND;
}

///
/// Calculate the velocity in radians/s
double PositionController::calculateVelocity(const volatile long &sensorCount, const volatile long &lastSensorCount, const double &interval)
{
	return (interval == 0) ? 0 : fabs(((double)(sensorCount - lastSensorCount)) * RADIANS_PER_SENSOR_TICK / interval);
}

///
/// Calculate the acceleration in radians/s^2
double PositionController::calculateAcceleration(const double &velocity, const volatile double &lastVelocity, const double &interval)
{
	double acceleration = 0;

	if (interval != 0)
	{
		// the acceleration is just the derivative of the velocity
		acceleration = (velocity - lastVelocity) / interval;
		if (velocity < 0)
			acceleration *= -1;
	}
	return acceleration;
}