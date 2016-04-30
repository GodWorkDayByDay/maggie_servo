#ifndef POSITION_CONTROLLER_h
#define POSITION_CONTROLLER_h

#include <math.h>

#define RADIANS_PER_SENSOR_TICK 0.0038929277

#define MOTOR_MAX_ROTATION (M_PI * 2) // rads
#define MOTOR_MAX_SPEED 11.5 // rads/s measured for Lyxmotion 12V 90 rpm motor

#define LM_MOTOR_TORQUE_CONSTANT // estimated to be: 0.43 nm/A from chart - Kt = T / I

class PositionController {
public:
	PositionController();
	//PositionController(double maxRotation = MOTOR_MAX_ROTATION, double maxSpeed = MOTOR_MAX_SPEED);

	double calculateVelocity(const long &sensorCount, const long &lastSensorCount, const double &interval);
	double calculateAcceleration(const double &velocity, const double &lastVelocity, const double &interval);

private:
	// double _maxRotation;
	// double _maxSpeed;
	double _lastSensorCount;	
};

#endif