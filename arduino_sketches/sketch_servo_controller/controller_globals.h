#ifndef CONTROLLER_GLOBALS_h
#define CONTROLLER_GLOBALS_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define SERIAL_DEBUGGING 1 

#define TIMER1_MICRO_SECONDS 10000
#define TIMER2_MILLI_SECONDS 250

#define RADIANS_PER_SENSOR_TICK 0.0038929277

#define MOTOR_PWM_VELOCITY_FACTOR 0.05 // no load tests of Lynxmotion 12V 90 rpm motor reveal a deadbank of 16, and a velocity gradient of approximately 0.05 rad/s per PWM pulse.
#define MOTOR_PWM_DEADBAND 16

#define MOTOR_MAX_VELOCITY 11.5 // rads/s measured for Lyxmotion 12V 90 rpm motor

#define PROPORTIONAL_BAND_FACTOR 1

#define POSITION_TOLERANCE 0.01

#ifdef SERIAL_DEBUGGING
typedef struct _MotorDebug {
	double position;
	double velocity;
	double acceleration;
	double positionError;
	bool directionCCW;
	double proportionalBand;
	double pidTerm;
	uint8_t adjustedTargetVelocity;
} MotorDebug;
#endif

#endif