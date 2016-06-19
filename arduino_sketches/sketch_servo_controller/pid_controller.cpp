#include "pid_controller.h"

PIDController::PIDController() : _integralTerm(0)
{
}

double PIDController::calculatePID(const double &velocity, const double &targetVelocity)
//double PIDController::calculatePID(const double velocityError)
{
  double velocityError = targetVelocity - velocity;
  
  if (velocityError == 0)
    _integralTerm = 0;
  else
    _integralTerm += (INTEGRAL_PARAM * velocityError);
  
  if (_integralTerm > MAX_PID_OUTPUT) 
    _integralTerm = MAX_PID_OUTPUT;
  else if (_integralTerm < MIN_PID_OUTPUT) 
    _integralTerm = MIN_PID_OUTPUT;
  
  double dInput = 0; //(velocity - _lastVelocity);

  /*Compute PID Output*/
  double output = PROPORTIONAL_PARAM * velocityError + _integralTerm - DERIVATIVE_PARAM  * dInput;
  
  if (output > MAX_PID_OUTPUT) 
    output = MAX_PID_OUTPUT;
  else if (output < MIN_PID_OUTPUT) 
    output = MIN_PID_OUTPUT;
  
  //_lastVelocity = velocity;

  return output;
}

// void PIDController::reset()
// {
//   _integralTerm = 0;  
// }