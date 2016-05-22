#include "pid_controller.h"

PIDController::PIDController() : _integralTerm(0)
{
}

double PIDController::CalculatePID(const double &velocity, const double &targetVelocity)
{
  double error = targetVelocity - velocity;
  
  _integralTerm += (INTEGRAL_PARAM * error);
  if (_integralTerm > MAX_PID_OUTPUT) 
    _integralTerm = MAX_PID_OUTPUT;
  else if (_integralTerm < MIN_PID_OUTPUT) 
    _integralTerm = MIN_PID_OUTPUT;
  
  double dInput = (velocity - _lastVelocity);

  /*Compute PID Output*/
  double output = PROPORTIONAL_PARAM * error + _integralTerm - DERIVATIVE_PARAM  * dInput;
  
  if (output > MAX_PID_OUTPUT) 
    output = MAX_PID_OUTPUT;
  else if (output < MIN_PID_OUTPUT) 
    output = MIN_PID_OUTPUT;
  
  _lastVelocity = velocity;

  return output;
}