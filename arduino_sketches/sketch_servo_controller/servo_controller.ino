#include "position_controller.h"
#include <TimerOne.h>
//#include <MsTimer2.h>
// #include <PID_v1.h> 

//#define SERIAL_DEBUGGING

//#define RADIANS_PER_SENSOR_TICK 0.0038929277
#define TIMER1_MICRO_SECONDS 250000
#define TIMER2_MILLI_SECONDS 250

void setup();
void loop();

void timer1_CB();
//void timer2_CB();

void sensorA_ISR();
void sensorB_ISR();

// pin assignments
int ledPin = 13;
int pwmPin = 11;    
int sensorA_Pin = 0; // pin 2 (UNO)
int sensorB_Pin = 1; // pin 3 (UNO)
int motorDirPin = 7;

uint8_t _pwm = 0;

#ifdef SERIAL_DEBUGGING
  char _serialDebugString[200];
#endif

// ISR globals
volatile long _sensorCount, _lastSensorCount;
volatile double _lastVelocity; 
volatile bool _lastSensorA, _lastSensorB;
volatile bool _directionCCW, _directionA_CCW, _directionB_CCW;


PositionController _positionController;

// double _pidInput;
// double _pidOutput;
// double _pidSetpoint;
// PID _pid(&_pidInput, &_pidOutput, &_pidSetpoint, 10, 5, 3, DIRECT);

bool toggle;

void setup()  { 

  pinMode(ledPin, OUTPUT);  
  pinMode(motorDirPin, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(sensorA_Pin, INPUT);
  pinMode(sensorB_Pin, INPUT);

   /*
   * Increase the pwm frequency to 31250 - no prescaling. 
   * Increasing the PWM frequency helps reduce the acoustic noise
   */
  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  /*
   * External interupts for the quadrature sensor
   */
  attachInterrupt(sensorA_Pin, sensorA_ISR, CHANGE);
  attachInterrupt(sensorB_Pin, sensorB_ISR, CHANGE);

  _sensorCount = _lastSensorCount = 0;
  _lastVelocity = 0;

  _directionCCW = LOW; 
   digitalWrite(motorDirPin, !_directionCCW); // CW

  Timer1.initialize(TIMER1_MICRO_SECONDS);
  Timer1.attachInterrupt(timer1_CB);

  // MsTimer2::set(TIMER2_MILLI_SECONDS, timer2_CB);
  // MsTimer2::start();

  // _pid.SetOutputLimits(-255,255);
  // _pid.SetMode(AUTOMATIC);
  // _pidInput = 0;
  // _pidOutput = 0;
  // _pidSetpoint = 6.28; // 360
  //_pidSetpoint = 1 / RADIANS_PER_SENSOR_TICK; // 1 radian
  //

#ifdef SERIAL_DEBUGGING
  Serial.begin(115200);
#endif

  _pwm = 128; // test only!!!
  analogWrite(pwmPin, _pwm);
} 

void loop()  
{ 

#ifdef SERIAL_DEBUGGING
  if (toggle) 
  {
    Serial.println(_serialDebugString);
    toggle = false;
  }
#endif

}

void timer1_CB() 
{
  double velocity = _positionController.calculateVelocity((long)_sensorCount, (long)_lastSensorCount, TIMER1_MICRO_SECONDS / 1E6);
  double acceleration = _positionController.calculateAcceleration(velocity, (double)_lastVelocity, TIMER1_MICRO_SECONDS / 1E6);

  _lastVelocity = velocity;
  _lastSensorCount = _sensorCount; 

#ifdef SERIAL_DEBUGGING
  char velocity_str[6];
  char acceleration_str[6];
  dtostrf(velocity, 4, 2, velocity_str);
  dtostrf(acceleration, 4, 2, acceleration_str);
  sprintf(_serialDebugString, "PWM = %3d , Velocity = %s , Acceleration = %s, Sensor count = %ld Current = %d", _pwm, velocity_str, acceleration_str, _sensorCount, analogRead(A0));
  toggle = !toggle;
#endif
}

// void timer2_CB() 
// {
// }

void sensorA_ISR()
{ 
    _directionCCW ? _sensorCount-- : _sensorCount++;
    _lastSensorA = PIND & 0x04; // atmega 328 specific!!

    _directionA_CCW = _lastSensorA == HIGH && _lastSensorB == LOW || _lastSensorA == LOW && _lastSensorB == HIGH;
    if (!(_directionA_CCW ^ _directionB_CCW))
    {
      _directionCCW = _directionA_CCW;
    }
}

void sensorB_ISR()
{
    _directionCCW ? _sensorCount-- : _sensorCount++;
    _lastSensorB = PIND & 0x08; // atmega 328 specific!!

    _directionB_CCW = _lastSensorA == HIGH && _lastSensorB == HIGH || _lastSensorA == LOW && _lastSensorB == LOW;
    if (!(_directionA_CCW ^ _directionB_CCW))
    {
      _directionCCW = _directionB_CCW;
    }
}