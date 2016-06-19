#include "position_controller.h"
#include <TimerOne.h>
//#include <MsTimer2.h>

void setup();
void loop();

void timer1_CB();
//void timer2_CB();

void motorCB(uint8_t pwm, bool directionCCW);

#ifdef SERIAL_DEBUGGING
uint8_t _serialDebugDivider;
MotorDebug _motorDebug;
void printDebug();
#endif

void sensorA_ISR();
void sensorB_ISR();

// pin assignments
int ledPin = 13;
int pwmPin = 11;    
int sensorA_Pin = 0; // pin 2 (UNO)
int sensorB_Pin = 1; // pin 3 (UNO)
int motorDirPin = 7;

uint8_t _pwm = 0;

// ISR globals
volatile long _sensorCount, _lastSensorCount;
volatile double _lastVelocity; 
volatile bool _lastSensorA, _lastSensorB;
volatile bool _directionCCW, _directionA_CCW, _directionB_CCW;

PositionController _positionController;

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

  Timer1.initialize(TIMER1_MICRO_SECONDS);
  Timer1.attachInterrupt(timer1_CB);

#ifdef SERIAL_DEBUGGING
  Serial.begin(115200);
  _serialDebugDivider = 0;
#endif

  double targetPosition = -6.28;
  double targetVelocity = 3;
  
  _positionController.setTargets(targetPosition, targetVelocity);
} 

void loop()  
{ 

#ifdef SERIAL_DEBUGGING
  if (_serialDebugDivider > 25) 
  {
    printDebug();
    _serialDebugDivider = 0;
  }
#endif

}

void timer1_CB() 
{
#ifdef SERIAL_DEBUGGING
_serialDebugDivider++;
#endif

  _positionController.moveToGoal( _sensorCount,
                                  _lastSensorCount,
                                  motorCB
                                  #ifdef SERIAL_DEBUGGING 
                                    ,_motorDebug 
                                  #endif
                                  );    

  _lastSensorCount = _sensorCount; 
}

void motorCB(uint8_t pwm, bool directionCCW)
{
  _pwm = pwm;

  digitalWrite(motorDirPin, !directionCCW); // CW
  analogWrite(pwmPin, _pwm);
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

#ifdef SERIAL_DEBUGGING

// void printDebug2()
// {
//   char *title_str = "a test title: ";
//   char *text_str;
//   char value_str[7];

//   text_str = (char *)malloc(strlen(title_str)+7);
//   strcpy(text_str, title_str);
//   dtostrf(_motorDebug.position, 4, 2, value_str);
//   strcat(text_str, value_str);
  
//   Serial.println(text_str);
//   free(text_str); 
// }

void printDebug()
{
    char serialDebugString[300];
    char position_str[7];
    char velocity_str[7];
    char acceleration_str[7];
    char position_error_str[7];
    char proportional_band_str[7];
    char adjusted_target_velocity_str[7];
    char pid_term_str[7];

    dtostrf(_motorDebug.position, 4, 2, position_str);
    dtostrf(_motorDebug.velocity, 4, 2, velocity_str);
    dtostrf(_motorDebug.acceleration, 4, 2, acceleration_str);
    dtostrf(_motorDebug.positionError, 4, 2, position_error_str);
    dtostrf(_motorDebug.proportionalBand, 4, 2, proportional_band_str);
    dtostrf(_motorDebug.adjustedTargetVelocity, 4, 2, adjusted_target_velocity_str);
    dtostrf(_motorDebug.pidTerm, 4, 2, pid_term_str);

    sprintf(serialDebugString,  "velocity = %s, position = %s, acceleration = %s, positionError = %s, directionCCW = %c, proportionalBand = %s, pidTerm = %s, adj targ Vel = %s, Sensor count = %ld, current = %d", 
              velocity_str,
              position_str,
              acceleration_str,
              position_error_str,
              _motorDebug.directionCCW == true ? '1' : '0',
              proportional_band_str,
              pid_term_str,
              adjusted_target_velocity_str,
              _sensorCount,
              analogRead(A0));

    Serial.println(serialDebugString);
}
#endif