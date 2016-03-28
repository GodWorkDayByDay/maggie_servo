int ledPin = 13;
int pwmPin = 11;    
int sensorA_Pin = 0; // pin 2 (UNO)
int sensorB_Pin = 1; // pin 3 (UNO)
int motorDirPin = 7;

volatile int _sensorCount;
volatile bool _lastSensorA, _lastSensorB;
volatile bool _directionCCW, _directionA_CCW, _directionB_CCW;

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

  _sensorCount = 0;
   analogWrite(pwmPin, 255); 

   digitalWrite(motorDirPin, LOW); // HIGH == CW
} 

void loop()  
{ 
  digitalWrite (ledPin, _directionCCW); 

  // test only!
  if (_sensorCount > 6220) { //63
      analogWrite(pwmPin, 0); 
  } else if (_sensorCount == 6000) {
      analogWrite(pwmPin, 20);
  }
}

void sensorA_ISR()
{ 
    _sensorCount++;
    _lastSensorA = PIND & 0x04; // atmega 328 specific!!

    _directionA_CCW = _lastSensorA == HIGH && _lastSensorB == LOW || _lastSensorA == LOW && _lastSensorB == HIGH;
    if (!(_directionA_CCW ^ _directionB_CCW))
    {
      _directionCCW = _directionA_CCW;
    }
}

void sensorB_ISR()
{
    _sensorCount++;
    _lastSensorB = PIND & 0x08; // atmega 328 specific!!

    _directionB_CCW = _lastSensorA == HIGH && _lastSensorB == HIGH || _lastSensorA == LOW && _lastSensorB == LOW;
    if (!(_directionA_CCW ^ _directionB_CCW))
    {
      _directionCCW = _directionB_CCW;
    }
}