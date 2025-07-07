#include <Arduino.h>

class ServoMotor
{
private:
  int angle = 0;
  int pulsePin, dirPin;
  int ppr, gearRatio, multiplier;
  int pulseDelay = 1000;

public:
  ServoMotor(int pulsePinNum, int dirPinNum, int pprNum, int gearRatioNum, int multiplierNum = 4)
  {
    pulsePin = pulsePinNum;
    dirPin = dirPinNum;
    ppr = pprNum;
    gearRatio = gearRatioNum;
    multiplier = multiplierNum;
  }
  void begin()
  {
    pinMode(this->pulsePin, OUTPUT);
    pinMode(this->dirPin, OUTPUT);
    digitalWrite(this->pulsePin, HIGH);
    digitalWrite(this->dirPin, LOW);
  }

  void write(int angle)
  {
    if (angle >= 0 && angle <= 360)
    {
      int diffStep = angle - this->angle;
      digitalWrite(this->dirPin, (int)(diffStep < 0));
      float stepPerDegree = (float) ((4.0*(float)this->ppr*(float)this->gearRatio)/((float)this->multiplier*360.0));
      for (int i = 0; i <= ceil((float) (abs(diffStep)*(stepPerDegree))); i++)
      {
        digitalWrite(this->pulsePin, LOW);
        delayMicroseconds(this->pulseDelay);
        digitalWrite(this->pulsePin, HIGH);
        delayMicroseconds(this->pulseDelay);
      }
      this->angle = angle;
    }
  }

  int getAngle() {
    return this->angle;
  }

  void setDriveSpeed(int speed)
  {
    this->pulseDelay = speed;
  }
};