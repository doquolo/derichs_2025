#include <Arduino.h>

#include <controller.hpp>
#include <motors.hpp>
#include <imu.hpp>

void setup()
{
  Serial.begin(115200);
  initIMU(); // init imu

  initMotor(); // init motor driver

  // init bluetooth serial as master mode
  initController(); // init controller
}

long long int timeFromLastPacket = 0;
int direction = 0;
int currentSpeed = 0;
int16_t currentAngle = 0;

int targetSpeed = 0;
const int speed[3] = {50, 100, 150};
const int step[3] = {5, 10, 15};
int currentSpeedUse = 0;

void loop()
{
  String currentCommand = fetchController();
  Serial.printf("Angle: %i\n", currentAngle);
  // Serial.printf("Command: %s\n", currentCommand);
  if (currentCommand != "-1")
  {
    timeFromLastPacket = millis();
    if (currentCommand == "F")
    {
      direction = 0;
      targetSpeed = speed[currentSpeedUse];
    }
    else if (currentCommand == "B")
    {
      direction = 0;
      targetSpeed = -speed[currentSpeedUse];
    }
    else if (currentCommand == "L")
    {
      direction = 1;
      targetSpeed = -speed[currentSpeedUse];
    }
    else if (currentCommand == "R")
    {
      direction = 1;
      targetSpeed = speed[currentSpeedUse];
    }
    else if (currentCommand == "SR")
    {
      direction = 2;
      targetSpeed = speed[currentSpeedUse];
    }
    else if (currentCommand == "SL")
    {
      direction = 2;
      targetSpeed = -speed[currentSpeedUse];
    }
    else if (currentCommand == "S1") currentSpeedUse = 0; 
    else if (currentCommand == "S2") currentSpeedUse = 1; 
    else if (currentCommand == "S3") currentSpeedUse = 2; 
    else if (currentCommand == "")
    {
      targetSpeed = 0;
    }
  }
  else
  {
    if (millis() - timeFromLastPacket >= 1000)
    {
      Serial.print("Controller might disconnected.");
      stop();
      while (fetchController() == "-1")
      {
        Serial.print(".");
        delay(25);
      }
      Serial.println("\nController reconnected");
    }
  }

  if (currentSpeed != targetSpeed)
  {
    if (currentSpeed < targetSpeed)
      currentSpeed += step[currentSpeedUse];
    else if (currentSpeed > targetSpeed)
      currentSpeed -= step[currentSpeedUse];
  }

  // output to motor
  if (currentSpeed == 0)
    stop();
  else
  {
    switch (direction)
    {
    case 0:
    {
      if (currentSpeed > 0)
        forward(currentSpeed, calculateYawPID(currentAngle, getIMUYaw()));
      else if (currentSpeed < 0)
        backward(abs(currentSpeed), calculateYawPID(currentAngle, getIMUYaw()));
      break;
    }
    case 1:
    {
      if (currentSpeed > 0)
        sideRight(currentSpeed, calculateYawPID(currentAngle, getIMUYaw()));
      else if (currentSpeed < 0)
        sideLeft(abs(currentSpeed), calculateYawPID(currentAngle, getIMUYaw()));
      break;
    }
    case 2:
    {
      if (currentSpeed > 0) {
        rotateRight(currentSpeed);
        currentAngle = getIMUYaw();
      }
      else if (currentSpeed < 0) {
        rotateLeft(abs(currentSpeed));
        currentAngle = getIMUYaw();
      }
      break;
    }
    default:
    {
      stop();
      break;
    }
    }
  }
  delay(25);
}