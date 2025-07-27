#include <Arduino.h>

#include <controller.hpp>
#include <motors.hpp>
#include <imu.hpp>

void setup()
{
  Serial.begin(115200);
  // initIMU(); // init imu

  initMotor(); // init motor driver

  initController(); // init controller
}

int direction = 0;
int currentSpeed = 0;
int16_t currentAngle = 0;

int targetSpeed = 0;
const int speed[3] = {50, 100, 150};
const int step[3] = {5, 10, 15};
int currentSpeedUse = 0;

void processMovement()
{
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
        forward(currentSpeed, 0);
      else if (currentSpeed < 0)
        backward(abs(currentSpeed), 0);
      break;
    }
    case 1:
    {
      if (currentSpeed > 0)
        sideRight(currentSpeed, 0);
      else if (currentSpeed < 0)
        sideLeft(abs(currentSpeed), 0);
      break;
    }
    case 2:
    {
      if (currentSpeed > 0)
      {
        rotateRight(currentSpeed);
        currentAngle = getIMUYaw();
      }
      else if (currentSpeed < 0)
      {
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
}

void processPeriperals() { return; }

void loop()
{
  fetchController();

  if (connected)
  {
    // Movement
    if (controller.LD.A6)
    {
      direction = 0;
      targetSpeed = speed[currentSpeedUse];
    }
    else if (controller.LD.B2)
    {
      direction = 0;
      targetSpeed = -speed[currentSpeedUse];
    }
    else if (controller.RD.B4)
    {
      direction = 1;
      targetSpeed = speed[currentSpeedUse];
    }
    else if (controller.RD.B3)
    {
      direction = 1;
      targetSpeed = -speed[currentSpeedUse];
    }
    else {
      direction = 0;
      targetSpeed = 0;
    }
    Serial.print("DIR: "); Serial.print(direction);
    Serial.print(" SPEED: "); Serial.println(targetSpeed);
    processMovement();
  }
  else
  {
    stop();
    while (1)
    {
      Serial.print(".");
    }
  }
  delay(25);
}