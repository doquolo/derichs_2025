#include <Arduino.h>

#include <controller.hpp>
#include <motors.hpp>
#include <imu.hpp>

void setup()
{
  Serial.begin(115200);

  initIMU();

  initMotor();

  initController(); 
}

// GLOBAL MOVEMENT VARIABLES
int16_t currentAngle = 0;
int direction = 0;
int currentSpeed = 0;
int targetSpeed = 0;
int currentSpeedUse = 0;

// MODIFIABLE MOVEMENT VARIABLES
const int speed[3] = {50, 100, 150};
const int step[3] = {5, 10, 15};

// movement logic
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
    switch (abs(direction))
    {
    case 0:
    {
      if (currentSpeed > 0)
        forward(currentSpeed, calculateYawPID(currentAngle, -getIMUYaw()));
      else if (currentSpeed < 0)
        backward(abs(currentSpeed), calculateYawPID(currentAngle, -getIMUYaw()));
      break;
    }
    case 1:
    {
      if (direction > 0)
        sideRight(currentSpeed, calculateYawPID(currentAngle, -getIMUYaw()));
      else if (direction < 0)
        sideLeft(currentSpeed, calculateYawPID(currentAngle, -getIMUYaw()));
      break;
    }
    case 2:
    {
      if (direction > 0)
      {
        rotateRight(currentSpeed);
      }
      else if (direction < 0)
      {
        rotateLeft(currentSpeed);
      }
      currentAngle = -getIMUYaw();
      break;
    }
    case 3:
    {
      if (direction > 0) {
        driftRight(currentSpeed);
      } else if (direction < 0) {
        driftLeft(currentSpeed);
      }
      currentAngle = -getIMUYaw();
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

// periperals logic
void processPeriperals() { return; }

void loop()
{
  fetchController();
  
  if (connected) // if latency < 350ms
  {

    // Periperals
    processPeriperals();
    // end Periperals

    // Movement

    // Speed
    if (controller.RS.A12) currentSpeedUse = 0;
    else if (controller.RS.B9) currentSpeedUse = 1;
    else if (controller.RS.PB11) currentSpeedUse = 2;

    // CW/CCW
    if (controller.LD.A6) targetSpeed = speed[currentSpeedUse];
    else if (controller.LD.B2) targetSpeed = -speed[currentSpeedUse];
    else targetSpeed = 0;

    // side move
    // direction case:
    // 0: forward/backward
    // 1: side move
    // 2: spin
    // 3: drift
    if (controller.RD.B4) direction = 1;
    else if (controller.RD.B3) direction = -1;
    // spin
    else if (controller.LS.A3) direction = 2;
    else if (controller.RS.C13) direction = -2;
    // drift
    else if (controller.LS.A0) direction = 2;
    else if (controller.RS.A1) direction = -2;
    // forward/backward 
    else direction = 0;

    processMovement();

    // end Movement

  }
  else // latency > 350ms => might be disconnected
  {
    stop();
    while (1) // kill all robot movement
    {
      Serial.print("."); 
    }
  }
  delay(25);
}