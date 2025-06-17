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

int targetSpeed = 0;
const int step = 10;
int currentSpeed = 0;

void loop()
{
  String currentCommand = fetchController();
  if (currentCommand == "F")
  {
    Serial.println("Forward");
    // targetSpeed = 50;
    forward(50, calculateYawPID(0, getIMUYaw()));

  }
  else if (currentCommand == "B")
  {
    Serial.println("Backward");
    // targetSpeed = -50;
    backward(50, calculateYawPID(0, getIMUYaw()));

  }
  else stop();
  delay(50);
    // targetSpeed = 0;

  // if (currentSpeed != targetSpeed)
  // {
  //   if (currentSpeed < targetSpeed)
  //     currentSpeed += step;
  //   else if (currentSpeed > targetSpeed)
  //     currentSpeed -= step;

  //   // output to motor
  //   if (currentSpeed > 0)
  //     forward(currentSpeed, getIMUYaw());
  //   else if (currentSpeed < 0)
  //     backward(currentSpeed, getIMUYaw());
  //   else if (currentSpeed == 0)
  //     stop();
  // }
}