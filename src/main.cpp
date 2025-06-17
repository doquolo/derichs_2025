#include <Arduino.h>
#include <HardwareSerial.h>
#include <bits/stdc++.h>

#include <motors.hpp>
#include <imu.hpp>

void setup()
{
  Serial.begin(115200);
  initIMU(); // init imu

  initMotor(); // init motor driver

  // wait for BOOT btn to be pressed
  pinMode(0, INPUT);
  while (1) {
    if (digitalRead(0) == 0) break;
    delay(25);
  }
}


void loop()
{
  long long int startTime = millis();
  while (millis() - startTime <= 5000) {
    if (currentSpeed <= targetSpeed) currentSpeed += step;
    forward(currentSpeed, calculateYawPID(0.0, getIMUYaw()));
    delay(50);
  }
  stop();
  delay(500);
  startTime = millis();
  while (millis() - startTime <= 5000) {
    if (currentSpeed <= targetSpeed) currentSpeed += step;
    backward(currentSpeed, calculateYawPID(0.0, getIMUYaw()));
    delay(50);
  }
  stop();
  delay(500);
}