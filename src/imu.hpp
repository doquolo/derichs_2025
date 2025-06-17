#include <Arduino.h>
#include <HardwareSerial.h>
#include <bits/stdc++.h>

void initIMU()
{
    Serial2.begin(115200);
    Serial2.write('a'); // calibrating
    delay(1000);
    // done imu
}

int16_t getIMUYaw() {
    Serial2.write('z'); // get imu data
    Serial2.flush(); // wait until current transmittion is completed
    return (Serial2.read() << 8) | Serial2.read(); // get yaw value (16bit signed int)
}

float Kp = .6, Ki = 0.0, Kd = .6;
float currentYaw, error, prevError = 0, integral = 0;

int calculateYawPID(int16_t target, int16_t yaw) {
  currentYaw = ((float) yaw) / 10.0; // yaw value = (yaw angle * 10)
  error = target - currentYaw;
  integral += error;
  float derivative = error - prevError;
  prevError = error;
  float pidOutput = (float)(Kp * error) + (float)(Ki * integral) + (float)(Kd * derivative);
  Serial.printf("Yaw: %f, Error: %f\n", currentYaw, pidOutput); // debug only
  return floor(pidOutput);
}