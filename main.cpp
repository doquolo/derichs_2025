#include <Arduino.h>
#include <HardwareSerial.h>
#include <bits/stdc++.h>

const int DIR1 = 15;
const int PWM1 = 2;
const int DIR2 = 33;
const int PWM2 = 32;
const int DIR3 = 26;
const int PWM3 = 25;
const int DIR4 = 14;
const int PWM4 = 27;

void m1(int speed)
{
  if (speed > 255)
  {
    speed = 255;
  }
  if (speed < -255)
  {
    speed = -255;
  }
  if (speed > 0)
  {
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, 255 - abs(speed));
  }
  else
  {
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, 255 - abs(speed));
  }
}
void m2(int speed)
{
  if (speed > 255)
  {
    speed = 255;
  }
  if (speed < -255)
  {
    speed = -255;
  }
  if (speed > 0)
  {
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, 255 - abs(speed));
  }
  else
  {
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, 255 - abs(speed));
  }
}

void m3(int speed)
{
  if (speed > 255)
  {
    speed = 255;
  }
  if (speed < -255)
  {
    speed = -255;
  }
  if (speed > 0)
  {
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, 255 - abs(speed));
  }
  else
  {
    digitalWrite(DIR3, HIGH);
    analogWrite(PWM3, 255 - abs(speed));
  }
}
void m4(int speed)
{
  if (speed > 255)
  {
    speed = 255;
  }
  if (speed < -255)
  {
    speed = -255;
  }
  if (speed > 0)
  {
    digitalWrite(DIR4, HIGH);
    analogWrite(PWM4, 255 - abs(speed));
  }
  else
  {
    digitalWrite(DIR4, LOW);
    analogWrite(PWM4, 255 - abs(speed));
  }
}

void forward(int speed, int pid = 0)
{
  m1(speed - pid);
  m2(speed + pid);
  m3(speed + pid);
  m4(speed - pid);
}

void backward(int speed, int pid = 0)
{
  m1(-(speed + pid));
  m2(-(speed - pid));
  m3(-(speed - pid));
  m4(-(speed + pid));
}

void turnRight(int speed, int pid = 0)
{
  m1((speed - pid));
  m2(-(speed + pid));
  m3((speed + pid));
  m4(-(speed - pid));
}

void turnLeft(int speed, int pid = 0)
{
  m1(-(speed + pid));
  m2((speed - pid));
  m3(-(speed - pid));
  m4((speed + pid));
}

void stop()
{
  m1(1);
  m2(1);
  m3(1);
  m4(1);
}

void setup()
{
  // put your setup code here, to run once:
  Serial2.begin(115200);
  Serial.begin(115200);
  Serial2.write('a'); // calibrating
  delay(1000);
  // done imu
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(PWM4, OUTPUT);
  stop();

  pinMode(0, INPUT);
  while (1) {
    if (digitalRead(0) == 0) break;
    delay(25);
  }
}

float Kp = .7, Ki = 0.0, Kd = .1;
int calculatePID(int16_t target, float yaw) {
  float currentYaw, error, prevError = 0, integral = 0;
  currentYaw = (float) yaw / 10.0;
  error = target - currentYaw;
  integral += error;
  float derivative = error - prevError;
  float pidOutput = (float)(Kp * error) + (float)(Ki * integral) + (float)(Kd * derivative);
  Serial.printf("Yaw: %f, Error: %f\n", currentYaw, pidOutput);
  return floor(pidOutput);
}

void loop()
{
  long long int startTime = millis();
  while (millis() - startTime <= 5000) {
    Serial2.write('z');
    Serial2.flush();
    int16_t yaw = (Serial2.read() << 8) | Serial2.read();
    forward(20, calculatePID(0.0, yaw));
    delay(100);
  }
  stop();
  delay(500);
  startTime = millis();
  while (millis() - startTime <= 5000) {
    Serial2.write('z');
    Serial2.flush();
    int16_t yaw = (Serial2.read() << 8) | Serial2.read();
    backward(20, calculatePID(0.0, yaw));
    delay(100);
  }
  stop();
  delay(500);
}