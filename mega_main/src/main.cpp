#include <Arduino.h>

#include <controller.hpp>
#include <motors.hpp>
#include <imu.hpp>

// Servo
#include <ServoMotor.hpp>
ServoMotor s1(10, 9, 500, 14, 4);

void initServoMotor() {
  s1.begin();
  s1.setDriveSpeed(350);
}

// GLOBAL MOVEMENT VARIABLES
int16_t currentAngle = 0;
int direction = 0;
int currentSpeed = 0;
int targetSpeed = 0;
int currentSpeedUse = 1;

// MODIFIABLE MOVEMENT VARIABLES
const int speed[3] = {50, 100, 150};
const int step[3] = {3, 7, 10};

// GLOBAL VALVES TRIGGER
const int valveInterval = 250;
const int valveTrigger1[16] = {41, 39, 37, 35, 33, 31, 29, 27, 40, 38, 36, 34, 32, 30, 28, 26};
long long valveTime1[16] = {0};

// GLOBAL SENSORS
const int CB0 = 42;
const int CB90 = 45;

void initValves()
{
  for (auto x : valveTrigger1)
  {
    pinMode(x, OUTPUT);
    digitalWrite(x, LOW);
  }
  delay(50);
  for (auto x : valveTrigger1)
  {
    digitalWrite(x, HIGH);
  }
}

// GLOBAL AUX MOTORS
const int auxm1[2] = {14, 13}; // tời tay kíp nổ
const int auxm2[2] = {15, 16}; // tời tay quay

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);

  initIMU();

  initMotor();

  initServoMotor();

  initValves();

  initController();

  // testing
  pinMode(42, INPUT);
  pinMode(45, INPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(9, HIGH);
}

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
  Serial.print(currentSpeed);
  Serial.print("-");
  Serial.println(direction);
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
      if (direction > 0)
      {
        driftRight(currentSpeed);
      }
      else if (direction < 0)
      {
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
void processPeriperals()
{
  if (controller.RS.PB11) // tay quay
  {
    if (millis() - valveTime1[11] > valveInterval)
    {
      digitalWrite(valveTrigger1[11], LOW); // unlock
      Serial.println("Unlocked");
      s1.write((s1.getAngle() >= 90) ? 0 : 90);
      Serial.println("Locked");
      digitalWrite(valveTrigger1[11], HIGH); // lock
      valveTime1[11] = millis();
    }
  }
  // *** CYLINDER ***

  // tay chinh 1
  if (controller.RS.A5)
  {
    if (millis() - valveTime1[7] > valveInterval)
    {
      digitalWrite(valveTrigger1[7], !digitalRead(valveTrigger1[7]));
      valveTime1[7] = millis();
    }
  }
  // tay chinh 2
  if (controller.RS.A12)
  {
    if (millis() - valveTime1[6] > valveInterval)
    {
      digitalWrite(valveTrigger1[6], !digitalRead(valveTrigger1[6]));
      valveTime1[6] = millis();
    }
  }
  // tay chinh 3
  if (controller.RS.B9)
  {
    if (millis() - valveTime1[5] > valveInterval)
    {
      digitalWrite(valveTrigger1[5], !digitalRead(valveTrigger1[5]));
      valveTime1[5] = millis();
    }
  }
  // *** AUX MOTOR ***
  // toi tay ngoc
  if (controller.JOY.VALUE[0] > 4000)
  {
    if (millis() - valveTime1[auxm1[0] - 1] > valveInterval)
    {
      digitalWrite(valveTrigger1[auxm1[0] - 1], LOW);
      digitalWrite(valveTrigger1[auxm1[1] - 1], HIGH);
      valveTime1[auxm1[0] - 1] = millis();
    }
  }
  else if (controller.JOY.VALUE[0] < 20)
  {
    if (millis() - valveTime1[auxm1[0] - 1] > valveInterval)
    {
      digitalWrite(valveTrigger1[auxm1[0] - 1], HIGH);
      digitalWrite(valveTrigger1[auxm1[1] - 1], LOW);
      valveTime1[auxm1[0] - 1] = millis();
    }
  }
  else
  {
    digitalWrite(valveTrigger1[auxm1[0] - 1], HIGH);
    digitalWrite(valveTrigger1[auxm1[1] - 1], HIGH);
  }
  // toi tay chinh
  if (controller.RD.B5)
  {
    if (millis() - valveTime1[auxm2[0] - 1] > valveInterval)
    {
      digitalWrite(valveTrigger1[auxm2[0] - 1], LOW);
      digitalWrite(valveTrigger1[auxm2[1] - 1], HIGH);
      valveTime1[auxm2[0] - 1] = millis();
    }
  }
  else if (controller.RD.A15)
  {
    if (millis() - valveTime1[auxm2[0] - 1] > valveInterval)
    {
      digitalWrite(valveTrigger1[auxm2[0] - 1], HIGH);
      digitalWrite(valveTrigger1[auxm2[1] - 1], LOW);
      valveTime1[auxm2[0] - 1] = millis();
    }
  }
  else
  {
    digitalWrite(valveTrigger1[auxm2[0] - 1], HIGH);
    digitalWrite(valveTrigger1[auxm2[1] - 1], HIGH);
  }
  // tay kho ngoc
  if (controller.JOY.BTN) {
    if (millis() - valveTime1[4] > valveInterval)
    {
      digitalWrite(valveTrigger1[4], !digitalRead(valveTrigger1[4]));
      valveTime1[4] = millis();
    }
  }
   // tay ngoc 1
   if (controller.RD.B3)
   {
    if (millis() - valveTime1[3] > valveInterval)
    {
      digitalWrite(valveTrigger1[3], !digitalRead(valveTrigger1[3]));
      valveTime1[3] = millis();
    }
  }
  // tay ngoc 2
  if (controller.RD.B4)
  {
    if (millis() - valveTime1[2] > valveInterval)
    {
      digitalWrite(valveTrigger1[2], !digitalRead(valveTrigger1[2]));
      valveTime1[2] = millis();
    }
  }
  if (!controller.ENC.BTN) {
    if (millis() - valveTime1[1] > valveInterval)
    {
      digitalWrite(valveTrigger1[1], !digitalRead(valveTrigger1[1]));
      valveTime1[1] = millis();
    }  
  }
  if (controller.ALT.C14) {
    if (millis() - valveTime1[0] > valveInterval)
    {
      digitalWrite(valveTrigger1[0], !digitalRead(valveTrigger1[0]));
      valveTime1[0] = millis();
    }  
  }
}

void loop()
{
  fetchController();
  // sendTelemetry();

  if (connected) // if latency < 350ms
  {

    // Periperals
    processPeriperals();
    // end Periperals

    // Movement

    // Speed
    if (controller.LS.B8)
      currentSpeedUse = 0;
    else if (controller.LS.B13)
      currentSpeedUse = 1;
    else if (controller.LS.A4)
      currentSpeedUse = 2;

    // side move
    // direction case:
    // 0: forward/backward
    // 1: side move
    // 2: spin
    // 3: drift
    // side move
    if (controller.LD.A11)
    {
      direction = 1;
      targetSpeed = speed[currentSpeedUse];
    }
    else if (controller.LD.B12)
    {
      direction = -1;
      targetSpeed = speed[currentSpeedUse];
    }
    // spin
    else if (controller.LS.A3)
    {
      direction = 2;
      targetSpeed = speed[currentSpeedUse];
    }
    else if (controller.RS.C13)
    {
      direction = -2;
      targetSpeed = speed[currentSpeedUse];
    }
    else
    {
      // CW/CCW
      // forward/backward
      if (controller.LD.A8) {
        targetSpeed = speed[currentSpeedUse];
        direction = 0;
      }
      else if (controller.LD.B2) {
        targetSpeed = -speed[currentSpeedUse];
        direction = 0;
      }
      else
      targetSpeed = 0;
      // drift
      if (controller.LS.A2)
        direction = 3;
      else if (controller.RS.A1)
        direction = -3;
    }

    processMovement();

    // end Movement
  }
  else // latency > 350ms => might be disconnected
  {
    stop();
    while (Serial3.available() > 0)
    {
      Serial3.read();
      delay(25);
    }
    while (true) // kill all robot movement
    {
      fetchController();
      if (connected)
        break;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(50);
    }
  }
  delay(25);
}