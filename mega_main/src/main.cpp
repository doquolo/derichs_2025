#include <Arduino.h>

#include <controller.hpp>
#include <motors.hpp>
#include <imu.hpp>

// // Servo
// #include <ServoMotor.hpp>
// ServoMotor s1(10, 9, 13, 19.5, 4);

// void initServoMotor() {
//   s1.begin();
//   s1.setDriveSpeed(3500);
// }

// GLOBAL MOVEMENT VARIABLES
int16_t currentAngle = 0;
int direction = 0;
int currentSpeed = 0;
int targetSpeed = 0;
int currentSpeedUse = 0;

// MODIFIABLE MOVEMENT VARIABLES
const int speed[3] = {50, 100, 150};
const int step[3] = {5, 10, 15};

// GLOBAL VALVES TRIGGER
const int valveInterval = 250;
const int valveTrigger1[16] = {41, 39, 37, 35, 33, 31, 29, 27, 40, 38, 36, 34, 32, 30, 28, 26};
long long valveTime1[16] = {0};

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
const int auxm1[2] = {15, 16}; // tời tay quay
const int auxm2[2] = {14, 13}; // tời tay kíp nổ

void setup()
{
  Serial.begin(115200);

  initIMU();

  initMotor();

  // initServoMotor();

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
  if (controller.LD.A7)
  {
    if (millis() - valveTime1[11] > valveInterval)
    {
      digitalWrite(valveTrigger1[11], LOW); // unlock
      Serial.println("Unlocked");
      if (!digitalRead(45))
      { // go up
        Serial.println("Going up");
        digitalWrite(9, HIGH);
        analogWrite(10, 255 - 10);
        while (digitalRead(42))
        {
          delay(25);
        }
        analogWrite(10, 255 - 3);

        Serial.println("Done");
      }
      else if (!digitalRead(42))
      { // drop down
        Serial.println("Going down");
        digitalWrite(9, LOW);
        analogWrite(10, 255 - 10);
        while (digitalRead(45))
        {
          delay(25);
        }
        digitalWrite(10, HIGH);
        Serial.println("Done");
      }
      Serial.println("Locked");
      digitalWrite(valveTrigger1[11], HIGH); // lock
      valveTime1[11] = millis();
    }
  }
  // *** CYLINDER ***
  if (controller.RS.A12)
  {
    if (millis() - valveTime1[7] > valveInterval)
    {
      digitalWrite(valveTrigger1[7], !digitalRead(valveTrigger1[7]));
      valveTime1[7] = millis();
    }
  }
  if (controller.RS.B9)
  {
    if (millis() - valveTime1[6] > valveInterval)
    {
      digitalWrite(valveTrigger1[6], !digitalRead(valveTrigger1[6]));
      valveTime1[6] = millis();
    }
  }
  if (controller.RS.PB11)
  {
    if (millis() - valveTime1[5] > valveInterval)
    {
      digitalWrite(valveTrigger1[5], !digitalRead(valveTrigger1[5]));
      valveTime1[5] = millis();
    }
  }

  // *** AUX MOTOR ***
  // AUX1: tời tay quay
  if (controller.ALT.C15)
  {
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
  }
  else
  {
    if (controller.RD.B5)
    {
      if (millis() - valveTime1[auxm1[0] - 1] > valveInterval)
      {
        digitalWrite(valveTrigger1[auxm1[0] - 1], LOW);
        digitalWrite(valveTrigger1[auxm1[1] - 1], HIGH);
        valveTime1[auxm1[0] - 1] = millis();
      }
    }
    else if (controller.RD.A15)
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
  }
}

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
    if (controller.LS.PB10)
      currentSpeedUse = 0;
    else if (controller.LS.A4)
      currentSpeedUse = 1;
    else if (controller.LS.B13)
      currentSpeedUse = 2;

    // CW/CCW
    if (controller.LD.A6)
      targetSpeed = speed[currentSpeedUse];
    else if (controller.LD.B2)
      targetSpeed = -speed[currentSpeedUse];
    else
      targetSpeed = 0;

    // side move
    // direction case:
    // 0: forward/backward
    // 1: side move
    // 2: spin
    // 3: drift
    if (controller.RD.B4)
      direction = 1;
    else if (controller.RD.B3)
      direction = -1;
    // spin
    else if (controller.LS.A3)
      direction = 2;
    else if (controller.RS.C13)
      direction = -2;
    // drift
    else if (controller.LS.A0)
      direction = 2;
    else if (controller.RS.A1)
      direction = -2;
    // forward/backward
    else
      direction = 0;

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