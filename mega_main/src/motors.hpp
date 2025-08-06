#include <Arduino.h>

const int DIR4 = 7;
const int PWM4 = 8;
const int DIR3 = 5;
const int PWM3 = 6;
const int DIR2 = 3;
const int PWM2 = 4;
const int DIR1 = 44;
const int PWM1 = 2;

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

void sideLeft(int speed, int pid = 0)
{
    m1((speed - pid));
    m2(-(speed - pid));
    m3((speed + pid));
    m4(-(speed + pid));
}

void sideRight(int speed, int pid = 0)
{
    m1(-(speed + pid));
    m2((speed + pid));
    m3(-(speed - pid));
    m4((speed - pid));
}

void rotateRight(int speed) {
    m1(speed);
    m2(-speed);
    m3(-speed);
    m4(speed);
}

void rotateLeft(int speed) {
    m1(-speed);
    m2(speed);
    m3(speed);
    m4(-speed);
}

void driftRight(int speed) {
    m1(speed*(1));
    m2(speed*(0));
    m3(speed*(0.5));
    m4(speed*1.5);
}
void driftLeft(int speed) {
    m1(speed*(0));
    m2(speed*(1));
    m3(speed*1.5);
    m4(speed*(0.5));
}

void stop()
{
    m1(1);
    m2(1);
    m3(1);
    m4(1);
}

void initMotor()
{
    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR3, OUTPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(DIR4, OUTPUT);
    pinMode(PWM4, OUTPUT);
    stop();
}

