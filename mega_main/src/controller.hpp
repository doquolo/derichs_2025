#include <Arduino.h>

// Structure example to send data
#pragma pack(1) // Disable padding
struct CONTROLLER_READOUT
{
    struct
    {
        uint8_t A2, A3;
        uint8_t B8, B13, A4;
        uint8_t A0;
    } LS; // left shoulder
    struct
    {
        uint8_t A8, A11, B2, B12;
    } LD; // left dpad

    struct
    {
        uint8_t C13, A1;
        uint8_t A5, A12, B9;
        uint8_t PB11;
    } RS; // right shoulder
    struct
    {
        uint8_t B5, B4, A15, B3;
    } RD; // right dpad

    struct
    {
        uint8_t C15, C14;
    } ALT; // alternate button

    struct
    {
        uint8_t BTN;
        uint16_t VALUE[2];
    } JOY; // joystick

    struct
    {
        uint8_t BTN;
        uint8_t VALUE;
    } ENC; // encoder

} controller;
#pragma pack()
uint8_t bufferController[sizeof(controller)];

#pragma pack(1)
struct ROBOT_TELEMETRY
{
    uint8_t speed, handPos;
    uint16_t angle;
} robot;
#pragma pack()
uint8_t bufferRobotTelemetry[sizeof(robot)];

// Constants for start and end markers
const uint8_t START_MARKER = 0x7E; // '~'
const uint8_t END_MARKER = 0x7F;   // DEL (127)
uint8_t buffer[sizeof(controller)];

void resetController()
{
    // Left shoulder
    controller.LS.A2 = 0;
    controller.LS.A3 = 0;
    controller.LS.B8 = 0;
    controller.LS.B13 = 0;
    controller.LS.A4 = 0;
    controller.LS.A0 = 0;

    // Left dpad
    controller.LD.A8 = 0;
    controller.LD.A11 = 0;
    controller.LD.B2 = 0;
    controller.LD.B12 = 0;

    // Right shoulder
    controller.RS.C13 = 0;
    controller.RS.A1 = 0;
    controller.RS.A5 = 0;
    controller.RS.A12 = 0;
    controller.RS.B9 = 0;
    controller.RS.PB11 = 0;

    // Right dpad
    controller.RD.B5 = 0;
    controller.RD.B4 = 0;
    controller.RD.A15 = 0;
    controller.RD.B3 = 0;

    // Alternate buttons
    controller.ALT.C15 = 0;
    controller.ALT.C14 = 0;

    // Joystick
    controller.JOY.BTN = 0;
    controller.JOY.VALUE[0] = 0;
    controller.JOY.VALUE[1] = 0;

    // Encoder
    controller.ENC.BTN = 1; // Preserved your original design
    controller.ENC.VALUE = 0;
}

long long int prev_data_recv = 0;
const int controller_timeout = 500;
bool connected = 0;

void initController()
{
    Serial3.begin(115200);
    resetController();
    prev_data_recv = millis(); // important
    connected = 1;
}

void checkControllerTimeout()
{
    if (millis() - prev_data_recv >= controller_timeout)
    {
        resetController();
        connected = 0;
    }
}

void fetchController()
{
    while (Serial3.available())
    {
        if (Serial3.read() == START_MARKER)
        { // Detect the start marker
            // Manually discard the START_MARKER before reading the payload
            if (Serial3.available() >= sizeof(controller))
            {
                // Ensure enough data is available to read
                Serial3.readBytes(bufferController, sizeof(controller));   // Read the payload
                memcpy(&controller, bufferController, sizeof(controller)); // Copy to struct
                prev_data_recv = millis();
                connected = 1;
            }
        }
    }
    checkControllerTimeout();
}

void sendTelemetry()
{
    // update
    robot.angle = currentAngle;
    robot.speed = speed[currentSpeedUse];
    robot.handPos = (digitalRead(CB0) ? 1 : 0);
    
    // send data
    memcpy(&robot, bufferRobotTelemetry, sizeof(robot));
    // Send message via ESP-NOW
    for (auto x: bufferRobotTelemetry) {
        Serial3.write(x);
    }
}