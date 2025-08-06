#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <string.h>
#include <HardwareSerial.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x5c, 0x01, 0x3b, 0x9c, 0x57, 0xc4};
esp_now_peer_info_t robot_receiver;

void readMacAddress()
{
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK)
    {
        Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                      baseMac[0], baseMac[1], baseMac[2],
                      baseMac[3], baseMac[4], baseMac[5]);
    }
    else
    {
        Serial.println("Failed to read MAC address");
    }
}

// Structure to send data
#pragma pack(1) // Disable padding
struct CONTROLLER_READOUT
{
    struct
    {
        uint8_t A2, A3;
        uint8_t B8, B18, A4;
        uint8_t A0;
    } LS; // left shoulder
    struct
    {
        uint8_t A8, A11, B2, B12;
    } LD; // left dpad

    struct
    {
        uint8_t C18, A1;
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
uint8_t buffer[sizeof(controller)];

#pragma pack(1)
struct ROBOT_TELEMETRY
{
  uint8_t speed, handPos;
  uint16_t angle;
} robot;
#pragma pack()


// Constants for start and end markers
const uint8_t START_MARKER = 0x7E; // '~'
const uint8_t END_MARKER = 0x7F;   // DEL (127)

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    digitalWrite(18, !digitalRead(18));
    Serial.printf("Check? %i - Sent? %i\n", controller.LS.A0, status == ESP_NOW_SEND_SUCCESS);
    digitalWrite(18, !digitalRead(18));
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&robot, incomingData, len);
    Serial2.printf("S: %d, A: %i, H: %d\n", robot.speed, robot.angle, robot.handPos);
}

void setup()
{
    // Init Serial Monitor
    Serial.begin(115200);

    // init connection with stm32
    Serial2.begin(115200);

    // notify LED
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    Serial.print("[DEFAULT] Tranceiver MAC Address: ");
    readMacAddress();

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(robot_receiver.peer_addr, broadcastAddress, 6);
    robot_receiver.channel = 0;
    robot_receiver.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&robot_receiver) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    Serial.println("done");
}

void loop()
{
    while (Serial2.available())
    {
        if (Serial2.read() == START_MARKER)
        { // Detect the start marker
            // Manually discard the START_MARKER before reading the payload
            if (Serial2.available() >= sizeof(controller))
            {
                // Ensure enough data is available to read
                Serial2.readBytes(buffer, sizeof(controller));   // Read the payload
                memcpy(&controller, buffer, sizeof(controller)); // Copy to struct
                // Send message via ESP-NOW
                esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&controller, sizeof(controller));
                // logData(); // Log parsed data
            }
        }
    }
}