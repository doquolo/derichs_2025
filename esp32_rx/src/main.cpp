#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <string.h>

// REPLACE WITH YOUR TRANCEIVER MAC Address
uint8_t broadcastAddress[] = {0x5c, 0x01, 0x3b, 0x9c, 0x57, 0xc4};
esp_now_peer_info_t robot_tranceiver;

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

#pragma pack(1)
struct ROBOT_TELEMETRY
{
  uint8_t speed, handPos;
  uint16_t angle;
} robot;
#pragma pack()
uint8_t buffer[sizeof(robot)];

// Constants for start and end markers
const uint8_t START_MARKER = 0x7E; // '~'
const uint8_t END_MARKER = 0x7F;   // DEL (127)

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  digitalWrite(13, !digitalRead(13));
  Serial2.write(START_MARKER);
  for (int i = 0; i < len; i++)
    Serial2.write(incomingData[i]);
  digitalWrite(13, !digitalRead(13));
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  return;
}

void initController()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Serial.print("[DEFAULT] Receiver MAC Address: ");
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
  memcpy(robot_tranceiver.peer_addr, broadcastAddress, 6);
  robot_tranceiver.channel = 0;
  robot_tranceiver.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&robot_tranceiver) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Serial.println("done");
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  initController();
}

void loop()
{
  while (Serial2.available())
  {
    if (Serial2.read() == START_MARKER)
    { // Detect the start marker
      // Manually discard the START_MARKER before reading the payload
      if (Serial2.available() >= sizeof(robot))
      {
        // Ensure enough data is available to read
        Serial2.readBytes(buffer, sizeof(robot)); // Read the payload
        memcpy(&robot, buffer, sizeof(robot));    // Copy to struct
        // Send message via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&robot, sizeof(robot));
        // logData(); // Log parsed data
      }
    }
  }
}