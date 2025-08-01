#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <string.h>

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

// Constants for start and end markers
const uint8_t START_MARKER = 0x7E; // '~'
const uint8_t END_MARKER = 0x7F;   // DEL (127)

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  digitalWrite(13, !digitalRead(13));
  Serial2.write(START_MARKER);
  for (int i = 0; i < len; i++) Serial2.write(incomingData[i]);
  digitalWrite(13, !digitalRead(13));
}

void initController()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
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
}