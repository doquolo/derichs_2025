#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// REPLACE WITH YOUR RECEIVER MAC Address
// a0:b7:65:05:8b:6c
// cc:db:a7:2e:af:dc
// 5c:01:3b:9c:57:c4
// uint8_t broadcastAddress[] = {0xa0, 0xb7, 0x65, 0x05, 0x8b, 0x6c};
// uint8_t broadcastAddress[] = {0xcc, 0xdb, 0xa7, 0x2e, 0xaf, 0xdc};
uint8_t broadcastAddress[] = {0x5c, 0x01, 0x3b, 0x9c, 0x57, 0xc4};

// Structure example to send data
#pragma pack(1) // Disable padding
struct CONTROLLER_READOUT {
	struct {
		uint8_t A0, A3;
		uint8_t PB10, A4, B13;
		uint8_t A5;
	}LS; // left shoulder
	struct {
		uint8_t B12, A6, B2, A7;
	}LD; // left dpad

	struct {
		uint8_t C13, A1;
		uint8_t A12, B9, PB11;
		uint8_t B8;
	}RS; // right shoulder
	struct {
		uint8_t B3, B5, A15, B4;
	}RD; // right dpad

	struct {
		uint8_t A2, C15, C14;
	}ALT; // alternate button

	struct {
		uint8_t BTN;
		uint16_t VALUE[2];
	}JOY; // joystick

	struct {
		uint8_t BTN;
		uint8_t VALUE;
	}ENC; // encoder

} controller;
#pragma pack()

// Constants for start and end markers
const uint8_t START_MARKER = 0x7E; // '~'
const uint8_t END_MARKER = 0x7F;   // DEL (127)
uint8_t buffer[sizeof(controller)];

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Check? %i - Sent? %i\n", controller.LS.A0, status == ESP_NOW_SEND_SUCCESS);
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // init connection with stm32
  Serial2.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println("ESP-NOW start");
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
    Serial.println("done");
}
 
void loop() {
    while (Serial2.available()) {
        if (Serial2.read() == START_MARKER) { // Detect the start marker
            // Manually discard the START_MARKER before reading the payload
            if (Serial2.available() >= sizeof(controller)) { 
                // Ensure enough data is available to read
                Serial2.readBytes(buffer, sizeof(controller)); // Read the payload
                memcpy(&controller, buffer, sizeof(controller)); // Copy to struct
                // Send message via ESP-NOW
                esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &controller, sizeof(controller));
                // logData(); // Log parsed data
            }
        }
    }
}