#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial BTSerial;

const char *BTname = "robotESP32";
const char *slaveName = "controllerESP32";

void initController() {
    // init bluetooth serial as master mode
    BTSerial.begin(BTname, true);
    // connect to slave
    bool connected = BTSerial.connect(slaveName);
    if (!connected)
    {
        Serial.print("Waiting for device.");
        while (!BTSerial.connected(30000))  {
            // 10 secs timeout (might be 30 if connect via slave's name)
            Serial.print(".");
            delay(250);
        }
    }
    Serial.printf("Connected to %s", slaveName);
}

String fetchController() {
    if (BTSerial.available() > 0)
    {
        // create buffer
        String buffer = "";
        // read to buffer until \n
        while (BTSerial.available() > 0)
        {
            int temp = BTSerial.read();
            if (temp == '\n')
                break;
            else
                buffer += (char)temp;
        }
        // parse command
        buffer.trim();
        return buffer;
    } 
    else return "-1";
}