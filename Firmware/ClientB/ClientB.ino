#include <SPI.h>
#include "CANBusManager.h"

#define CAN_ID 0x20
#define CAN_CS_PIN 9
#define CAN_INT_PIN 2

CANBusManager canBus(CAN_ID, CAN_CS_PIN, CAN_INT_PIN);
unsigned long lastHeartbeat = 0;

void OnMessage(const CANMessage& msg) {
    MessageCode code = (MessageCode)msg.data[0];
    switch (code) {
        case HEARTBEAT:
            Serial.println("Client B: Received Heartbeat");
            canBus.SendInt(HEARTBEAT, 456);
            canBus.SendBool(TOGGLE_LED, false);
            break;
        case TOGGLE_LED:
            Serial.print("Client B: Toggle LED - ");
            Serial.println(CANBusManager::ReadBool(msg));
            break;
        case TEMPERATURE:
            Serial.print("Client B: Temperature = ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case STATUS_FLOAT:
            Serial.print("Client B: Status float = ");
            Serial.println(CANBusManager::ReadFloat(msg));
            break;
    }
}

void OnStringReceived(MessageCode code, const char* str) {
    Serial.print("Client B: String received [");
    Serial.print((int)code);
    Serial.print("] -> ");
    Serial.println(str);
}

void setup() {
    Serial.begin(115200);
    canBus.Begin();
    canBus.SetCallback(OnMessage);
    canBus.SetStringCallback(OnStringReceived);
}

void loop() {
    canBus.Update();
    if (millis() - lastHeartbeat > 3000) {
        canBus.SendInt(HEARTBEAT, 999);
        canBus.SendInt(TEMPERATURE, 28);
        canBus.SendFloat(STATUS_FLOAT, 6.28);
        canBus.SendString(DESCRIPTION, "Client B operational.");
        lastHeartbeat = millis();
    }
}