#include <SPI.h>
#include "CANBusManager.h"

#define CAN_ID 0x10
#define CAN_CS_PIN 10
#define CAN_INT_PIN 2

CANBusManager canBus(CAN_ID, CAN_CS_PIN, CAN_INT_PIN);
unsigned long lastHeartbeat = 0;

void OnMessage(const CANMessage& msg) {
    MessageCode code = (MessageCode)msg.data[0];
    switch (code) {
        case HEARTBEAT:
            Serial.println("Client A: Received Heartbeat");
            canBus.SendBool(TOGGLE_LED, true);
            break;
        case TOGGLE_LED:
            Serial.print("Client A: Toggle LED - ");
            Serial.println(CANBusManager::ReadBool(msg));
            break;
        case TEMPERATURE:
            Serial.print("Client A: Temperature = ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case STATUS_FLOAT:
            Serial.print("Client A: Status float = ");
            Serial.println(CANBusManager::ReadFloat(msg));
            break;
    }
}

void OnStringReceived(MessageCode code, const char* str) {
    Serial.print("Client A: String received [");
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
        canBus.SendInt(HEARTBEAT, 123);
        canBus.SendInt(TEMPERATURE, 26);
        canBus.SendFloat(STATUS_FLOAT, 3.14);
        canBus.SendString(DESCRIPTION, "Client A is alive and happy.");
        lastHeartbeat = millis();
    }
}