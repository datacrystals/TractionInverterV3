#ifndef CANBUS_DRIVER_H
#define CANBUS_DRIVER_H

#include <SPI.h>
#include <mcp2515.h>

class CanbusDriver {
private:
    MCP2515 mcp2515;
    uint8_t csPin;
    uint8_t intPin;
    volatile bool messageReceived;
    uint8_t deviceId;
    bool debugMode;

    static CanbusDriver* instance;

    static void handleInterrupt() {
        if (instance) {
            instance->messageReceived = true;
        }
    }

public:
    CanbusDriver(uint8_t csPin, uint8_t intPin, uint8_t deviceId = 0x01) : 
        mcp2515(csPin), csPin(csPin), intPin(intPin), deviceId(deviceId), 
        messageReceived(false), debugMode(false) {
        instance = this;
    }

    void setDebug(bool enable) {
        debugMode = enable;
    }

    bool begin() {
        SPI.begin();
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
        
        if (mcp2515.reset() != MCP2515::ERROR_OK) {
            if (debugMode) Serial.println("CAN: Reset failed");
            return false;
        }
        if (mcp2515.setBitrate(CAN_500KBPS) != MCP2515::ERROR_OK) {
            if (debugMode) Serial.println("CAN: Bitrate set failed");
            return false;
        }
        if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
            if (debugMode) Serial.println("CAN: Normal mode set failed");
            return false;
        }

        pinMode(intPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(intPin), handleInterrupt, FALLING);
        if (debugMode) Serial.println("CAN: Initialized successfully");
        return true;
    }

    void sendCommand(uint8_t commandId, uint8_t* data = nullptr, uint8_t length = 0) {
        struct can_frame frame;
        frame.can_id = (deviceId << 5) | commandId;
        frame.can_dlc = length;
        if (data && length > 0) {
            memcpy(frame.data, data, length);
        }
        
        if (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK) {
            if (debugMode) {
                Serial.print("CAN TX: ID=0x");
                Serial.print(frame.can_id, HEX);
                Serial.print(" Data:");
                for (int i = 0; i < frame.can_dlc; i++) {
                    Serial.print(" 0x");
                    Serial.print(frame.data[i], HEX);
                }
                Serial.println();
            }
        } else if (debugMode) {
            Serial.println("CAN: Send failed");
        }
    }

    bool readResponse(can_frame* frame) {
        if (messageReceived) {
            messageReceived = false;
            if (mcp2515.readMessage(frame) == MCP2515::ERROR_OK) {
                if (debugMode) {
                    Serial.print("CAN RX: ID=0x");
                    Serial.print(frame->can_id, HEX);
                    Serial.print(" Data:");
                    for (int i = 0; i < frame->can_dlc; i++) {
                        Serial.print(" 0x");
                        Serial.print(frame->data[i], HEX);
                    }
                    Serial.println();
                }
                return true;
            } else if (debugMode) {
                Serial.println("CAN: Read failed");
            }
        }
        return false;
    }

    void reset() {
        if (debugMode) Serial.println("CAN: Resetting...");
        begin();
    }
};

CanbusDriver* CanbusDriver::instance = nullptr;

#endif
