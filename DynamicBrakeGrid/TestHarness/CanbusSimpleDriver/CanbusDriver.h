#ifndef CANBUS_DRIVER_H
#define CANBUS_DRIVER_H

#include "CANFramework/src/CANBusManager.h"

class CanbusDriver {
private:
    CANBusManager canManager;

public:
    CanbusDriver(uint8_t deviceId, uint8_t csPin, uint8_t intPin)
        : canManager(deviceId, csPin, intPin) {}

    void begin() {
        canManager.Begin();
    }

    void setCallback(CANBusManager::MessageCallback callback) {
        canManager.SetCallback(callback);
    }

    void sendCommand(MessageCode code, uint8_t* data = nullptr, uint8_t length = 0) {
        CANMessage msg;
        msg.code = code;
        msg.length = length;
        if (data && length > 0) {
            memcpy(msg.data, data, length);
        }
        canManager.SendMessage(msg);
    }

    void update() {
        canManager.Update();
    }
};

#endif
