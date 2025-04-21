#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <mcp_can.h>
#include <SPI.h>
#include "MessageCodes.h"

class CanProtocol {
public:
    CanProtocol(uint8_t csPin);
    bool begin();
    void update();

    // Generic Send
    bool sendMessage(uint8_t msgCode, const uint8_t* data, uint8_t len);

    // Helpers
    bool sendBool(MessageCode code, bool value);
    bool sendInt(MessageCode code, int value);
    bool sendString(uint16_t stringId, const String& str);

    // User-implemented callbacks
    virtual void onEnableReceived(bool enabled) {}
    virtual void onVoltageReceived(int voltage) {}
    virtual void onStringReceived(uint16_t stringId, const String& str) {}

private:
    MCP_CAN can;
    uint8_t csPin;

    struct StringBuffer {
        uint16_t id;
        String content;
        bool active;
    } stringBuf;

    void processMessage(uint8_t code, const uint8_t* data, uint8_t len);
};

#endif
