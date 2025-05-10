#ifndef CANBUSMANAGER_H
#define CANBUSMANAGER_H

#include <mcp2515.h>
#include <SPI.h>
#include "MessageCode.h"
#include "CANMessage.h"

#define MAX_STRING_BUFFER 128

class CANBusManager {
public:
    typedef void (*MessageCallback)(const CANMessage&);
    typedef void (*StringReceivedCallback)(MessageCode userCode, const char* message);

    CANBusManager(uint32_t canID, uint8_t csPin, uint8_t intPin);
    void Begin();
    void Update();
    void SetCallback(MessageCallback callback);
    void SetStringCallback(StringReceivedCallback callback);

    void SendMessage(const CANMessage& msg);
    void Send(MessageCode code);
    void SendCode(MessageCode code);
    void SendBool(MessageCode code, bool value);
    void SendInt(MessageCode code, int32_t value);
    void SendFloat(MessageCode code, float value);
    bool SendString(MessageCode code, const String& value); // will return false if string is too long

    static MessageCode GetMessageCode(const CANMessage& msg);
    static bool ReadBool(const CANMessage& msg);
    static int32_t ReadInt(const CANMessage& msg);
    static float ReadFloat(const CANMessage& msg);

    void PrintChipStatus();
    bool HasError() const {
        return hasError;
    }

private:
    void ProcessMessage(const struct can_frame& frame);
    void SendStringStart(MessageCode code, uint16_t length);
    MCP2515 mcp;
    int csPin;
    struct can_frame canMsg;
    uint32_t canID;
    MessageCallback messageCallback;
    StringReceivedCallback stringCallback;

    char stringBuffer[MAX_STRING_BUFFER];
    uint16_t stringIndex;
    MessageCode currentStringCode;
    uint16_t expectedLength;
    bool hasError;
};

#endif