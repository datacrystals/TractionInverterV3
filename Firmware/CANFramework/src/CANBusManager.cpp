#include "CANBusManager.h"

#define CAN_API_VERSION 1

CANBusManager::CANBusManager(uint32_t canID, uint8_t csPin, uint8_t intPin)
    : mcp(csPin), canID(canID), csPin(csPin), messageCallback(nullptr), stringCallback(nullptr), stringIndex(0), expectedLength(0), hasError(false) {}

void CANBusManager::Begin() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    SPI.begin();
    mcp.reset();
    mcp.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp.setNormalMode();
}

void CANBusManager::SetCallback(MessageCallback callback) {
    messageCallback = callback;
}

void CANBusManager::SetStringCallback(StringReceivedCallback callback) {
    stringCallback = callback;
}

void CANBusManager::Update() {
    if (mcp.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        ProcessMessage(canMsg);
    }
}

void CANBusManager::ProcessMessage(const struct can_frame& frame) {
    MessageCode code = (MessageCode)frame.data[0];

    if (code == STRING_START) {
        MessageCode userMessageCode = (MessageCode)frame.data[1]; // Extract code from data[1]
        expectedLength = frame.data[2]; // Extract length from data[2]
        stringIndex = 0;
        memset(stringBuffer, 0, MAX_STRING_BUFFER);
    }
    else if (code == STRING_PART) {
        for (int i = 2; i < frame.can_dlc && stringIndex < MAX_STRING_BUFFER - 1; ++i) {
            stringBuffer[stringIndex++] = frame.data[i];
        }
    }
    else if (code == STRING_END) {
        if (stringCallback) {
            stringBuffer[stringIndex] = '\0';
            MessageCode userMessageCode = (MessageCode)frame.data[1]; // Extract code from data[1]
            stringCallback(userMessageCode, stringBuffer);
        }
    }
    else if (code == API_VERSION_REQUEST) {
        CANMessage msg = { (uint16_t)canID, 2, { (uint8_t)API_VERSION_RESPONSE, (uint8_t)CAN_API_VERSION } }; // Ensure id is uint16_t
        SendMessage(msg);
    }
    else {
        if (messageCallback) {
            CANMessage msg = { (uint16_t)frame.can_id, frame.can_dlc }; // Ensure id is uint16_t
            memcpy(msg.data, frame.data, frame.can_dlc);
            messageCallback(msg);
        }
    }
}

void CANBusManager::SendMessage(const CANMessage& msg) {
    struct can_frame frame;
    frame.can_id = msg.id; // Use msg.id directly
    frame.can_dlc = msg.length;
    memcpy(frame.data, msg.data, msg.length);
    if (mcp.sendMessage(&frame) != MCP2515::ERROR_OK) {
        hasError = true;
    #ifdef ENABLE_SERIAL_PRINT
        Serial.println(F("Failed to Send CAN Message"));
    #endif
    }
}

void CANBusManager::SendCode(MessageCode code) {
    CANMessage msg = { (uint16_t)canID, 1, { (uint8_t)code } }; // Ensure id is uint16_t
    SendMessage(msg);
}

void CANBusManager::SendBool(MessageCode code, bool value) {
    CANMessage msg = { (uint16_t)canID, 2, { (uint8_t)code, (uint8_t)value } }; // Ensure id is uint16_t
    SendMessage(msg);
}

void CANBusManager::SendInt(MessageCode code, int32_t value) {
    CANMessage msg = { (uint16_t)canID, 5 }; // Ensure id is uint16_t
    msg.data[0] = (uint8_t)code;
    memcpy(&msg.data[1], &value, 4);
    SendMessage(msg);
}

void CANBusManager::SendFloat(MessageCode code, float value) {
    CANMessage msg = { (uint16_t)canID, 5 }; // Ensure id is uint16_t
    msg.data[0] = (uint8_t)code;
    memcpy(&msg.data[1], &value, 4);
    SendMessage(msg);
}

bool CANBusManager::SendString(MessageCode code, const String& value) {
    if (value.length() >= MAX_STRING_BUFFER) return false;

    uint16_t length = value.length();
    SendStringStart(code, length); // Use the simplified STRING_START function
    uint8_t partID = 0;
    for (uint16_t i = 0; i < length; i += 6) {
        CANMessage partMsg = { (uint16_t)canID, 8 }; // Ensure id is uint16_t
        partMsg.data[0] = (uint8_t)STRING_PART;
        partMsg.data[1] = partID++;
        for (uint8_t j = 0; j < 6 && (i + j) < length; ++j) {
            partMsg.data[2 + j] = value[i + j];
        }
        SendMessage(partMsg);
    }
    SendInt(STRING_END, code);
    return true;
}

void CANBusManager::SendStringStart(MessageCode code, uint16_t length) {
    CANMessage msg = { (uint16_t)canID, 3 }; // Ensure id is uint16_t
    msg.data[0] = (uint8_t)STRING_START;
    msg.data[1] = (uint8_t)code;
    msg.data[2] = (uint8_t)length; // Send length as a single byte (if length fits in 8 bits)
    SendMessage(msg);
}

MessageCode CANBusManager::GetMessageCode(const CANMessage& msg) {
    return (MessageCode)msg.data[0];
}

bool CANBusManager::ReadBool(const CANMessage& msg) {
    return msg.data[1] != 0;
}

int32_t CANBusManager::ReadInt(const CANMessage& msg) {
    int32_t value;
    memcpy(&value, &msg.data[1], 4);
    return value;
}

float CANBusManager::ReadFloat(const CANMessage& msg) {
    float value;
    memcpy(&value, &msg.data[1], 4);
    return value;
}