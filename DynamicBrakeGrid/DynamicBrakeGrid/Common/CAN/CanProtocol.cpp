#include "CanProtocol.h"

CanProtocol::CanProtocol(uint8_t csPin) : can(csPin), csPin(csPin) {
    stringBuf.active = false;
}

bool CanProtocol::begin() {
    if (CAN_OK != can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) return false;
    can.setMode(MCP_NORMAL);
    return true;
}

void CanProtocol::update() {
    if (can.checkReceive() == CAN_MSGAVAIL) {
        byte len = 0;
        byte buf[8];
        can.readMsgBuf(&len, buf);
        processMessage(buf[0], buf + 1, len - 1);
    }
}

bool CanProtocol::sendMessage(uint8_t msgCode, const uint8_t* data, uint8_t len) {
    uint8_t buf[8];
    buf[0] = msgCode;
    memcpy(buf + 1, data, len);
    return can.sendMsgBuf(0x100, 0, len + 1, buf) == CAN_OK;
}

bool CanProtocol::sendBool(MessageCode code, bool value) {
    uint8_t data = value ? 1 : 0;
    return sendMessage(code, &data, 1);
}

bool CanProtocol::sendInt(MessageCode code, int value) {
    uint8_t data[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return sendMessage(code, data, 2);
}

bool CanProtocol::sendString(uint16_t stringId, const String& str) {
    uint8_t idBytes[2] = { (uint8_t)(stringId >> 8), (uint8_t)(stringId & 0xFF) };
    sendMessage(STRING_START, idBytes, 2);

    uint8_t partId = 0;
    for (size_t i = 0; i < str.length(); i += 6) {
        uint8_t packet[7];
        packet[0] = partId++;
        memcpy(packet + 1, str.c_str() + i, min(6, str.length() - i));
        sendMessage(STRING_PART, packet, min(7, (int)(str.length() - i + 1)));
    }

    sendMessage(STRING_END, idBytes, 2);
    return true;
}

void CanProtocol::processMessage(uint8_t code, const uint8_t* data, uint8_t len) {
    switch (code) {
        case ENABLE:
            if (len >= 1) onEnableReceived(data[0] != 0);
            break;

        case VOLTAGE:
            if (len >= 2) {
                int value = (data[0] << 8) | data[1];
                onVoltageReceived(value);
            }
            break;

        case STRING_START:
            if (len >= 2) {
                stringBuf.id = (data[0] << 8) | data[1];
                stringBuf.content = "";
                stringBuf.active = true;
            }
            break;

        case STRING_PART:
            if (len >= 2 && stringBuf.active) {
                for (uint8_t i = 1; i < len; i++)
                    stringBuf.content += (char)data[i];
            }
            break;

        case STRING_END:
            if (len >= 2 && stringBuf.active) {
                uint16_t id = (data[0] << 8) | data[1];
                if (id == stringBuf.id) {
                    onStringReceived(stringBuf.id, stringBuf.content);
                    stringBuf.active = false;
                }
            }
            break;
    }
}
