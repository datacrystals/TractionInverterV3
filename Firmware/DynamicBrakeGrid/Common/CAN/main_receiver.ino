#include "CanProtocol.h"

class MyCanReceiver : public CanProtocol {
public:
    MyCanReceiver(uint8_t csPin) : CanProtocol(csPin) {}

    void onEnableReceived(bool enabled) override {
        Serial.print("Enable received: ");
        Serial.println(enabled);
    }

    void onVoltageReceived(int voltage) override {
        Serial.print("Voltage received: ");
        Serial.println(voltage);
    }

    void onStringReceived(uint16_t id, const String& str) override {
        Serial.print("String received [");
        Serial.print(id);
        Serial.print("]: ");
        Serial.println(str);
    }
};

MyCanReceiver can(10);

void setup() {
    Serial.begin(9600);
    if (!can.begin()) {
        Serial.println("CAN init failed");
        while (1);
    }
    Serial.println("CAN receiver ready");
}

void loop() {
    can.update();
}
