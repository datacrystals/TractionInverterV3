#include "CanProtocol.h"

CanProtocol can(10);

void setup() {
    Serial.begin(9600);
    if (!can.begin()) {
        Serial.println("CAN init failed");
        while (1);
    }
    Serial.println("CAN started");

    can.sendBool(ENABLE, true);
    can.sendInt(VOLTAGE, 1234);
    can.sendString(42, "Hello from sender");
}

void loop() {
    can.update();
}
