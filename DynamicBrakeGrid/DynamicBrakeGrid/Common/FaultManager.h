


class FaultManager {
public:
    FaultManager() {
        // Initialize the fault buffer
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
            faultBuffer[i].name[0] = '\0';
            faultBuffer[i].blinkCode[0] = '\0';
            faultBuffer[i].resetCount = 0;
            faultBuffer[i].startTime = 0;
            faultBuffer[i].isAutoReset = false;
            faultBuffer[i].isActive = false;
        }
    }

    // Assert a fault
    void assertFault(const char* faultName, const char* blinkCode, bool isAutoReset = true, bool overwriteFaults = false) {

        if (isOverflown) {
            return;
        }

        if (overwriteFaults) {

            // Now add our fault to the buffer
            strncpy(faultBuffer[0].name, faultName, FAULT_NAME_LENGTH);
            strncpy(faultBuffer[0].blinkCode, blinkCode, BLINK_CODE_LENGTH);
            faultBuffer[0].resetCount = 0;
            faultBuffer[0].startTime = millis();
            faultBuffer[0].isAutoReset = isAutoReset;
            faultBuffer[0].isActive = true;
            faultCount = 1;

            return;
        }

        // Check if the fault buffer is full
        // Serial.println(faultCount);

        if (faultCount >= FAULT_BUFFER_SIZE) {
            // Assert the FAULT_BUFFER_OVERFLOW fault if the buffer is full
            resetFaults();

            // Force-add the overflow fault, overwriting existing faults
            assertFault("FAULT_BUFFER_OVERFLOW", "SSSS", false, true);

            isOverflown = true;

        }

        // Check if the fault is already in the buffer
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
            if (strcmp(faultBuffer[i].name, faultName) == 0) {
                // Fault is already in the buffer, update the start time
                faultBuffer[i].startTime = millis();
                faultBuffer[i].isActive = true;
                return;
            }
        }

        // Add the fault to the buffer
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
            if (faultBuffer[i].name[0] == '\0') {
                strncpy(faultBuffer[i].name, faultName, FAULT_NAME_LENGTH);
                strncpy(faultBuffer[i].blinkCode, blinkCode, BLINK_CODE_LENGTH);
                faultBuffer[i].resetCount = 0;
                faultBuffer[i].startTime = millis();
                faultBuffer[i].isAutoReset = isAutoReset;
                faultBuffer[i].isActive = true;
                faultCount++;
                break;
            }
        }
    }

    void resetFaults() {

        isOverflown = false;

        // Clear existing faults
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
            faultBuffer[i].name[0] = '\0';
            faultBuffer[i].blinkCode[0] = '\0';
            faultBuffer[i].resetCount = 0;
            faultBuffer[i].startTime = 0;
            faultBuffer[i].isAutoReset = false;
            faultBuffer[i].isActive = false;
        }
        faultCount = 0;
    }

    // Clear auto-reset faults
    void clearAutoResetFaults() {
        unsigned long currentMillis = millis();
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
            if (faultBuffer[i].name[0] != '\0' && faultBuffer[i].isAutoReset) {
                unsigned long faultDuration = currentMillis - faultBuffer[i].startTime;
                if (faultDuration >= MIN_RESET_TIME_SEC * 1000) {
                    if (faultBuffer[i].resetCount < MAX_AUTO_RESET_COUNT) {
                        faultBuffer[i].resetCount++;
                        faultBuffer[i].startTime = currentMillis;
                    } else {
                        faultBuffer[i].name[0] = '\0';
                        faultBuffer[i].blinkCode[0] = '\0';
                        faultBuffer[i].resetCount = 0;
                        faultCount--;
                    }
                }
            }
        }
    }

    // Forget auto-reset faults after a certain time
    void forgetAutoResetFaults() {
        unsigned long currentMillis = millis();
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
            if (faultBuffer[i].name[0] != '\0' && faultBuffer[i].isAutoReset) {
                unsigned long faultDuration = currentMillis - faultBuffer[i].startTime;
                if (faultDuration >= AUTO_RESET_FAULT_TIME * 1000) {
                    faultBuffer[i].name[0] = '\0';
                    faultBuffer[i].blinkCode[0] = '\0';
                    faultBuffer[i].resetCount = 0;
                    faultCount--;
                }
            }
        }
    }

    // Get the current fault names
    void getFaultNames(char* faultNames, int maxLength) {
        faultNames[0] = '\0';
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
            if (faultBuffer[i].name[0] != '\0' && faultBuffer[i].isActive) {
                if (faultNames[0] != '\0') {
                    strncat(faultNames, ", ", maxLength - strlen(faultNames) - 1);
                }
                strncat(faultNames, faultBuffer[i].name, maxLength - strlen(faultNames) - 1);
            }
        }
    }

    // Get the blink code of the most recent fault
    void getMostRecentFaultBlinkCode(char* blinkCode, int maxLength) {
        blinkCode[0] = '\0';
        for (int i = FAULT_BUFFER_SIZE - 1; i >= 0; i--) {
            if (faultBuffer[i].name[0] != '\0' && faultBuffer[i].isActive) {
                strncpy(blinkCode, faultBuffer[i].blinkCode, maxLength);
                break;
            }
        }
    }

    // Get the current fault count
    int getFaultCount() {
        return faultCount;
    }

    // Update the fault status based on duration
    void updateFaultStatus() {
        unsigned long currentMillis = millis();
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
            if (faultBuffer[i].name[0] != '\0') {
                unsigned long faultDuration = currentMillis - faultBuffer[i].startTime;
                if (faultDuration >= FAULT_DURATION_THRESHOLD) {
                    faultBuffer[i].isActive = true;
                } else {
                    faultBuffer[i].isActive = false;
                }
            }
        }
    }

private:
    struct Fault {
        char name[FAULT_NAME_LENGTH];
        char blinkCode[BLINK_CODE_LENGTH];
        int resetCount;
        unsigned long startTime;
        bool isAutoReset;
        bool isActive;
    };

    Fault faultBuffer[FAULT_BUFFER_SIZE];
    int faultCount = 0;
    bool isOverflown;

};
