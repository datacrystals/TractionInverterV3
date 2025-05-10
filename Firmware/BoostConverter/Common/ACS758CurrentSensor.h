class ACS758CurrentSensor {
public:
    // Constructor
    ACS758CurrentSensor(int voutPin, float sensitivity_mVA = 20.0, float quiescentVoltage = 0.6)
    : voutPin(voutPin), sensitivity_mVA(sensitivity_mVA), quiescentVoltage(quiescentVoltage) {}

    // Set the acceptable current range
    void setCurrentRange(float minCurrent, float maxCurrent) {
        this->minCurrent = minCurrent;
        this->maxCurrent = maxCurrent;
    }

    // Read the current from the sensor
    float getCurrent(int n = 5) {
        float totalCurrent = 0.0;
        for (int i = 0; i < n; i++) {
            int sensorValue = analogRead(voutPin);
            float voltage = sensorValue * (5.0 / 1023.0); // Convert to voltage
            float current = ((voltage - quiescentVoltage) * 1000.0) / sensitivity_mVA;
            totalCurrent += abs(current); // Take the absolute value of the current
        }
        float averageCurrent = totalCurrent / n;
        return averageCurrent;
    }



    // Check if the current is within the acceptable range
    bool getError() {
        float current = getCurrent();
        return (current < minCurrent || current > maxCurrent);
    }

private:
    int voutPin;
    float sensitivity_mVA;
    float quiescentVoltage;
    float minCurrent = 0.0;
    float maxCurrent = 200.0;
};
