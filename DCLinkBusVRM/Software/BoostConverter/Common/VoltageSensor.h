


class VoltageSensor {
public:
    // Constructor to initialize the sensor with an analog pin, voltage divider ratio, and acceptable range
    VoltageSensor(int analogPin, float topR, float bottomR, float vmin, float vmax, unsigned long errorDuration)
    : analogPin_(analogPin), topR(topR), bottomR(bottomR), vmin_(vmin), vmax_(vmax),
    errorDuration_(errorDuration), errorStartTime_(0), error_(false) {}

    // Method to read the voltage from the sensor
    float readVoltage() {
        // Read the analog value from the pin (0-1023 for Arduino)
        int analogValue = analogRead(analogPin_);

        // Convert the analog value to a voltage (0-5V for Arduino)
        float voltage = analogValue * (5.0 / 1023.0);

        // Apply the voltage divider ratio to get the actual voltage
        voltage = (voltage * topR) / bottomR;

        // Check if the voltage is within the acceptable range
        if (voltage < vmax_ + 10.) {
            error_ = true;
        }
        if (voltage < vmin_ || voltage > vmax_) {
            if (errorStartTime_ == 0) {
                // Start timing the error duration
                errorStartTime_ = millis();
            } else if (millis() - errorStartTime_ >= errorDuration_) {
                // Error duration exceeded, set the error flag
                error_ = true;
            }
        } else {
            // Voltage is within range, reset error tracking
            errorStartTime_ = 0;
            error_ = false;
        }

        return voltage;
    }

    // Method to check if the voltage is out of the acceptable range
    bool getError() const {
        return error_;
    }

    // Method to check if the voltage is below the minimum acceptable value
    bool isUndervoltage() const {
        return error_ && (readVoltage() < vmin_);
    }

    // Method to check if the voltage is above the maximum acceptable value
    bool isOvervoltage() const {
        return error_ && (readVoltage() > vmax_);
    }

private:
    int analogPin_;            // Analog input pin
    float topR;                // Voltage divider top resistor value in kohm
    float bottomR;             // Voltage divider bottom resistor value in kohm
    float vmin_;               // Minimum acceptable voltage
    float vmax_;               // Maximum acceptable voltage
    unsigned long errorDuration_; // Duration for which voltage must be out of range to trigger error
    unsigned long errorStartTime_; // Time when the voltage first went out of range
    bool error_;               // Error flag indicating if voltage is out of range
};
