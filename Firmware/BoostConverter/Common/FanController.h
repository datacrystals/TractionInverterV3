

class FanController {
public:
    // Constructor
    FanController(int rpmPin, int pwmPin, float exponent, int sampleSize)
    : rpmPin(rpmPin), pwmPin(pwmPin), rpm(0), lastTime(0), pulseCount(0),
    lastAirflowUpdate(0), cachedAirflowLFM(0), exponent(exponent),
    sampleSize(sampleSize), temperatureQueue(nullptr), head(0), tail(0), count(0) {
        pinMode(rpmPin, INPUT_PULLUP);
        pinMode(pwmPin, OUTPUT);
        attachInterrupt(digitalPinToInterrupt(rpmPin), rpmInterruptStatic, FALLING);
        instance = this;
        temperatureQueue = new float[sampleSize];
    }

    // Destructor
    ~FanController() {
        delete[] temperatureQueue;
    }

    // Set the fan speed (0-255)
    void setSpeed(int speed) {
        if (speed < 0) speed = 0;
        if (speed > 255) speed = 255;
        analogWrite(pwmPin, speed);
    }

    // Adjust fan speed based on temperature
    void setFanSpeed(float currentTemp) {
        // Add the current temperature to the queue
        addTemperatureSample(currentTemp);

        // Calculate the average temperature from the last n samples
        float avgTemp = calculateAverageTemperature();

        int speed = BASE_FAN_SPEED;
        if (avgTemp > MAX_TEMP) {
            speed = 255;
        } else if (avgTemp > MIN_TEMP) {
            // Exponential fan curve
            float tempRange = MAX_TEMP - MIN_TEMP;
            float normalizedTemp = (avgTemp - MIN_TEMP) / tempRange;
            speed = BASE_FAN_SPEED + (255 - BASE_FAN_SPEED) * pow(normalizedTemp, exponent);
        }
        setSpeed(speed);
    }

    // Get the current RPM
    int getRPM() {
        noInterrupts(); // Disable interrupts to read shared variables
        unsigned long now = millis();
        rpm = (pulseCount * 60000) / (now - lastTime);
        lastTime = now;
        pulseCount = 0;
        interrupts(); // Re-enable interrupts
        return rpm;
    }

    // Get the current airflow in LFM
    int getAirflowLFM() {
        updateAirflowLFM();
        return cachedAirflowLFM;
    }

    // Calculate and print airflow in LFM
    void printAirflowLFM() {
        updateAirflowLFM();
        Serial.print("Airflow: ");
        Serial.print(cachedAirflowLFM);
        Serial.println(" LFM");
    }

    // Check if the airflow is within the acceptable range
    bool checkError() {
        updateAirflowLFM();
        return (cachedAirflowLFM < MIN_AIRFLOW_LFM) || (cachedAirflowLFM > MAX_AIRFLOW_LFM);
    }

    // Public static instance pointer
    static FanController* instance;

private:
    int rpmPin;
    int pwmPin;
    volatile int rpm;
    volatile unsigned long lastTime;
    volatile int pulseCount;
    unsigned long lastAirflowUpdate;
    float cachedAirflowLFM;
    float exponent;
    int sampleSize;
    float* temperatureQueue;
    int head;
    int tail;
    int count;

    // Update the cached airflow value every half second
    void updateAirflowLFM() {
        unsigned long currentMillis = millis();
        if (currentMillis - lastAirflowUpdate >= 500) {
            lastAirflowUpdate = currentMillis;
            int currentRPM = getRPM();
            float airflowCFM = (currentRPM / (float)MAX_RPM) * MAX_AIRFLOW_CFM;
            cachedAirflowLFM = airflowCFM / DUCT_AREA_SQFT;
        }
    }

    // Static ISR wrapper
    static void rpmInterruptStatic() {
        // Call the actual ISR on the instance
        instance->rpmInterrupt();
    }

    // Interrupt service routine for RPM measurement
    void rpmInterrupt() {
        pulseCount++;
    }

    // Add a temperature sample to the queue
    void addTemperatureSample(float temp) {
        temperatureQueue[head] = temp;
        head = (head + 1) % sampleSize;
        if (count < sampleSize) {
            count++;
        } else {
            tail = (tail + 1) % sampleSize;
        }
    }

    // Calculate the average temperature from the queue
    float calculateAverageTemperature() {
        float sum = 0;
        for (int i = 0; i < count; i++) {
            sum += temperatureQueue[(tail + i) % sampleSize];
        }
        return sum / count;
    }
};
