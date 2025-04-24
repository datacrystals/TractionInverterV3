#define NUM_AVG_SAMPLES 20  // Define the number of samples to average

class FanController {
public:
    // Constructor for both tach and non-tach fans
    FanController(int pwmPin, float exponent, int sampleSize, int rpmPin = -1)
    : rpmPin(rpmPin), pwmPin(pwmPin), rpm(0), lastTime(0), pulseCount(0),
    lastAirflowUpdate(0), cachedAirflowLFM(0), exponent(exponent),
    sampleSize(sampleSize), temperatureQueue(nullptr), head(0), tail(0), count(0),
    cfmQueue(nullptr), cfmHead(0), cfmTail(0), cfmCount(0) {
        pinMode(pwmPin, OUTPUT);
        temperatureQueue = new float[sampleSize];
        cfmQueue = new float[NUM_AVG_SAMPLES];

        if (rpmPin != -1) {
            pinMode(rpmPin, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(rpmPin), rpmInterruptStatic, FALLING);
            instance = this;
        }
    }

    // Destructor
    ~FanController() {
        delete[] temperatureQueue;
        delete[] cfmQueue;
        if (rpmPin != -1) {
            detachInterrupt(digitalPinToInterrupt(rpmPin));
        }
    }

    void SetStats(int MaxCFM, int NumFans) {
        numFans = NumFans;
        maxCFM = MaxCFM;
    }

    float getCFM() {
        float cfm = (float)numFans * (float)maxCFM * (float)currentSpeed / 255.0;
        addCFMSample(cfm);  // Add the current CFM to the queue
        return cfm;
    }

    // Set the fan speed (0-255)
    void setSpeed(int speed) {
        if (speed < 0) speed = 0;
        if (speed > 255) speed = 255;
        currentSpeed = speed;

        speed = 255 - speed; // convert speed for inverted fan FanControl

        analogWrite(pwmPin, speed);
    }

    // Get the current RPM (returns -1 if no tachometer)
    int getRPM() {
        if (rpmPin == -1) return -1;

        noInterrupts();
        unsigned long now = millis();
        rpm = (pulseCount * 60000) / (now - lastTime);
        lastTime = now;
        pulseCount = 0;
        interrupts();
        return rpm;
    }

    // Get airflow (returns -1 if no tachometer)
    int getAirflowLFM() {
        if (rpmPin == -1) return -1;
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

    // Get the rolling average of the last NUM_AVG_SAMPLES CFM values
    float getRollingCFMAvg() {
        if (cfmCount == 0) return 0;

        float sum = 0;
        for (int i = 0; i < cfmCount; i++) {
            sum += cfmQueue[(cfmTail + i) % NUM_AVG_SAMPLES];
        }
        return sum / cfmCount;
    }

    // Public static instance pointer
    static FanController* instance;

private:
    int rpmPin;
    int pwmPin;
    int numFans;
    int maxCFM;
    int currentSpeed;
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

    // CFM queue for rolling average
    float* cfmQueue;
    int cfmHead;
    int cfmTail;
    int cfmCount;

    // Update the cached airflow value every half second (only if tach available)
    void updateAirflowLFM() {
        if (rpmPin == -1) return;

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

    // Add a CFM sample to the queue
    void addCFMSample(float cfm) {
        cfmQueue[cfmHead] = cfm;
        cfmHead = (cfmHead + 1) % NUM_AVG_SAMPLES;
        if (cfmCount < NUM_AVG_SAMPLES) {
            cfmCount++;
        } else {
            cfmTail = (cfmTail + 1) % NUM_AVG_SAMPLES;
        }
    }
};
