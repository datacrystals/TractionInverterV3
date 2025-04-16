// System Configuration
#define SYSTEM_NAME "Chopper Controller v1.3"

// Hardware Defines
#define MAIN_PWM_PIN          9
#define FAN_PWM_PIN           6
#define FAN_TACH_PIN          3
#define STATUS_LED1_PIN       4
#define STATUS_LED2_PIN       5
#define VOLTAGE_SENSOR_PIN   A2
#define CURRENT_SENSOR_PIN   A0

// System Constants
#define VOLTAGE_MIN          1.0f
#define VOLTAGE_MAX          450.0f
#define OVERVOLTAGE_THRESHOLD 20.0f
#define UNDERVOLTAGE_THRESHOLD 5.0f
#define CURRENT_MAX          200.0f
#define CONTROL_INTERVAL_MS  2
#define LOG_INTERVAL_MS      1000
#define SAMPLE_WINDOW_SIZE   20

// Indicator Constants
#define SHORT_BLINK_MS       200
#define LONG_BLINK_MS        600
#define BLINK_GAP_MS         200
#define END_GAP_MS          1000

// Fault Manager Constants
#define MIN_RESET_TIME_SEC 5      // Minimum time a fault must be active before it can auto-reset
#define FAULT_NAME_LENGTH    20
#define BLINK_CODE_LENGTH     5
#define FAULT_BUFFER_SIZE     5
#define FAULT_DURATION_THRESHOLD 1000
#define AUTO_RESET_TIME_SEC  30
#define MAX_AUTO_RESET_COUNT  3

#define FAULT_BUFFER_OVERFLOW "FAULT_BUFFER_OVERFLOW"
#define AUTO_RESET_FAULT_TIME 60  // Time after which an auto-reset fault is forgotten
#define FAULT_NAME_LENGTH 32
#define BLINK_CODE_LENGTH 8

// Fan Control Constants
#define FAN_SAMPLE_SIZE      10
#define FAN_EXPONENT         1.5
#define BASE_FAN_SPEED       50
#define MIN_TEMP            25.0
#define MAX_TEMP            60.0
#define MIN_AIRFLOW_LFM     50
#define MAX_AIRFLOW_LFM    200
#define MAX_RPM            3000
#define MAX_AIRFLOW_CFM     100
#define DUCT_AREA_SQFT      0.25

#define DELTA_TEMP_RAMP_START 10 // Kelvin
#define DELTA_TEMP_RAMP_END   50 // Kelvin
#define NUM_DISCRETE_STEPS 8  // Define the number of discrete steps for fan speed

#define AIRFLOW_PER_FAN 200 // CFM
#define NUM_FANS 4

// Control Parameters
#define KP                  0.75f
#define SAFETY_KP           20.0f
#define BASE_MAX_DUTY_CHANGE 30
#define EMERGENCY_DUTY_CHANGE 1000

// Constants
#define SPECIFIC_HEAT_CAPACITY 1005.0 // J/(kg·K)
#define DENSITY 1.225 // kg/m^3
#define CFM_TO_CMS 0.00047194744 // Conversion factor from cfm to m³/s


#include "Common/VoltageSensor.h"
#include "Common/ACS758CurrentSensor.h"
#include "Common/Indicator.h"
#include "Common/FaultManager.h"
#include "Common/FanController.h"

// Forward declarations
class Indicator;
class FanController;
class FaultManager;

// Define the static member
FanController* FanController::instance = nullptr;

// System Components
Indicator FaultLED(STATUS_LED1_PIN);
Indicator StatusLED(STATUS_LED2_PIN);
FaultManager faultManager;
FanController fanController(FAN_PWM_PIN, FAN_EXPONENT, FAN_SAMPLE_SIZE);

class PwmController {
public:
    static void Initialize() {
        pinMode(MAIN_PWM_PIN, OUTPUT);
        SetFrequency(1000);
    }

    static void SetFrequency(uint16_t freqHz) {
        freqHz = constrain(freqHz, 1000, 4000); // Constrain frequency within valid range

        // Define possible prescalers and their corresponding register bits
        struct PrescalerOption {
            uint16_t prescaler;
            uint8_t csBits;
        };

        const PrescalerOption prescalers[] = {
            {1, _BV(CS10)},                     // Prescaler 1
            {8, _BV(CS11)},                     // Prescaler 8
            {64, _BV(CS10) | _BV(CS11)},        // Prescaler 64
            {256, _BV(CS12)},                   // Prescaler 256
            {1024, _BV(CS12) | _BV(CS10)}      // Prescaler 1024
        };

        uint16_t top = 0;
        uint8_t selectedCsBits = _BV(CS10); // Default to prescaler 1

        // Find the best prescaler and calculate the top value
        for (size_t i = 0; i < sizeof(prescalers)/sizeof(prescalers[0]); ++i) {
            const PrescalerOption& option = prescalers[i];
            uint32_t calculatedTop = (16000000UL / (option.prescaler * (uint32_t)freqHz)) - 1;

            if (calculatedTop <= 0xFFFF) {
                selectedCsBits = option.csBits;
                top = (uint16_t)calculatedTop;
                break;
            }
        }

        // Configure Timer1 for Fast PWM mode using ICR1 as TOP
        TCCR1A = _BV(COM1A1) | _BV(WGM11); // Non-inverting mode, WGM11 set
        TCCR1B = _BV(WGM13) | _BV(WGM12) | selectedCsBits; // WGM13/WGM12 set, prescaler applied
        ICR1 = top; // Set the top value for the frequency
        currentTop_ = top; // Store current top for duty cycle calculation
    }

    static void WriteDuty(uint8_t duty) {
        duty = constrain(duty, 0, 255);
        // Map 0-255 duty to 0-currentTop_ range
        OCR1A = (uint16_t)map(duty, 0, 255, 0, currentTop_);
    }

private:
    static uint16_t currentTop_; // Tracks the current top value for duty scaling
};

uint16_t PwmController::currentTop_ = 0;

#define NUM_AVG_SAMPLES 10  // Define the number of samples to average

class VoltageController {
public:
    VoltageController() : voltageSensor_(VOLTAGE_SENSOR_PIN, 6000.0f, 120.0f, VOLTAGE_MIN, VOLTAGE_MAX, 100),
                        currentSensor_(CURRENT_SENSOR_PIN, 20.0f, 0.6f),
                        powerQueue(nullptr), powerHead(0), powerTail(0), powerCount(0) {
        currentSensor_.setCurrentRange(0.0f, CURRENT_MAX);
        Reset();
        powerQueue = new float[NUM_AVG_SAMPLES];
    }

    ~VoltageController() {
        delete[] powerQueue;
    }

    void Update() {
        UpdateVoltageMeasurement();
        UpdateControlLoop();
        UpdateFrequencyStep();
        CheckFaults();
    }

    float GetVoltage() const { return measuredVoltage_; }
    float GetSetpoint() const { return setpoint_; }
    float GetDutyCycle() const { return dutyCycle_; }
    float GetCurrent() const { return currentSensor_.getCurrent(); }
    float GetPower() const {
        float power = GetVoltage() * GetCurrent();
        addPowerSample(power);  // Add the current power to the queue
        return power;
    }

    void SetSetpoint(float voltage) { setpoint_ = constrain(voltage, VOLTAGE_MIN, VOLTAGE_MAX); }

    // Get the rolling average of the last NUM_AVG_SAMPLES power values
    float getRollingPowerAvg() {
        if (powerCount == 0) return 0;

        float sum = 0;
        for (int i = 0; i < powerCount; i++) {
            sum += powerQueue[(powerTail + i) % NUM_AVG_SAMPLES];
        }
        return sum / powerCount;
    }

private:
    VoltageSensor voltageSensor_;
    ACS758CurrentSensor currentSensor_;
    float setpoint_ = 27.0f;
    float measuredVoltage_ = 0.0f;
    int dutyCycle_ = 0;
    float voltageSamples_[SAMPLE_WINDOW_SIZE] = {};
    uint8_t sampleIndex_ = 0;

    // Power queue for rolling average
    float* powerQueue;
    int powerHead;
    int powerTail;
    int powerCount;

    void Reset() {
        for (auto& sample : voltageSamples_) sample = setpoint_;
        powerHead = 0;
        powerTail = 0;
        powerCount = 0;
    }

    void UpdateVoltageMeasurement() {
        voltageSamples_[sampleIndex_] = voltageSensor_.readVoltage();
        sampleIndex_ = (sampleIndex_ + 1) % SAMPLE_WINDOW_SIZE;

        measuredVoltage_ = 0.0f;
        for (const auto& sample : voltageSamples_) {
            measuredVoltage_ += sample;
        }
        measuredVoltage_ /= SAMPLE_WINDOW_SIZE;
    }

    void UpdateControlLoop() {
        static uint32_t lastControlTime = 0;
        uint32_t currentTime = millis();

        if (currentTime - lastControlTime >= CONTROL_INTERVAL_MS) {
            float dt = (currentTime - lastControlTime) / 1000.0f;
            lastControlTime = currentTime;

            float error = measuredVoltage_ - setpoint_;
            float kp = KP;
            float maxChange = BASE_MAX_DUTY_CHANGE;

            if (error > OVERVOLTAGE_THRESHOLD) {
                kp = SAFETY_KP;
                maxChange = EMERGENCY_DUTY_CHANGE;
            } else {
                float errorRatio = fabs(error) / OVERVOLTAGE_THRESHOLD;
                maxChange += (EMERGENCY_DUTY_CHANGE - BASE_MAX_DUTY_CHANGE) * errorRatio;
            }

            float change = kp * error;
            change = constrain(change, -maxChange * dt, maxChange * dt);

            dutyCycle_ += static_cast<int>(change * 2.55f);
            dutyCycle_ = constrain(dutyCycle_, 0, 255);

            PwmController::WriteDuty(dutyCycle_);
        }
    }

    void UpdateFrequencyStep() {
        float dutyPercent = (dutyCycle_ / 255.0f) * 100.0f;
        
//
//        if (dutyPercent <= 25.0f) {
//            PwmController::SetFrequency(1000);
//        } else if (dutyPercent <= 50.0f) {
//            PwmController::SetFrequency(2000);
//        } else if (dutyPercent <= 75.0f) {
//            PwmController::SetFrequency(3000);
//        } else {
//            PwmController::SetFrequency(4000);
//        }
        //PwmController::SetFrequency(map(constrain(dutyPercent, 0, 100), 0, 100, 1000, 4000));
    
        // Generate a random frequency between 1000 Hz and 4000 Hz
//        int randomFrequency = random(1000, 4001);
    
//        PwmController::SetFrequency(randomFrequency);

        PwmController::SetFrequency(4186);
    }

    void CheckFaults() {
        float error = measuredVoltage_ - setpoint_;

        if (error > OVERVOLTAGE_THRESHOLD) {
            faultManager.assertFault("OVERVOLTAGE", "LSSL", true);
        }
        else if (error < -UNDERVOLTAGE_THRESHOLD) {
            faultManager.assertFault("UNDERVOLTAGE", "SLSL", true);
        }

        if (currentSensor_.getCurrent() > CURRENT_MAX) {
            faultManager.assertFault("OVERCURRENT", "LLSS", false);
        }
    }

    // Add a power sample to the queue
    void addPowerSample(float power) {
        powerQueue[powerHead] = power;
        powerHead = (powerHead + 1) % NUM_AVG_SAMPLES;
        if (powerCount < NUM_AVG_SAMPLES) {
            powerCount++;
        } else {
            powerTail = (powerTail + 1) % NUM_AVG_SAMPLES;
        }
    }
};


class StatusIndicators {
public:
    void Update() {
        StatusLED.Blink(1000);
        if (faultManager.getFaultCount() > 0) {
            char blinkCode[BLINK_CODE_LENGTH];
            faultManager.getMostRecentFaultBlinkCode(blinkCode, BLINK_CODE_LENGTH);
            FaultLED.BlinkCode(blinkCode);
        } else {
            FaultLED.Blink(1000);
        }
    }
};

// Global instances
VoltageController voltageController;
StatusIndicators statusIndicators;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    PwmController::Initialize();
    FaultLED.SetState(LOW);
    StatusLED.SetState(LOW);
    StatusLED.Blink(1000);
    voltageController.SetSetpoint(25.0f);
    fanController.SetStats(AIRFLOW_PER_FAN, NUM_FANS);

    Serial.println(SYSTEM_NAME " initialized");
}



double estimate_delta_t(double airflow_cfm, double power_watts) {
    double airflow_cms = airflow_cfm * CFM_TO_CMS;
    return (power_watts) / (airflow_cms * SPECIFIC_HEAT_CAPACITY * DENSITY);
}


// Function to map fan speed to discrete steps
int mapToDiscreteSteps(float fanSpeed, int numSteps) {
    int stepSize = 255 / (numSteps - 1);  // Calculate the size of each step
    int discreteSpeed = (int)(fanSpeed / stepSize) * stepSize;  // Map to the nearest step
    return discreteSpeed;
}

void loop() {
    static uint32_t lastFanUpdate = 0;
    static uint32_t lastLogTime = 0;

    voltageController.Update();
    statusIndicators.Update();
    faultManager.updateFaultStatus();
    faultManager.clearAutoResetFaults();

    // Update fan every 5 milliseconds
    if (millis() - lastFanUpdate >= 5) {
        lastFanUpdate = millis();

        // Update Sensors
        voltageController.GetPower();
        fanController.getCFM();

        // Calculate delta T
        double deltaT = estimate_delta_t(fanController.getRollingCFMAvg(), voltageController.getRollingPowerAvg());

        // Map delta T to fan speed
        float fanSpeed = map(constrain(deltaT, DELTA_TEMP_RAMP_START, DELTA_TEMP_RAMP_END),
                              DELTA_TEMP_RAMP_START, DELTA_TEMP_RAMP_END,
                              BASE_FAN_SPEED, 255.0f);

        // Map fan speed to discrete steps
        int discreteFanSpeed = mapToDiscreteSteps(fanSpeed, NUM_DISCRETE_STEPS);

        // Set the fan speed to the discrete value
        fanController.setSpeed(discreteFanSpeed);
    }

    // Log status every LOG_INTERVAL_MS milliseconds
    if (millis() - lastLogTime >= LOG_INTERVAL_MS) {
        lastLogTime = millis();
        Serial.print("Setpoint: ");
        Serial.print(voltageController.GetSetpoint(), 2);
        Serial.print("V | VIN: ");
        Serial.print(voltageController.GetVoltage(), 2);
        Serial.print("V | Duty: ");
        Serial.print((voltageController.GetDutyCycle() / 255.0f) * 100.0f, 1);
        Serial.print("% | Current: ");
        Serial.print(voltageController.GetCurrent(), 1);
        Serial.print("A | Power: ");
        Serial.print(voltageController.GetPower(), 1);
        Serial.print("W | Est. Delta T: ");
        Serial.print(estimate_delta_t(fanController.getRollingCFMAvg(), voltageController.getRollingPowerAvg()), 1);
        Serial.print("K | Fan: ");
        Serial.print(fanController.getRollingCFMAvg());
        Serial.println(" CFM");
    }
}
