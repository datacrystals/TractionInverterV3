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

// Control Parameters
#define KP                  0.75f
#define SAFETY_KP           20.0f
#define BASE_MAX_DUTY_CHANGE 30
#define EMERGENCY_DUTY_CHANGE 1000


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
        freqHz = constrain(freqHz, 1000, 4000);
        
        uint16_t prescaler;
        uint16_t top;
        
        if (freqHz >= 3000) {
            prescaler = 1;  // 4kHz
            top = 3999;
        } else if (freqHz >= 2000) {
            prescaler = 8;  // 2kHz
            top = 999;
        } else {
            prescaler = 64; // 1kHz
            top = 249;
        }

        TCCR1A = _BV(COM1A1) | _BV(WGM11);
        TCCR1B = _BV(WGM13) | _BV(WGM12);

        switch(prescaler) {
            case 1: TCCR1B |= _BV(CS10); break;
            case 8: TCCR1B |= _BV(CS11); break;
            case 64: TCCR1B |= _BV(CS10) | _BV(CS11); break;
        }

        ICR1 = top;
        currentTop_ = top;
    }

    static void WriteDuty(uint8_t duty) {
        duty = constrain(duty, 0, 255);
        uint16_t scaled = map(duty, 0, 255, 0, currentTop_);
        OCR1A = scaled;
    }

private:
    static uint16_t currentTop_;
};
uint16_t PwmController::currentTop_ = 0;

class VoltageController {
public:
    VoltageController() : voltageSensor_(VOLTAGE_SENSOR_PIN, 6000.0f, 120.0f, VOLTAGE_MIN, VOLTAGE_MAX, 100),
                        currentSensor_(CURRENT_SENSOR_PIN, 20.0f, 0.6f) {
        currentSensor_.setCurrentRange(0.0f, CURRENT_MAX);
        Reset();
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
    float GetPower() const { return GetVoltage() * GetCurrent(); }
    void SetSetpoint(float voltage) { setpoint_ = constrain(voltage, VOLTAGE_MIN, VOLTAGE_MAX); }

private:
    VoltageSensor voltageSensor_;
    ACS758CurrentSensor currentSensor_;
    float setpoint_ = 25.0f;
    float measuredVoltage_ = 0.0f;
    int dutyCycle_ = 0;
    float voltageSamples_[SAMPLE_WINDOW_SIZE] = {};
    uint8_t sampleIndex_ = 0;

    void Reset() {
        for (auto& sample : voltageSamples_) sample = setpoint_;
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
        
        if (dutyPercent <= 25.0f) {
            PwmController::SetFrequency(1000);
        } else if (dutyPercent <= 50.0f) {
            PwmController::SetFrequency(2000);
        } else if (dutyPercent <= 75.0f) {
            PwmController::SetFrequency(3000);
        } else {
            PwmController::SetFrequency(4000);
        }
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
};

class StatusIndicators {
public:
    void Update() {
        if (faultManager.getFaultCount() > 0) {
            char blinkCode[BLINK_CODE_LENGTH];
            faultManager.getMostRecentFaultBlinkCode(blinkCode, BLINK_CODE_LENGTH);
            FaultLED.BlinkCode(blinkCode);
            StatusLED.BlinkCode(blinkCode);
        } else {
            FaultLED.Blink(1000);
            StatusLED.Blink(1000);
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
    voltageController.SetSetpoint(25.0f);
    fanController.SetStats(220, 4);
    
    Serial.println(SYSTEM_NAME " initialized");
}

void loop() {
    static uint32_t lastFanUpdate = 0;
    static uint32_t lastLogTime = 0;
    
    voltageController.Update();
    statusIndicators.Update();
    faultManager.updateFaultStatus();
    faultManager.clearAutoResetFaults();
        
    // Update fan every second
    if (millis() - lastFanUpdate >= 1000) {
        lastFanUpdate = millis();
        fanController.setSpeed(10.0f); // Replace with actual temp reading
    }
    
    // Log status every 5 seconds
    if (millis() - lastLogTime >= 1000) {
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
        Serial.print("W | Fan: ");
        Serial.print(fanController.getCFM());
        Serial.println(" CFM");
    }
}
