// System Configuration
#define SYSTEM_NAME "Chopper Controller v1.3"
#define FW_VERSION "1.3"
#define DEVICE_ID 0x01

// Hardware Defines
#define MAIN_PWM_PIN          9
#define FAN_PWM_PIN           6
#define FAN_TACH_PIN          3
#define STATUS_LED1_PIN       4
#define STATUS_LED2_PIN       5
#define VOLTAGE_SENSOR_PIN   A2
#define CURRENT_SENSOR_PIN   A0
#define CAN_CS_PIN           10  // CAN Bus Chip Select
#define CAN_INT_PIN          2   // CAN Bus Interrupt

// System Constants
#define VOLTAGE_MIN          1.0f
#define VOLTAGE_MAX          450.0f
#define OVERVOLTAGE_THRESHOLD 20.0f
#define UNDERVOLTAGE_THRESHOLD 5.0f
#define CURRENT_MAX          200.0f
#define CONTROL_INTERVAL_MS  2
#define LOG_INTERVAL_MS      1000
#define SAMPLE_WINDOW_SIZE   20

#define DEFAULT_V_SETPOINT 300.0f


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
#define MAX_TEMP            45.0
#define MIN_AIRFLOW_LFM     50
#define MAX_AIRFLOW_LFM    200
#define MAX_RPM            3000
#define MAX_AIRFLOW_CFM     100
#define DUCT_AREA_SQFT      0.25

#define DELTA_TEMP_RAMP_START 7 // Kelvin
#define DELTA_TEMP_RAMP_END   30 // Kelvin
#define NUM_DISCRETE_STEPS 14  // Define the number of discrete steps for fan speed

#define AIRFLOW_PER_FAN 220 // CFM
#define NUM_FANS 4

// Control Parameters
#define KP                  0.75f
#define SAFETY_KP           20.0f
#define BASE_MAX_DUTY_CHANGE 30
#define EMERGENCY_DUTY_CHANGE 1000

#define CURRENT_SOFT_LIMIT      35.0f    // Soft current threshold
#define CURRENT_HARD_LIMIT      40.0f    // Hard current threshold (replaces CURRENT_MAX)
#define POWER_SOFT_LIMIT        3500.0f  // Soft power threshold (adjust as needed)
#define POWER_HARD_LIMIT        4000.0f  // Hard power threshold

// Constants
#define SPECIFIC_HEAT_CAPACITY 1005.0 // J/(kg·K)
#define DENSITY 1.225 // kg/m^3
#define CFM_TO_CMS 0.00047194744 // Conversion factor from cfm to m³/s
#define CURRENT_ROLLING_AVERAGE_WINDOW 10


#include "Common/VoltageSensor.h"
#include "Common/ACS758CurrentSensor.h"
#include "Common/Indicator.h"
#include "Common/FaultManager.h"
#include "Common/FanController.h"

#include <mcp2515.h> // https://github.com/autowp/arduino-MCP2515/archive/master.zip


// CAN Bus Constants
#define CAN_BAUDRATE         CAN_500KBPS
#define CAN_CLOCK            MCP_16MHZ
#define CAN_RETRY_DELAY_MS   100

/*
 * CAN Bus Message Protocol
 * 
 * Message ID Format: 0x1XX where XX is the message type
 * 
 * Outgoing Messages (Controller → Host):
 * 0x100 - Status Message (8 bytes)
 *   Byte 0-3: Input Voltage (float, volts)
 *   Byte 4-5: Current (uint16_t, 0.01A resolution)
 *   Byte 6:   Duty Cycle (0-255)
 *   Byte 7:   Fault Count (0-255)
 * 
 * 0x101 - Fault Message (8 bytes)
 *   Byte 0-7: Fault Name (ASCII, null-padded)
 * 
 * 0x102 - System Info (8 bytes)
 *   Byte 0-3: Firmware Version (float)
 *   Byte 4-7: Uptime (uint32_t, seconds)
 * 
 * Incoming Messages (Host → Controller):
 * 0x200 - Set Voltage Setpoint (4 bytes)
 *   Byte 0-3: Setpoint Voltage (float, volts)
 * 
 * 0x201 - Reset Faults (1 byte)
 *   Byte 0:   Reset Code (0x55 = reset all faults)
 * 
 * 0x202 - Request Status (1 byte)
 *   Byte 0:   Request Type (0x01 = immediate status)
 * 
 * 0x203 - System Command (1 byte)
 *   Byte 0:   Command (0x01 = reboot, 0x02 = factory reset)
 */

// CAN Message IDs
#define CAN_ID_STATUS         0x100
#define CAN_ID_FAULT          0x101
#define CAN_ID_SYSTEM_INFO    0x102
#define CAN_ID_SET_SETPOINT   0x200
#define CAN_ID_RESET_FAULTS   0x201
#define CAN_ID_REQUEST_STATUS 0x202
#define CAN_ID_SYSTEM_CMD     0x203
#define CAN_ID_FAULT_LIST     0x103
// Command IDs
#define CMD_EMERGENCY_STOP   0x00
#define CMD_DISABLE_OUTPUT   0x01
#define CMD_ENABLE_OUTPUT    0x02
#define CMD_RESET_FAULTS     0x03
#define CMD_GET_FAULT_COUNT  0x04
#define CMD_GET_FAULT_LIST   0x05
#define CMD_GET_VOLTAGE_IN   0x06
#define CMD_GET_VOLTAGE_OUT  0x07
#define CMD_GET_CURRENT      0x08
#define CMD_GET_TEMPERATURE  0x09
#define CMD_GET_FAN_SPEED    0x0A
#define CMD_GET_POWER        0x0B
#define CMD_SET_VOLTAGE      0x0C
#define CMD_HEARTBEAT        0x0D


// Forward declarations
class Indicator;
class FanController;
class FaultManager;
class CANController;

void sendAck();
void sendFaultCount();
void sendFaultList();
void sendVoltageIn();
void sendVoltageOut(); 
void sendCurrent();
void sendTemperature();
void sendFanSpeed();
void sendPower();

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

    void SetDutyCycle(int duty) {
        dutyCycle_ = duty;
        PwmController::WriteDuty(duty);
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
    float setpoint_ = 380.0f;
    float measuredVoltage_ = 0.0f;
    int dutyCycle_ = 0;
    float voltageSamples_[SAMPLE_WINDOW_SIZE] = {};
    uint8_t sampleIndex_ = 0;
    float current_rolling_avg[CURRENT_ROLLING_AVERAGE_WINDOW] = {0.0};
    int current_rolling_avg_i = 0;
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

    
            // Soft limit back-off
            float current = currentSensor_.getCurrent();
            float power = measuredVoltage_ * current;
    
            if (current > CURRENT_SOFT_LIMIT || power > POWER_SOFT_LIMIT) {
                // Reduce duty cycle by 5% of current value each cycle
                dutyCycle_ = static_cast<int>(dutyCycle_ * 0.95f);
                dutyCycle_ = constrain(dutyCycle_, 0, 255);
            }
    
            PwmController::WriteDuty(dutyCycle_);
            lastControlTime = currentTime;
        }
    }

    void UpdateFrequencyStep() {
        // float dutyPercent = (dutyCycle_ / 255.0f) * 100.0f;
      //   float Current_A = GetCurrent();
      
      // (this->current_rolling_avg)[this->current_rolling_avg_i] = Current_A;
      // this->current_rolling_avg_i = (this->current_rolling_avg_i + 1) % CURRENT_ROLLING_AVERAGE_WINDOW;
      //  if (Current_A <= 5.0f) {
      //      PwmController::SetFrequency(4000);
      //  } else if (Current_A <= 12.0f) {
      //      PwmController::SetFrequency(2000);
      //  } else {
      //      PwmController::SetFrequency(1000);
      //  }
        // PwmController::SetFrequency(map(constrain(dutyPercent, 0, 100), 0, 100, 1250, 1024));
    
        // Generate a random frequency between 1000 Hz and 4000 Hz
//        int randomFrequency = random(1000, 4001);
    
//        PwmController::SetFrequency(randomFrequency);

        PwmController::SetFrequency(1024);
    }

    float get_avg_current() {
      float sum = 0.0;
      for (int i = 0; i < CURRENT_ROLLING_AVERAGE_WINDOW; i++) {
        sum += this->current_rolling_avg[i];
      }

      return sum / ((float) CURRENT_ROLLING_AVERAGE_WINDOW);
    }

    void CheckFaults() {
        float current = currentSensor_.getCurrent();
        float power = measuredVoltage_ * current; // Instantaneous power
    
        // Hard limits trigger panic
        if (current > CURRENT_HARD_LIMIT || power > POWER_HARD_LIMIT) {
            faultManager.assertFault(current > CURRENT_HARD_LIMIT ? "OVERCURRENT" : "OVERPOWER", 
                                    "LLLS", false); // No auto-reset
            setpoint_ = VOLTAGE_MAX;
            SetDutyCycle(0); // Force duty cycle to 0
        }
    
        // Existing checks
        float error = measuredVoltage_ - setpoint_;
        if (error > OVERVOLTAGE_THRESHOLD) {
            faultManager.assertFault("OVERVOLTAGE", "LSSL", true);
        } else if (error < -UNDERVOLTAGE_THRESHOLD) {
            faultManager.assertFault("UNDERVOLTAGE", "SLSL", true);
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


class CANController {
private:
    int csPin;
    int interruptPin;
    MCP2515 MCP2515;
    void (*callback)(struct can_frame*);
    bool hasError;
    bool debugEnabled;

    static void handleInterruptStatic() {
        if (instance) {
            instance->handleInterrupt();
        }
    }

    void handleInterrupt() {
        struct can_frame frame;
        uint8_t result = MCP2515.readMessage(&frame);
        
        if (result == MCP2515::ERROR_OK) {
            if (debugEnabled) {
                Serial.print("CAN RX: ID=0x");
                Serial.print(frame.can_id, HEX);
                Serial.print(" DLC=");
                Serial.print(frame.can_dlc);
                Serial.print(" Data:");
                for (int i = 0; i < frame.can_dlc; i++) {
                    Serial.print(" 0x");
                    Serial.print(frame.data[i], HEX);
                }
                Serial.println();
            }
            
            if (callback) {
                callback(&frame);
            }
        } else {
            hasError = true;
            if (debugEnabled) {
                Serial.print("CAN RX Error: ");
                printError(result);
            }
        }
    }

    void printError(uint8_t error) {
        switch(error) {
            case MCP2515::ERROR_FAIL:    Serial.println("General failure"); break;
            case MCP2515::ERROR_ALLTXBUSY:Serial.println("All TX buffers busy"); break;
            case MCP2515::ERROR_FAILINIT:Serial.println("Initialization failed"); break;
            case MCP2515::ERROR_FAILTX:  Serial.println("Transmission failed"); break;
            case MCP2515::ERROR_NOMSG:  Serial.println("No message available"); break;
            default: Serial.print("Unknown error code: 0x"); Serial.println(error, HEX); break;
        }
    }

    static CANController* instance;
    
public:
    CANController(int csPin, int interruptPin)
        : csPin(csPin), interruptPin(interruptPin), MCP2515(csPin), 
          hasError(false), debugEnabled(true) {
        instance = this;
    }

    void enableDebug(bool enable) {
        debugEnabled = enable;
        if (enable) {
            Serial.println("CAN: Debug enabled");
        }
    }

    bool initialize() {
        if (debugEnabled) Serial.println("CAN: Initializing...");
        
        if (MCP2515.reset() != MCP2515::ERROR_OK) {
            if (debugEnabled) Serial.println("CAN: Reset failed");
            return false;
        }
        
        if (MCP2515.setBitrate(CAN_500KBPS, CAN_CLOCK) != MCP2515::ERROR_OK) {
            if (debugEnabled) Serial.println("CAN: Bitrate set failed");
            return false;
        }
        
        if (MCP2515.setNormalMode() != MCP2515::ERROR_OK) {
            if (debugEnabled) Serial.println("CAN: Normal mode set failed");
            return false;
        }
        
        pinMode(interruptPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterruptStatic, FALLING);
        
        if (debugEnabled) Serial.println("CAN: Initialized successfully");
        return true;
    }

    void begin() {
        SPI.begin();
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
        
        if (debugEnabled) Serial.println("CAN: Starting initialization...");
        
        for (int attempt = 0; attempt < 3; attempt++) {
            if (debugEnabled) {
                Serial.print("CAN: Attempt ");
                Serial.println(attempt + 1);
            }
            
            if (initialize()) {
                hasError = false;
                if (debugEnabled) Serial.println("CAN: Started successfully");
                return;
            }
            delay(100);
        }
        
        hasError = true;
        if (debugEnabled) Serial.println("CAN: Initialization failed after 3 attempts");
    }

    void registerCallback(void (*callback)(struct can_frame*)) {
        this->callback = callback;
        if (debugEnabled) Serial.println("CAN: Callback registered");
    }

    bool sendMessage(uint32_t id, uint8_t* data = nullptr, uint8_t len = 0) {
        struct can_frame frame;
        frame.can_id = id;
        frame.can_dlc = len;
        if (data && len > 0) {
            memcpy(frame.data, data, len);
        }
        
        uint8_t result = MCP2515.sendMessage(&frame);
        if (result == MCP2515::ERROR_OK) {
            if (debugEnabled) {
                Serial.print("CAN TX: ID=0x");
                Serial.print(frame.can_id, HEX);
                Serial.print(" DLC=");
                Serial.print(frame.can_dlc);
                Serial.print(" Data:");
                for (int i = 0; i < frame.can_dlc; i++) {
                    Serial.print(" 0x");
                    Serial.print(frame.data[i], HEX);
                }
                Serial.println();
            }
            return true;
        } else {
            hasError = true;
            if (debugEnabled) {
                Serial.print("CAN TX Error: ");
                printError(result);
            }
            return false;
        }
    }

    bool getError() const { 
        if (debugEnabled && hasError) {
            Serial.println("CAN: Error flag is set");
        }
        return hasError; 
    }
    
    void reset() { 
        if (debugEnabled) Serial.println("CAN: Resetting...");
        begin(); 
    }
    
    void checkStatus() {
        if (debugEnabled) {
            uint8_t status = MCP2515.getStatus();
            Serial.print("CAN Status: 0x");
            Serial.println(status, HEX);
            
            // Simplified status interpretation
            Serial.print("RX Status: ");
            Serial.println((status & 0xC0) >> 6, BIN); // RX status bits
            
            Serial.print("TX Status: ");
            Serial.println((status & 0x38) >> 3, BIN); // TX status bits
        }
    }
};

CANController* CANController::instance = nullptr;

CANController canController(CAN_CS_PIN, CAN_INT_PIN);

void handleCANMessage(struct can_frame* frame) {
    uint8_t commandId = frame->can_id & 0x1F; // Extract command ID
    
    switch(commandId) {
        case CMD_EMERGENCY_STOP:
        case CMD_DISABLE_OUTPUT:
            voltageController.SetSetpoint(VOLTAGE_MAX);
            voltageController.SetDutyCycle(0); // Hard disable
            sendAck();
            break;
            
        case CMD_ENABLE_OUTPUT:
            sendAck(); // Will use last setpoint
            break;
            
        case CMD_RESET_FAULTS:
            faultManager.resetFaults();
            sendAck();
            break;
            
        case CMD_GET_FAULT_COUNT:
            sendFaultCount();
            break;
            
        case CMD_GET_FAULT_LIST:
            sendFaultList();
            break;
            
        case CMD_GET_VOLTAGE_IN:
            sendVoltageIn();
            break;
            
        case CMD_GET_VOLTAGE_OUT:
            sendVoltageOut();
            break;
            
        case CMD_GET_CURRENT:
            sendCurrent();
            break;
            
        case CMD_GET_TEMPERATURE:
            sendTemperature();
            break;
            
        case CMD_GET_FAN_SPEED:
            sendFanSpeed();
            break;
            
        case CMD_GET_POWER:
            sendPower();
            break;
            
        case CMD_SET_VOLTAGE:
            if(frame->can_dlc == 4) {
                float newVoltage;
                memcpy(&newVoltage, frame->data, 4);
                voltageController.SetSetpoint(newVoltage);
                sendAck();
            }
            break;
    }
}

// Implementation of all response functions
void sendAck() {
    uint8_t data[1] = {0x01};
    canController.sendMessage((DEVICE_ID << 5) | 0x1F, data, 1);
}

void sendFaultCount() {
    uint8_t data[1] = {faultManager.getFaultCount()};
    canController.sendMessage((DEVICE_ID << 5) | CMD_GET_FAULT_COUNT, data, 1);
}

void sendVoltageIn() {
    uint8_t data[4];
    float voltage = voltageController.GetVoltage();
    memcpy(data, &voltage, 4);
    canController.sendMessage((DEVICE_ID << 5) | CMD_GET_VOLTAGE_IN, data, 4);
}

void sendVoltageOut() {
    uint8_t data[4];
    float voltage = voltageController.GetVoltage(); // Or specific output voltage if different
    memcpy(data, &voltage, 4);
    canController.sendMessage((DEVICE_ID << 5) | CMD_GET_VOLTAGE_OUT, data, 4);
}

void sendCurrent() {
    uint8_t data[4];
    float current = voltageController.GetCurrent();
    memcpy(data, &current, 4);
    canController.sendMessage((DEVICE_ID << 5) | CMD_GET_CURRENT, data, 4);
}

void sendTemperature() {
    uint8_t data[4];
    float temp = 25.0; // Replace with actual temperature reading
    memcpy(data, &temp, 4);
    canController.sendMessage((DEVICE_ID << 5) | CMD_GET_TEMPERATURE, data, 4);
}

void sendFanSpeed() {
    uint8_t data[4];
    float cfm = fanController.getRollingCFMAvg();
    memcpy(data, &cfm, 4);
    canController.sendMessage((DEVICE_ID << 5) | CMD_GET_FAN_SPEED, data, 4);
}

void sendPower() {
    uint8_t data[4];
    float power = voltageController.GetPower();
    memcpy(data, &power, 4);
    canController.sendMessage((DEVICE_ID << 5) | CMD_GET_POWER, data, 4);
}

// Status message (uses different message ID scheme)
void sendCANStatus() {
    uint8_t data[8];
    float voltage = voltageController.GetVoltage();
    float current = voltageController.GetCurrent();
    uint16_t currentScaled = current * 100; // 0.01A resolution
    
    memcpy(&data[0], &voltage, 4);
    memcpy(&data[4], &currentScaled, 2);
    data[6] = voltageController.GetDutyCycle();
    data[7] = faultManager.getFaultCount();
    
    canController.sendMessage(CAN_ID_STATUS, data, 8);
}

void sendCANFault() {
    char faultName[FAULT_NAME_LENGTH];
    if(faultManager.getMostRecentFaultName(faultName, FAULT_NAME_LENGTH)) {
        uint8_t data[8] = {0};
        memcpy(data, faultName, min(strlen(faultName), 8));
        canController.sendMessage(CAN_ID_FAULT, data, 8);
    }
}

void sendFaultList() {
    char allFaults[FAULT_BUFFER_SIZE * (FAULT_NAME_LENGTH + 2)] = {0};
    faultManager.getFaultNames(allFaults, sizeof(allFaults));
    
    int len = strlen(allFaults);
    for (int i = 0; i < len; i += 8) {
        uint8_t data[8] = {0};
        int chunkLen = min(8, len - i);
        memcpy(data, allFaults + i, chunkLen);
        data[7] = (i + 8 >= len) ? 1 : 0; // End of message flag
        canController.sendMessage(CAN_ID_FAULT_LIST, data, 8);
    }
}
// Send system info over CAN
void sendSystemInfo() {
    uint8_t data[8];
    memcpy(&data[0], FW_VERSION, 4);
//    memcpy(&data[4], &systemUptime, 4);
    canController.sendMessage(CAN_ID_SYSTEM_INFO, data, 8);
}



void setup() {
    Serial.begin(9600);
    while (!Serial);

    // Initialize CAN Bus
    canController.begin();
    canController.registerCallback(handleCANMessage);

    if(canController.getError()) {
        Serial.println("CAN Bus initialization failed!");
        while(1);
    }


    PwmController::Initialize();
    FaultLED.SetState(LOW);
    StatusLED.SetState(LOW);
    StatusLED.Blink(1000);
    voltageController.SetSetpoint(DEFAULT_V_SETPOINT);
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
