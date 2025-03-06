// Define Pins In Headers
#define LED_FAULT_PIN 4       // Fault Indicator Light
#define LED_INDICATOR1_PIN 6  // Power Indicator Light
#define LED_INDICATOR2_PIN 8  // Output Enabled Indicator
#define LED_INDICATOR3_PIN 9  // Temperature OK
#define FAN_TACH 3            // Fan Tachometer
#define FAN_PWM 5             // Fan PWM duty cycle control

// Constants for airflow calculation
#define MAX_RPM 5500          // Maximum RPM at 100% PWM
#define MAX_AIRFLOW_CFM 225.0 // Maximum airflow in CFM at 100% PWM
#define DUCT_AREA_SQFT 0.25   // Cross-sectional area of the duct in square feet

// Thresholds for airflow in LFM
#define MIN_AIRFLOW_LFM 50.0  // Minimum acceptable airflow in LFM
#define MAX_AIRFLOW_LFM 2500.0 // Maximum acceptable airflow in LFM

// Temperature and fan speed settings
#define MIN_TEMP 30.0         // Minimum temperature for fan speed adjustment
#define MAX_TEMP 80.0         // Maximum temperature for fan speed adjustment
#define BASE_FAN_SPEED 25     // Base fan speed (0-255)

// Fault management constants
#define MIN_RESET_TIME_SEC 5      // Minimum time a fault must be active before it can auto-reset
#define MAX_AUTO_RESET_COUNT 3    // Maximum number of times a fault can auto-reset
#define AUTO_RESET_FAULT_TIME 60  // Time after which an auto-reset fault is forgotten
#define FAULT_BUFFER_SIZE 4       // Size of the fault buffer

// Indicator Class
class Indicator {
  public:
    // Constructor
    Indicator(int _Pin) {
      Pin_ = _Pin;
      pinMode(_Pin, OUTPUT);
    }

    // Toggle the LED state
    bool ToggleIndicator() {
      State_ = !State_;
      digitalWrite(Pin_, State_);
      return State_;
    }

    // Set the LED state explicitly
    void SetState(bool _NewState) {
      State_ = _NewState;
      digitalWrite(Pin_, State_);
    }

    // Blink the LED at a fixed frequency
    void Blink(int _Frequency_ms) {
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis_ >= _Frequency_ms) {
        previousMillis_ = currentMillis;
        ToggleIndicator();
      }
    }

    // Blink a specific pattern (e.g., "SLS" for Short-Long-Short)
    void BlinkCode(const String& blinkCode) {
      unsigned long currentMillis = millis();

      switch (blinkState_) {
        case IDLE:
          // Start the blink sequence immediately
          blinkCodeIndex_ = 0;
          blinkState_ = BLINK_ON;
          break;

        case BLINK_ON:
          // Turn on the LED and start the blink duration
          if (blinkCodeIndex_ < blinkCode.length()) {
            SetState(true);
            blinkStartTime_ = currentMillis;
            blinkState_ = BLINK_WAIT;
          } else {
            // End of sequence, wait for 1.5s gap before repeating
            blinkState_ = END_GAP;
            blinkStartTime_ = currentMillis;
          }
          break;

        case BLINK_WAIT:
          // Wait for the blink duration to complete
          if (currentMillis - blinkStartTime_ >= getBlinkDuration(blinkCode[blinkCodeIndex_])) {
            SetState(false);
            blinkCodeIndex_++;
            blinkState_ = BLINK_GAP;
            blinkStartTime_ = currentMillis;
          }
          break;

        case BLINK_GAP:
          // Wait for the gap between blinks
          if (currentMillis - blinkStartTime_ >= 300) {
            blinkState_ = BLINK_ON;
          }
          break;

        case END_GAP:
          // Wait for the 1.5s gap after the sequence ends
          if (currentMillis - blinkStartTime_ >= 1500) {
            blinkState_ = IDLE; // Restart the sequence
          }
          break;
      }
    }

  private:
    // States for the blink code state machine
    enum BlinkState {
      IDLE,      // Waiting to start the blink sequence
      BLINK_ON,  // LED is on
      BLINK_WAIT, // Waiting for the blink duration to complete
      BLINK_GAP,  // Waiting for the gap between blinks
      END_GAP    // Waiting for the 1.5s gap after the sequence ends
    };

    int Pin_; // Output Pin
    bool State_ = false; // Current LED State
    unsigned long previousMillis_ = 0; // For the Blink method
    int blinkCodeIndex_ = 0; // Current position in the blink code
    BlinkState blinkState_ = IDLE; // Current state of the blink sequence
    unsigned long blinkStartTime_ = 0; // When the current blink started

    // Helper function to get the duration of a blink based on the code character
    unsigned long getBlinkDuration(char blinkChar) {
      return (blinkChar == 'L') ? 600 : 300; // Long blink: 600ms, Short blink: 300ms
    }
};

class VoltageSensor {
    public:
        // Constructor to initialize the sensor with an analog pin and voltage divider ratio
        VoltageSensor(int analogPin, float dividerRatio)
            : analogPin_(analogPin), dividerRatio_(dividerRatio) {}

        // Method to read the voltage from the sensor
        float readVoltage() {
            // Read the analog value from the pin (0-1023 for Arduino)
            int analogValue = analogRead(analogPin_);

            // Convert the analog value to a voltage (0-5V for Arduino)
            float voltage = analogValue * (5.0 / 1023.0);

            // Apply the voltage divider ratio to get the actual voltage
            return voltage / dividerRatio_;
        }

    private:
        int analogPin_;        // Analog input pin
        float dividerRatio_;   // Voltage divider ratio
};

class FanController {
    public:
        // Constructor
        FanController(int rpmPin, int pwmPin)
            : rpmPin(rpmPin), pwmPin(pwmPin), rpm(0), lastTime(0), pulseCount(0),
              lastAirflowUpdate(0), cachedAirflowLFM(0) {
            pinMode(rpmPin, INPUT_PULLUP);
            pinMode(pwmPin, OUTPUT);
            attachInterrupt(digitalPinToInterrupt(rpmPin), rpmInterruptStatic, FALLING);
        }

        // Set the fan speed (0-255)
        void setSpeed(int speed) {
            if (speed < 0) speed = 0;
            if (speed > 255) speed = 255;
            analogWrite(pwmPin, speed);
        }

        // Adjust fan speed based on temperature
        void setFanSpeed(float currentTemp) {
            int speed = BASE_FAN_SPEED;
            if (currentTemp > MAX_TEMP) {
                speed = 255;
            } else if (currentTemp > MIN_TEMP) {
                speed = map(currentTemp, MIN_TEMP, MAX_TEMP, BASE_FAN_SPEED, 255);
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
};

// Define the static member
FanController* FanController::instance = nullptr;

#define FAULT_NAME_LENGTH 32
#define BLINK_CODE_LENGTH 8

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
      }
    }

    // Assert a fault
    void assertFault(const char* faultName, const char* blinkCode, bool isAutoReset = true, bool overwriteFaults = false) {
      
      if (overwriteFaults) {
        // Clear existing faults
        resetFaults();

        // Now add our fault to the buffer
        strncpy(faultBuffer[0].name, faultName, FAULT_NAME_LENGTH);
        strncpy(faultBuffer[0].blinkCode, blinkCode, BLINK_CODE_LENGTH);
        faultBuffer[0].resetCount = 0;
        faultBuffer[0].startTime = millis();
        faultBuffer[0].isAutoReset = isAutoReset;
        faultCount = 1;
        return;
      }

      // Check if the fault buffer is full
      if (faultCount >= FAULT_BUFFER_SIZE) {
        // Assert the FAULT_BUFFER_OVERFLOW fault if the buffer is full
        bool overflowAsserted = false;
        for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
          if (strcmp(faultBuffer[i].name, "FAULT_BUFFER_OVERFLOW") == 0) {
            // If FAULT_BUFFER_OVERFLOW is already in the buffer, do not add it again
            overflowAsserted = true;
            break;
          }
        }
        if (!overflowAsserted) {
          // Add the FAULT_BUFFER_OVERFLOW fault to the buffer
          assertFault("FAULT_BUFFER_OVERFLOW", "SSSS", false, true);
        }
        return;
      }

      // Add the fault to the buffer
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        if (faultBuffer[i].name[0] == '\0') {
          strncpy(faultBuffer[i].name, faultName, FAULT_NAME_LENGTH);
          strncpy(faultBuffer[i].blinkCode, blinkCode, BLINK_CODE_LENGTH);
          faultBuffer[i].resetCount = 0;
          faultBuffer[i].startTime = millis();
          faultBuffer[i].isAutoReset = isAutoReset;
          faultCount++;
          break;
        }
      }
    }



    void resetFaults() {
      // Clear existing faults
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        faultBuffer[i].name[0] = '\0';
        faultBuffer[i].blinkCode[0] = '\0';
        faultBuffer[i].resetCount = 0;
        faultBuffer[i].startTime = 0;
        faultBuffer[i].isAutoReset = false;
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
        if (faultBuffer[i].name[0] != '\0') {
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
        if (faultBuffer[i].name[0] != '\0') {
          strncpy(blinkCode, faultBuffer[i].blinkCode, maxLength);
          break;
        }
      }
    }

    // Get the current fault count
    int getFaultCount() {
      return faultCount;
    }

  private:
    struct Fault {
      char name[FAULT_NAME_LENGTH];
      char blinkCode[BLINK_CODE_LENGTH];
      int resetCount;
      unsigned long startTime;
      bool isAutoReset;
    };

    Fault faultBuffer[FAULT_BUFFER_SIZE];
    int faultCount = 0;
};


// -- Define Global Objects --//
Indicator IndicatorFault(LED_FAULT_PIN);
Indicator IndicatorPower(LED_INDICATOR1_PIN);
Indicator IndicatorOutput(LED_INDICATOR2_PIN);
Indicator IndicatorTempOK(LED_INDICATOR3_PIN);

VoltageSensor VoltageSensorVIN(A2, 6000.0 / 120.0);

FanController SystemFan(FAN_TACH, FAN_PWM);

FaultManager faultManager;

bool FaultAsserted = false;

//-- System Run Modes --//
void setup() {
    // Initialize the static instance pointer
    FanController::instance = &SystemFan;

    // Initialize serial communication
    Serial.begin(9600);

    // Led Self Test
    IndicatorFault.SetState(true);
    IndicatorPower.SetState(true);
    IndicatorOutput.SetState(true);
    IndicatorTempOK.SetState(true);
    delay(1000);
    IndicatorFault.SetState(false);
    IndicatorPower.SetState(false);
    IndicatorOutput.SetState(false);
    IndicatorTempOK.SetState(false);

    // Power is okay, since the LED is on...
    IndicatorPower.SetState(true);

    // Fan Baseline Duty Cycle
    SystemFan.setSpeed(BASE_FAN_SPEED);
}

void loop() {
    // Update Indicators
    // IndicatorFault.SetState(faultManager.getFaultCount() > 0);
    IndicatorPower.Blink(750);

    // Get the blink code for the most recent fault
    char blinkCode[BLINK_CODE_LENGTH];
    if (faultManager.getFaultCount() > 0) {
      faultManager.getMostRecentFaultBlinkCode(blinkCode, BLINK_CODE_LENGTH);
      if (blinkCode[0] != '\0') {
          IndicatorFault.BlinkCode(blinkCode);
      }
    } else {
      IndicatorFault.SetState(false);
    }

    // Example temperature value (replace with actual sensor reading)
    float currentTemp = 20.0;

    // Adjust fan speed based on temperature
    SystemFan.setFanSpeed(currentTemp);

    // Print airflow in LFM every second
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 1000) {
        lastPrintTime = currentTime;
        SystemFan.printAirflowLFM();

        // Check for airflow errors
        if (SystemFan.checkError()) {
            Serial.println("Error: Airflow out of acceptable range!");
            faultManager.assertFault("AIRFLOW_ERROR", "SSSL");
        }
    }


    // Various faults for future reference:
    // // Trigger faults with specific blink codes
    // faultManager.assertFault("AIRFLOW_OUT_OF_RANGE", "SSSL");
    // faultManager.assertFault("TEMPSENSE_1_OUT_OF_RANGE", "SSLS");
    // faultManager.assertFault("TEMPSENSE_2_OUT_OF_RANGE", "SSLL");
    // faultManager.assertFault("VIN_UNDERVOLTAGE", "SLSS");
    // faultManager.assertFault("VIN_OVERVOLTAGE", "SLSL");
    // faultManager.assertFault("VOUT_UNDERVOLTAGE", "SLLS");
    // faultManager.assertFault("VOUT_OVERVOLTAGE", "SLLL");
    // faultManager.assertFault("PHASE_A_OVERCURRENT", "LSSS");
    // faultManager.assertFault("PHASE_B_OVERCURRENT", "LSSL");
    // faultManager.assertFault("CANBUS_COMM_ERROR", "LSLS");
    // faultManager.assertFault("CANBUS_REMOTE_ESTOP", "LSLL");

    // Clear and forget auto-reset faults
    faultManager.clearAutoResetFaults();
    faultManager.forgetAutoResetFaults();

    // Print current faults
    if (faultManager.getFaultCount() > 0) {
        char faultNames[FAULT_BUFFER_SIZE * FAULT_NAME_LENGTH];
        faultManager.getFaultNames(faultNames, sizeof(faultNames));
        Serial.print("Current Faults: ");
        Serial.println(faultNames);
    }
}
