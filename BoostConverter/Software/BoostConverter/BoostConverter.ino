// Define Pins In Headers
#define LED_FAULT_PIN 4       // Fault Indicator Light
#define LED_INDICATOR1_PIN 6  // Power Indicator Light
#define LED_INDICATOR2_PIN 8  // Output Enabled Indicator
#define LED_INDICATOR3_PIN 9  // Temperature OK
#define FAN_TACH 3            // Fan Tachometer
#define FAN_PWM 5             // Fan PWM duty cycle control
#define OUTPUT_ENABLE_PIN 7   // Pin which enables output, must be high for output to be on
#define PHASE_1_TEMP_SENSOR_PIN A4 // Phase A Temp Sensor
#define PHASE_2_TEMP_SENSOR_PIN A5 // Phase A Temp Sensor


// Constants for airflow calculation
#define MAX_RPM 5500          // Maximum RPM at 100% PWM
#define MAX_AIRFLOW_CFM 225.0 // Maximum airflow in CFM at 100% PWM
#define DUCT_AREA_SQFT 0.15   // Cross-sectional area of the duct in square feet

// Thresholds for airflow in LFM
#define MIN_AIRFLOW_LFM 50.0  // Minimum acceptable airflow in LFM
#define MAX_AIRFLOW_LFM 2500.0 // Maximum acceptable airflow in LFM

// Temperature and fan speed settings
#define MIN_TEMP 30.0         // Minimum temperature for fan speed adjustment
#define MAX_TEMP 120.0         // Maximum temperature for fan speed adjustment
#define BASE_FAN_SPEED 25     // Base fan speed (0-255)

// Voltage Sensor settings
#define VIN_MIN_V 24
#define VIN_MAX_V 185
#define VOUT_MIN_V 320
#define VOUT_MAX_V 360
#define VSENSE_GRACE_TIME_MS 2000 // Time which voltage is allowed to be out of spec before error is asserted

// Current Sensor Settings
#define PHASE_1_CURRENT_SENSE_PIN A0 // Pin which is used to sample the current for phase 1
#define PHASE_2_CURRENT_SENSE_PIN A1 // Pin which is used to sample the current for phase 2
#define CURRENT_MIN_VALUE_A -5       // Min current value below which current error thrown
#define CURRENT_MAX_VALUE_A 180      // Max current value above which current error thrown

// Fault management constants
#define MIN_RESET_TIME_SEC 5      // Minimum time a fault must be active before it can auto-reset
#define MAX_AUTO_RESET_COUNT 3    // Maximum number of times a fault can auto-reset
#define AUTO_RESET_FAULT_TIME 60  // Time after which an auto-reset fault is forgotten
#define FAULT_BUFFER_SIZE 4       // Size of the fault buffer
#define FAULT_DURATION_THRESHOLD 5000 // Duration in milliseconds to consider a fault as proper

#define FAULT_BUFFER_OVERFLOW "FAULT_BUFFER_OVERFLOW"
#define FAULT_NAME_LENGTH 32
#define BLINK_CODE_LENGTH 8

// Debug Info
#define ENABLE_SERIAL_PRINT






// Indicator Class
class Indicator {
  public:
    // Constructor
    Indicator(int pin) : Pin_(pin), State_(false), previousMillis_(0),
                          blinkCodeIndex_(0), blinkState_(IDLE), blinkStartTime_(0) {
      pinMode(Pin_, OUTPUT);
    }

    // Toggle the LED state
    bool ToggleIndicator() {
      State_ = !State_;
      digitalWrite(Pin_, State_);
      return State_;
    }

    // Set the LED state explicitly
    void SetState(bool newState) {
      State_ = newState;
      digitalWrite(Pin_, State_);
    }

    // Blink the LED at a fixed frequency
    void Blink(int frequencyMs) {
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis_ >= frequencyMs) {
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
            // End of sequence, wait for the end gap before repeating
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
          if (currentMillis - blinkStartTime_ >= BLINK_GAP_DURATION) {
            blinkState_ = BLINK_ON;
          }
          break;

        case END_GAP:
          // Wait for the end gap after the sequence ends
          if (currentMillis - blinkStartTime_ >= END_GAP_DURATION) {
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
      END_GAP    // Waiting for the end gap after the sequence ends
    };

    // Constants for timing
    const unsigned long LONG_BLINK_DURATION = 1000; // Long blink duration in ms
    const unsigned long SHORT_BLINK_DURATION = 500; // Short blink duration in ms
    const unsigned long BLINK_GAP_DURATION = 500; // Gap between blinks in ms
    const unsigned long END_GAP_DURATION = 1750; // End gap after sequence in ms

    int Pin_; // Output Pin
    bool State_; // Current LED State
    unsigned long previousMillis_; // For the Blink method
    int blinkCodeIndex_; // Current position in the blink code
    BlinkState blinkState_; // Current state of the blink sequence
    unsigned long blinkStartTime_; // When the current blink started

    // Helper function to get the duration of a blink based on the code character
    unsigned long getBlinkDuration(char blinkChar) {
      return (blinkChar == 'L') ? LONG_BLINK_DURATION : SHORT_BLINK_DURATION;
    }
};

class VoltageSensor {
  public:
      // Constructor to initialize the sensor with an analog pin, voltage divider ratio, and acceptable range
      VoltageSensor(int analogPin, float dividerRatio, float vmin, float vmax, unsigned long errorDuration)
          : analogPin_(analogPin), dividerRatio_(dividerRatio), vmin_(vmin), vmax_(vmax),
            errorDuration_(errorDuration), errorStartTime_(0), error_(false) {}

      // Method to read the voltage from the sensor
      float readVoltage() {
          // Read the analog value from the pin (0-1023 for Arduino)
          int analogValue = analogRead(analogPin_);

          // Convert the analog value to a voltage (0-5V for Arduino)
          float voltage = analogValue * (5.0 / 1023.0);

          // Apply the voltage divider ratio to get the actual voltage
          voltage /= dividerRatio_;

          // Check if the voltage is within the acceptable range
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
      int analogPin_;             // Analog input pin
      float dividerRatio_;       // Voltage divider ratio
      float vmin_;               // Minimum acceptable voltage
      float vmax_;               // Maximum acceptable voltage
      unsigned long errorDuration_; // Duration for which voltage must be out of range to trigger error
      unsigned long errorStartTime_; // Time when the voltage first went out of range
      bool error_;               // Error flag indicating if voltage is out of range
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

class TMP36Sensor {
  public:
      // Constructor
      TMP36Sensor(int pin, float minTempETH = -40.0, float maxTempETH = 125.0, float minTemp = -40.0, float maxTemp = 125.0)
          : pin(pin), minTempETH(minTempETH), maxTempETH(maxTempETH), minTemp(minTemp), maxTemp(maxTemp) {}

      // Method to read the temperature
      float readTemperature() {
          int sensorValue = analogRead(pin);  // Read the analog value from the sensor
          float voltage = sensorValue * (5.0 / 1023.0);  // Convert to voltage (assuming 5V reference)
          float temperatureC = (voltage - 0.5) * 100.0;  // Convert voltage to temperature in Celsius
          return temperatureC;
      }

      // Method to check if the temperature is within the specified range
      bool getFault() {
          float temperatureC = readTemperature();
          return (temperatureC < minTempETH || temperatureC > maxTempETH);
      }

  private:
      int pin;  // Analog pin where the sensor is connected
      float minTemp;  // Minimum temperature threshold
      float minTempETH;  // Minimum temperature threshold
      float maxTemp;  // Maximum temperature threshold
      float maxTempETH;  // Maximum temperature threshold
};

class ACS758CurrentSensor {
  public:
      // Constructor
      ACS758CurrentSensor(int voutPin, float sensitivity = 20.0, float quiescentVoltage = 0.6)
          : voutPin(voutPin), sensitivity(sensitivity), quiescentVoltage(quiescentVoltage) {}

      // Set the acceptable current range
      void setCurrentRange(float minCurrent, float maxCurrent) {
          this->minCurrent = minCurrent;
          this->maxCurrent = maxCurrent;
      }

      // Read the current from the sensor
      float getCurrent() {
          int sensorValue = analogRead(voutPin);
          float voltage = sensorValue * (5.0 / 1023.0); // Convert to voltage
          float current = (voltage - quiescentVoltage) / sensitivity;
          return current;
      }

      // Check if the current is within the acceptable range
      bool getError() {
          float current = getCurrent();
          return (current < minCurrent || current > maxCurrent);
      }

  private:
      int voutPin;
      float sensitivity;
      float quiescentVoltage;
      float minCurrent = 0.0;
      float maxCurrent = 200.0;
};


// -- Define Global Objects --//
Indicator IndicatorFault(LED_FAULT_PIN);
Indicator IndicatorPower(LED_INDICATOR1_PIN);
Indicator IndicatorOutput(LED_INDICATOR2_PIN);
Indicator IndicatorTempOK(LED_INDICATOR3_PIN);

VoltageSensor VoltageSensorVIN(A2, 6000.0 / 120.0, VIN_MIN_V, VIN_MAX_V, VSENSE_GRACE_TIME_MS);
VoltageSensor VoltageSensorVOUT(A3, 6000.0 / 40.0, VOUT_MIN_V, VOUT_MAX_V, VSENSE_GRACE_TIME_MS);

TMP36Sensor Phase1TempSense(PHASE_1_TEMP_SENSOR_PIN, MIN_TEMP, MAX_TEMP);
TMP36Sensor Phase2TempSense(PHASE_2_TEMP_SENSOR_PIN, MIN_TEMP, MAX_TEMP);

ACS758CurrentSensor Phase1CurrentSense(PHASE_1_CURRENT_SENSE_PIN);
ACS758CurrentSensor Phase2CurrentSense(PHASE_2_CURRENT_SENSE_PIN);


FanController SystemFan(FAN_TACH, FAN_PWM);

FaultManager faultManager;

bool OutputEnabled = true; // Indicates if the output has been enabled or not

float CurrentTemp_C = 30.0; // Temperature which is used to drive the fan control




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

    // Setup Output Enable Pin
    pinMode(OUTPUT_ENABLE_PIN, OUTPUT);

    // Init Current Sensors
    Phase1CurrentSense.setCurrentRange(CURRENT_MIN_VALUE_A, CURRENT_MAX_VALUE_A);
    Phase2CurrentSense.setCurrentRange(CURRENT_MIN_VALUE_A, CURRENT_MAX_VALUE_A);
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

 

    // Adjust fan speed based on temperature
    SystemFan.setFanSpeed(CurrentTemp_C);

    // Print airflow in LFM every second
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 1000) {
        lastPrintTime = currentTime;
        SystemFan.printAirflowLFM();

        // Check for airflow errors
        if (SystemFan.checkError()) {
            faultManager.assertFault("AIRFLOW_OUT_OF_RANGE", "SSSL");
        }


        // ---- UPDATE VOLTAGE SENSOR DATA ---- //
        // Read Voltage Sensors
        float VIN  = VoltageSensorVIN.readVoltage();
        float VOUT = VoltageSensorVOUT.readVoltage();

#ifdef ENABLE_SERIAL_PRINT
        Serial.print("VIN Voltage: ");
        Serial.print(VIN);
        Serial.println("V");

        Serial.print("VOUT Voltage: ");
        Serial.print(VOUT);
        Serial.println("V");
#endif
        // Get voltage sense errors
        if (VoltageSensorVOUT.isUndervoltage() && OutputEnabled) {
            faultManager.assertFault("VOUT_UNDERVOLTAGE", "SLLS");
        }
        if (VoltageSensorVOUT.isOvervoltage()) {
            faultManager.assertFault("VOUT_OVERVOLTAGE", "SLLL");
        }
        if (VoltageSensorVIN.isUndervoltage()) {
            faultManager.assertFault("VIN_UNDERVOLTAGE", "SLSS");
        }
        if (VoltageSensorVIN.isOvervoltage()) {
            faultManager.assertFault("VIN_OVERVOLTAGE", "SLSL");
        }



        // ---- UPDATE TEMPERATURE SENSOR DATA ---- //
        // Read Temperature Sensors
        float Phase1Temp_C = Phase1TempSense.readTemperature();
        float Phase2Temp_C = Phase2TempSense.readTemperature();

        // Select Max Temperature
        if (Phase1Temp_C > Phase2Temp_C) {
          // CurrentTemp_C = Phase1Temp_C;
        } else {
          // CurrentTemp_C = Phase2Temp_C;
        }

#ifdef ENABLE_SERIAL_PRINT
        Serial.print("Phase1 Temp: ");
        Serial.print(Phase1Temp_C);
        Serial.println("C");

        Serial.print("Phase2 Temp: ");
        Serial.print(Phase2Temp_C);
        Serial.println("C");
#endif
        // Temperature Sensor Faults
        if (Phase1TempSense.getFault()) {
            faultManager.assertFault("TEMPSENSE_1_OUT_OF_RANGE", "SSLS");
        }
        if (Phase2TempSense.getFault()) {
            faultManager.assertFault("TEMPSENSE_2_OUT_OF_RANGE", "SSLL");
        }

        // Update thermal indicator led
        IndicatorTempOK.SetState(!(Phase1TempSense.getFault() || Phase2TempSense.getFault() || SystemFan.checkError()));




        // ---- Current Sensor Monitoring ---- //
        float Phase1Current_A = Phase1CurrentSense.getCurrent();
        float Phase2Current_A = Phase2CurrentSense.getCurrent();

#ifdef ENABLE_SERIAL_PRINT
        Serial.print("Phase1 Current: ");
        Serial.print(Phase1Current_A);
        Serial.println("A");

        Serial.print("Phase2 Current: ");
        Serial.print(Phase2Current_A);
        Serial.println("A");
#endif        

        if (Phase1CurrentSense.getError()) {
            faultManager.assertFault("PHASE_A_OVERCURRENT", "LSSS");
        }
        if (Phase2CurrentSense.getError()) {
            faultManager.assertFault("PHASE_B_OVERCURRENT", "LSSL");
        }



        // ---- SERIAL LOG FAULTS ---- //
        // Print current faults
        if (faultManager.getFaultCount() > 0) {
            char faultNames[FAULT_BUFFER_SIZE * FAULT_NAME_LENGTH];
            faultManager.getFaultNames(faultNames, sizeof(faultNames));
            Serial.print("Current Faults: ");
            Serial.println(faultNames);
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

    // Check that there are no errors, and disable output if there are
    if (faultManager.getFaultCount() != 0) {
      OutputEnabled = false;
    }
    IndicatorOutput.SetState(OutputEnabled);
    digitalWrite(OUTPUT_ENABLE_PIN, OutputEnabled);

    // Clear and forget auto-reset faults
    faultManager.clearAutoResetFaults();
    faultManager.forgetAutoResetFaults();


}
