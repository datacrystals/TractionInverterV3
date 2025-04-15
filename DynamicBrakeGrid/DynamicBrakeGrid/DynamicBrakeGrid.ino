#include "Common/VoltageSensor.h"
#include "Common/ACS758CurrentSensor.h"

// Hardware Configuration
#define MAIN_PWM_PIN         6
#define SECONDARY_PWM_PIN    9
#define STATUS_LED1          4
#define STATUS_LED2          5
#define VOLTAGE_SENSOR_PIN   A2
#define CURRENT_SENSOR_PIN   A0

// System Constants
#define VOLTAGE_MIN          1.0f        // Minimum system voltage (V)
#define VOLTAGE_MAX          450.0f      // Maximum system voltage (V)
#define CURRENT_MAX          200.0f      // Maximum current (A)
#define CONTROL_INTERVAL_MS  100         // Control loop interval
#define LOG_INTERVAL_MS      1000        // Data logging interval
#define SAMPLE_WINDOW_SIZE   10          // Moving average window size
#define SECONDARY_PWM_VALUE  250         // Fixed PWM value (250/255 ≈ 98%)

// Control Parameters
#define KP                  5.0f        // Proportional gain (now correctly oriented)
#define MAX_DUTY_CHANGE     1000          // Max duty cycle change per second (%/s)

// System State
VoltageSensor voltageSensor(VOLTAGE_SENSOR_PIN, 6000.0f, 120.0f, VOLTAGE_MIN, VOLTAGE_MAX, 100);
ACS758CurrentSensor currentSensor(CURRENT_SENSOR_PIN, 20.0f, 0.6f);

float setpointVoltage = 25.0f;          // Default setpoint
float measuredVoltage = 0.0f;
int currentDuty = 0;                    // Current duty cycle (0-255)

// Timing control
uint32_t lastControlTime = 0;
uint32_t lastLogTime = 0;

// Moving average buffer
float voltageSamples[SAMPLE_WINDOW_SIZE];
uint8_t sampleIndex = 0;

void initializeSystem() {
  pinMode(MAIN_PWM_PIN, OUTPUT);
  pinMode(SECONDARY_PWM_PIN, OUTPUT);
  pinMode(STATUS_LED1, OUTPUT);
  pinMode(STATUS_LED2, OUTPUT);

  analogWrite(SECONDARY_PWM_PIN, SECONDARY_PWM_VALUE);
  currentSensor.setCurrentRange(0.0f, CURRENT_MAX);

  for (auto& sample : voltageSamples) {
    sample = setpointVoltage;
  }

  Serial.begin(9600);
  while (!Serial);
  Serial.println("System initialized with setpoint: 25.00V");
}

float readFilteredVoltage() {
  voltageSamples[sampleIndex] = voltageSensor.readVoltage();
  sampleIndex = (sampleIndex + 1) % SAMPLE_WINDOW_SIZE;

  float sum = 0.0f;
  for (const auto& sample : voltageSamples) {
    sum += sample;
  }
  return sum / SAMPLE_WINDOW_SIZE;
}

void updateControl() {
  uint32_t now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0f; // Convert to seconds
    lastControlTime = now;

    // Calculate error (INVERTED DIRECTION)
    float error = measuredVoltage - setpointVoltage;  // Now positive when too high
    
    // Calculate desired duty change (INVERTED RESPONSE)
    float desiredChange = KP * error;  // Negative sign gives correct response
    
    // Limit rate of change (convert %/s to 0-255 range)
    float maxChange = (MAX_DUTY_CHANGE * dt) * 2.55f;
    desiredChange = constrain(desiredChange, -maxChange, maxChange);
    
    // Update duty cycle
    currentDuty += (int)desiredChange;
    currentDuty = constrain(currentDuty, 0, 255);
    
    analogWrite(MAIN_PWM_PIN, currentDuty);

    // Debug output
//    Serial.print("Control - Error: ");
//    Serial.print(error);
//    Serial.print(" | Duty Change: ");
//    Serial.print(desiredChange);
//    Serial.print(" | New Duty: ");
//    Serial.println(currentDuty);
  }
}

void updateSystemStatus() {
  static uint32_t lastBlinkTime = 0;
  constexpr uint32_t BLINK_INTERVAL = 500;

  if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
    digitalWrite(STATUS_LED1, !digitalRead(STATUS_LED1));
    digitalWrite(STATUS_LED2, !digitalRead(STATUS_LED2));
    lastBlinkTime = millis();
  }
}

void handleSerialCommunication() {
  static String inputBuffer;
  
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      float newSetpoint = inputBuffer.toFloat();
      if (newSetpoint >= VOLTAGE_MIN && newSetpoint <= VOLTAGE_MAX) {
        setpointVoltage = newSetpoint;
        Serial.print("Setpoint updated to: ");
        Serial.print(setpointVoltage, 2);
        Serial.println("V");
      } else {
        Serial.println("Error: Setpoint must be between " + String(VOLTAGE_MIN) + "V and " + String(VOLTAGE_MAX) + "V");
      }
      inputBuffer = "";
    } else if (isDigit(c) || c == '.' || c == '-') {
      inputBuffer += c;
    }
  }
}

void logSystemStatus() {
  uint32_t now = millis();
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;
    
    Serial.print("Setpoint: ");
    Serial.print(setpointVoltage, 2);
    Serial.print("V | Measured: ");
    Serial.print(measuredVoltage, 2);
    Serial.print("V | PWM: ");
    Serial.print((currentDuty / 255.0f) * 100.0f, 1);
    Serial.print("% | Current: ");
    Serial.print(currentSensor.getCurrent(), 2);
    Serial.println("A");

    if (currentSensor.getError()) {
      Serial.println("WARNING: Current out of range!");
    }
  }
}

void setup() {
  initializeSystem();
}

void loop() {
  measuredVoltage = readFilteredVoltage();
  updateControl();
  handleSerialCommunication();
  updateSystemStatus();
  logSystemStatus();
}
