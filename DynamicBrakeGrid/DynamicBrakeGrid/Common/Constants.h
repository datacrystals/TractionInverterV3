#pragma once 

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
