
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
