#include <AM2302-Sensor.h>

// Define the pin for the AM2302 sensor
const int AM2302_PIN = 15;  // GPIO 15 on ESP32-S3

// Create AM2302 sensor object
AM2302::AM2302_Sensor am2302{AM2302_PIN};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32-S3 with AM2302 (DHT22) Sensor");
  Serial.println("Using GPIO 15");
  Serial.println("Initializing sensor...");
  
  // Initialize the sensor
  am2302.begin();
  
  Serial.println("Sensor initialized. Starting readings...");
  Serial.println("Status | Temperature (°C) | Humidity (%)");
  Serial.println("-------|------------------|-------------");
}

void loop() {
  // Read sensor data
  auto status = am2302.read();
  
  // Print status
  Serial.print(AM2302::AM2302_Sensor::get_sensorState(status));
  Serial.print(" | ");
  
  // Check if reading was successful
  if (status == 0) {  // AM2302_READ_OK
    // Get temperature and humidity
    float temperature = am2302.get_Temperature();
    float humidity = am2302.get_Humidity();
    
    Serial.print(temperature, 1);
    Serial.print("°C");
    Serial.print("          | ");
    Serial.print(humidity, 1);
    Serial.println("%");
  } else {
    // Handle error cases
    Serial.println("Error reading sensor");
    
    switch (status) {
      case -1:  // AM2302_ERROR_CHECKSUM
        Serial.println("Checksum error - data corrupted");
        break;
      case -2:  // AM2302_ERROR_TIMEOUT
        Serial.println("Timeout error - sensor not responding");
        break;
      case -3:  // AM2302_ERROR_READ_FREQ
        Serial.println("Read frequency error - reading too fast");
        break;
      default:
        Serial.println("Unknown error");
        break;
    }
  }
  
  // Wait 2 seconds between readings (AM2302 requires minimum 2 second intervals)
  delay(2000);
}
