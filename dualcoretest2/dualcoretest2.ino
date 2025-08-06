#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Pin definitions
#define PIR_PIN 12
#define STATUS_LED_PIN 21
#define BRIGHTNESS_LED_PIN 42  // LED that gets progressively brighter

// LCD setup (I2C address 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// PWM properties for brightness LED (NEW API - no channel needed)
const int pwmFreq = 5000;      // 5kHz frequency
const int pwmResolution = 8;   // 8-bit resolution (0-255)

// Task handles
TaskHandle_t motionTaskHandle;
TaskHandle_t displayTaskHandle;

// Queue handle for communication between cores
QueueHandle_t motionQueue;

// Data structure for motion data
typedef struct {
    bool motionDetected;
    uint32_t timestamp;
    uint32_t duration;
    uint8_t brightnessLevel;
    String message;
} MotionData;

// Global variables
volatile bool motionDetected = false;
uint32_t motionStartTime = 0;
uint32_t motionDuration = 0;
uint8_t currentBrightness = 0;

// Motion duration thresholds for brightness levels
const uint32_t maxBrightnessTime = 10000; // 10 seconds for full brightness

// Interrupt service routine for PIR sensor
void ARDUINO_ISR_ATTR motionISR() {
    motionDetected = true;
    motionStartTime = millis();
}

// Function to calculate brightness based on motion duration
uint8_t calculateBrightness(uint32_t duration) {
    if (duration == 0) return 0;
    
    // Map duration (0 to maxBrightnessTime) to brightness (0 to 255)
    uint8_t brightness = map(duration, 0, maxBrightnessTime, 0, 255);
    
    // Ensure we don't exceed maximum brightness
    if (brightness > 255) brightness = 255;
    
    return brightness;
}

// Task running on Core 0 - Motion Detection and PWM Control
void motionDetectionTask(void *pvParameters) {
    Serial.println("Motion Detection Task started on Core: " + String(xPortGetCoreID()));
    
    while(1) {
        if (motionDetected) {
            // Calculate motion duration
            motionDuration = millis() - motionStartTime;
            
            // Calculate brightness based on motion duration
            currentBrightness = calculateBrightness(motionDuration);
            
            // Update PWM brightness on pin 42 using NEW API
            analogWrite(BRIGHTNESS_LED_PIN, currentBrightness);
            
            // Create motion data structure
            MotionData data;
            data.motionDetected = true;
            data.timestamp = millis();
            data.duration = motionDuration;
            data.brightnessLevel = currentBrightness;
            
            // Generate appropriate message based on duration
            if (motionDuration < 2000) {
                data.message = "Motion Starting";
            } else if (motionDuration < 5000) {
                data.message = "Motion Sustained";
            } else {
                data.message = "Long Motion!";
            }
            
            // Send data to display task via queue
            xQueueSend(motionQueue, &data, portMAX_DELAY);
            
            // Turn on status LED
            digitalWrite(STATUS_LED_PIN, HIGH);
            
            // Print to serial for debugging
            Serial.println("Motion Duration: " + String(motionDuration) + "ms, Brightness: " + String(currentBrightness));
            
            // Check if motion is still active
            if (digitalRead(PIR_PIN) == LOW) {
                // Motion stopped
                motionDetected = false;
                digitalWrite(STATUS_LED_PIN, LOW);
                
                // Gradually fade out the brightness LED
                for (int brightness = currentBrightness; brightness >= 0; brightness -= 5) {
                    analogWrite(BRIGHTNESS_LED_PIN, brightness);
                    vTaskDelay(pdMS_TO_TICKS(50)); // Smooth fade out
                }
                
                // Send motion stopped data
                MotionData stopData;
                stopData.motionDetected = false;
                stopData.timestamp = millis();
                stopData.duration = 0;
                stopData.brightnessLevel = 0;
                stopData.message = "Motion Stopped";
                
                xQueueSend(motionQueue, &stopData, portMAX_DELAY);
                
                Serial.println("Motion stopped - LED faded out");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms for smooth brightness updates
    }
}

// Task running on Core 1 - Display Management
void displayTask(void *pvParameters) {
    Serial.println("Display Task started on Core: " + String(xPortGetCoreID()));
    
    MotionData receivedData;
    uint32_t lastUpdateTime = 0;
    
    // Initial display
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motion Monitor");
    lcd.setCursor(0, 1);
    lcd.print("Waiting...");
    
    while(1) {
        // Check for new motion data from queue
        if (xQueueReceive(motionQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Clear display and show motion status
            lcd.clear();
            
            if (receivedData.motionDetected) {
                // Motion detected display
                lcd.setCursor(0, 0);
                lcd.print(receivedData.message);
                
                // Show duration and brightness percentage
                lcd.setCursor(0, 1);
                String durationStr = String(receivedData.duration / 1000) + "s ";
                String brightnessStr = String(map(receivedData.brightnessLevel, 0, 255, 0, 100)) + "%";
                lcd.print(durationStr + brightnessStr);
                
                Serial.println("Display updated: " + receivedData.message + " on Core " + String(xPortGetCoreID()));
            } else {
                // Motion stopped display
                lcd.setCursor(0, 0);
                lcd.print("Motion Monitor");
                lcd.setCursor(0, 1);
                lcd.print("Waiting...");
                
                Serial.println("Display updated: Motion stopped on Core " + String(xPortGetCoreID()));
            }
            
            lastUpdateTime = millis();
        }
        
        // Update time display every second when no motion
        if (!receivedData.motionDetected && (millis() - lastUpdateTime > 1000)) {
            lcd.setCursor(0, 1);
            lcd.print("Time: " + String(millis() / 1000) + "s    ");
            lastUpdateTime = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Update every 100ms
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Dual-Core PIR Motion Detection with Progressive LED Brightness");
    
    // Initialize pins
    pinMode(PIR_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(BRIGHTNESS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    
    // Setup PWM for brightness LED using NEW API
    // Set PWM frequency and resolution for the pin
    analogWriteFrequency(BRIGHTNESS_LED_PIN, pwmFreq);
    analogWriteResolution(BRIGHTNESS_LED_PIN, pwmResolution);
    analogWrite(BRIGHTNESS_LED_PIN, 0); // Start with LED off
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("System Starting");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");
    delay(2000);
    
    // Test PWM functionality
    Serial.println("Testing PWM brightness...");
    for (int brightness = 0; brightness <= 255; brightness += 5) {
        analogWrite(BRIGHTNESS_LED_PIN, brightness);
        delay(20);
    }
    for (int brightness = 255; brightness >= 0; brightness -= 5) {
        analogWrite(BRIGHTNESS_LED_PIN, brightness);
        delay(20);
    }
    Serial.println("PWM test complete");
    
    // Create queue for communication between tasks
    motionQueue = xQueueCreate(10, sizeof(MotionData));
    
    if (motionQueue == NULL) {
        Serial.println("Failed to create queue!");
        return;
    }
    
    // Create motion detection task on Core 0
    xTaskCreatePinnedToCore(
        motionDetectionTask,    // Task function
        "MotionTask",           // Name for debugging
        4096,                   // Increased stack size for PWM operations
        NULL,                   // Parameters
        2,                      // Priority
        &motionTaskHandle,      // Task handle
        0                       // Core 0
    );
    
    // Create display task on Core 1
    xTaskCreatePinnedToCore(
        displayTask,            // Task function
        "DisplayTask",          // Name for debugging
        2048,                   // Stack size
        NULL,                   // Parameters
        1,                      // Priority
        &displayTaskHandle,     // Task handle
        1                       // Core 1
    );
    
    // Setup PIR sensor interrupt
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, RISING);
    
    Serial.println("Tasks created successfully!");
    Serial.println("Motion detection and PWM control running on Core 0");
    Serial.println("Display management running on Core 1");
    Serial.println("Progressive brightness: 0-10 seconds = 0-100% brightness");
    
    // Final setup display
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    lcd.setCursor(0, 1);
    lcd.print("Monitoring...");
    delay(1000);
}

void loop() {
    // Main loop runs on Core 1 by default
    // Print system status every 5 seconds
    static uint32_t lastStatusPrint = 0;
    
    if (millis() - lastStatusPrint > 5000) {
        Serial.println("System Status:");
        Serial.println("- Core 0: Motion Detection + PWM Control");
        Serial.println("- Core 1: Display Management");
        Serial.println("- Current Brightness: " + String(currentBrightness) + "/255");
        Serial.println("- Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        lastStatusPrint = millis();
    }
    
    delay(1000);
}
