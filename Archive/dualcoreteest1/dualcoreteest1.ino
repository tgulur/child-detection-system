#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Pin definitions
#define PIR_PIN 12
#define STATUS_LED_PIN 21

// LCD setup (I2C address 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Task handles
TaskHandle_t motionTaskHandle;
TaskHandle_t displayTaskHandle;

// Queue handle for communication between cores
QueueHandle_t motionQueue;

// Data structure for motion data
typedef struct {
    bool motionDetected;
    uint32_t timestamp;
    String message;
} MotionData;

// Global variables
volatile bool motionDetected = false;
uint32_t motionStartTime = 0;
uint32_t motionDuration = 0;

// Interrupt service routine for PIR sensor
void ARDUINO_ISR_ATTR motionISR() {
    motionDetected = true;
    motionStartTime = millis();
}

// Task running on Core 0 - Motion Detection
void motionDetectionTask(void *pvParameters) {
    Serial.println("Motion Detection Task started on Core: " + String(xPortGetCoreID()));
    
    while(1) {
        if (motionDetected) {
            // Calculate motion duration
            motionDuration = millis() - motionStartTime;
            
            // Create motion data structure
            MotionData data;
            data.motionDetected = true;
            data.timestamp = millis();
            data.message = "MOTION DETECTED!";
            
            // Send data to display task via queue
            xQueueSend(motionQueue, &data, portMAX_DELAY);
            
            // Turn on status LED
            digitalWrite(STATUS_LED_PIN, HIGH);
            
            // Print to serial for debugging
            Serial.println("Motion detected! Duration: " + String(motionDuration) + "ms");
            
            // Wait for motion to stop (PIR sensor goes LOW)
            while(digitalRead(PIR_PIN) == HIGH) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Motion stopped
            motionDetected = false;
            digitalWrite(STATUS_LED_PIN, LOW);
            
            // Send motion stopped data
            MotionData stopData;
            stopData.motionDetected = false;
            stopData.timestamp = millis();
            stopData.message = "Motion stopped";
            
            xQueueSend(motionQueue, &stopData, portMAX_DELAY);
            
            Serial.println("Motion stopped");
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Check every 50ms
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
                lcd.print("*** MOTION ***");
                lcd.setCursor(0, 1);
                lcd.print("DETECTED!");
                
                Serial.println("Display updated: Motion detected on Core " + String(xPortGetCoreID()));
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
    Serial.println("ESP32 Dual-Core PIR Motion Detection System");
    
    // Initialize pins
    pinMode(PIR_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("System Starting");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");
    delay(2000);
    
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
        2048,                   // Stack size
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
    Serial.println("Motion detection running on Core 0");
    Serial.println("Display management running on Core 1");
    
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
        Serial.println("System running - Core 0: Motion Detection, Core 1: Display");
        Serial.println("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        lastStatusPrint = millis();
    }
    
    delay(1000);
}
