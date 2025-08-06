#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Pin definitions
#define PIR_PIN 12
#define STATUS_LED_PIN 21
#define ALERT_LED_PIN 42      // LED that blinks at variable rates

// LCD setup (I2C address 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Task handles
TaskHandle_t motionTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t alertLedTaskHandle;

// Queue handles for communication between cores
QueueHandle_t motionQueue;
QueueHandle_t alertQueue;

// Data structure for motion data
typedef struct {
    bool motionDetected;
    uint32_t timestamp;
    uint32_t duration;
    uint8_t alertLevel;
    String message;
} MotionData;

// Data structure for alert LED control
typedef struct {
    uint8_t blinkRate;      // 0=off, 1=slow, 2=medium, 3=fast, 4=very fast
    uint32_t duration;
    bool active;
    bool forceUpdate;       // Force LED task to update immediately
} AlertLedData;

// Global variables for motion tracking
volatile bool motionInterruptTriggered = false;
uint32_t motionStartTime = 0;
uint32_t lastMotionDetectedTime = 0;
uint32_t motionDuration = 0;
bool motionActive = false;

// PIR sensor timeout handling
const uint32_t PIR_TIMEOUT_THRESHOLD = 3000;  // 3 seconds without PIR trigger = motion stopped
const uint32_t MOTION_EXTENSION_TIME = 2000;  // Extend motion for 2 seconds after PIR timeout

// Motion duration thresholds for different blink rates
const uint32_t SLOW_BLINK_THRESHOLD = 2000;    // 2 seconds
const uint32_t MEDIUM_BLINK_THRESHOLD = 5000;  // 5 seconds
const uint32_t FAST_BLINK_THRESHOLD = 8000;    // 8 seconds
const uint32_t VERY_FAST_BLINK_THRESHOLD = 12000; // 12 seconds

// Blink intervals (in milliseconds)
const uint32_t SLOW_BLINK_INTERVAL = 1000;     // 1 second on/off
const uint32_t MEDIUM_BLINK_INTERVAL = 500;    // 0.5 second on/off
const uint32_t FAST_BLINK_INTERVAL = 200;      // 0.2 second on/off
const uint32_t VERY_FAST_BLINK_INTERVAL = 100; // 0.1 second on/off

// Interrupt service routine for PIR sensor
void ARDUINO_ISR_ATTR motionISR() {
    motionInterruptTriggered = true;
    lastMotionDetectedTime = millis();
}

// Function to determine blink rate based on motion duration
uint8_t calculateBlinkRate(uint32_t duration) {
    if (duration >= VERY_FAST_BLINK_THRESHOLD) {
        return 4; // Very fast blinking
    } else if (duration >= FAST_BLINK_THRESHOLD) {
        return 3; // Fast blinking
    } else if (duration >= MEDIUM_BLINK_THRESHOLD) {
        return 2; // Medium blinking
    } else if (duration >= SLOW_BLINK_THRESHOLD) {
        return 1; // Slow blinking
    } else {
        return 0; // No blinking
    }
}

// Function to get blink interval based on rate
uint32_t getBlinkInterval(uint8_t blinkRate) {
    switch(blinkRate) {
        case 1: return SLOW_BLINK_INTERVAL;
        case 2: return MEDIUM_BLINK_INTERVAL;
        case 3: return FAST_BLINK_INTERVAL;
        case 4: return VERY_FAST_BLINK_INTERVAL;
        default: return 0;
    }
}

// Task running on Core 0 - Enhanced Motion Detection
void motionDetectionTask(void *pvParameters) {
    Serial.println("Enhanced Motion Detection Task started on Core: " + String(xPortGetCoreID()));
    
    uint8_t lastBlinkRate = 0;
    
    while(1) {
        uint32_t currentTime = millis();
        
        // Check for PIR interrupt trigger
        if (motionInterruptTriggered) {
            motionInterruptTriggered = false;
            
            if (!motionActive) {
                // Motion just started
                motionActive = true;
                motionStartTime = currentTime;
                Serial.println("Motion started at: " + String(currentTime));
            }
            
            // Update last detection time
            lastMotionDetectedTime = currentTime;
        }
        
        // Handle ongoing motion detection
        if (motionActive) {
            // Calculate current motion duration
            motionDuration = currentTime - motionStartTime;
            
            // Check if PIR sensor has timed out
            uint32_t timeSinceLastDetection = currentTime - lastMotionDetectedTime;
            
            // If PIR hasn't triggered recently but we're still within extension time, continue motion
            if (timeSinceLastDetection > PIR_TIMEOUT_THRESHOLD) {
                if (timeSinceLastDetection > (PIR_TIMEOUT_THRESHOLD + MOTION_EXTENSION_TIME)) {
                    // Motion has definitely stopped
                    motionActive = false;
                    Serial.println("Motion stopped after timeout. Total duration: " + String(motionDuration) + "ms");
                    
                    // Send motion stopped data
                    MotionData stopData;
                    stopData.motionDetected = false;
                    stopData.timestamp = currentTime;
                    stopData.duration = 0;
                    stopData.alertLevel = 0;
                    stopData.message = "Motion Stopped";
                    xQueueSend(motionQueue, &stopData, portMAX_DELAY);
                    
                    // Stop alert LED
                    AlertLedData stopAlert;
                    stopAlert.blinkRate = 0;
                    stopAlert.duration = 0;
                    stopAlert.active = false;
                    stopAlert.forceUpdate = true;
                    xQueueSend(alertQueue, &stopAlert, portMAX_DELAY);
                    
                    digitalWrite(STATUS_LED_PIN, LOW);
                    continue;
                }
            }
            
            // Calculate current blink rate
            uint8_t currentBlinkRate = calculateBlinkRate(motionDuration);
            
            // Create motion data structure
            MotionData data;
            data.motionDetected = true;
            data.timestamp = currentTime;
            data.duration = motionDuration;
            data.alertLevel = currentBlinkRate;
            
            // Generate appropriate message based on duration
            if (currentBlinkRate == 0) {
                data.message = "Motion Starting";
            } else if (currentBlinkRate == 1) {
                data.message = "Motion Sustained";
            } else if (currentBlinkRate == 2) {
                data.message = "Medium Alert";
            } else if (currentBlinkRate == 3) {
                data.message = "High Alert";
            } else {
                data.message = "CRITICAL ALERT";
            }
            
            // Send data to display task
            xQueueSend(motionQueue, &data, portMAX_DELAY);
            
            // Send alert data to LED task (force update if blink rate changed)
            AlertLedData alertData;
            alertData.blinkRate = currentBlinkRate;
            alertData.duration = motionDuration;
            alertData.active = true;
            alertData.forceUpdate = (currentBlinkRate != lastBlinkRate);
            xQueueSend(alertQueue, &alertData, portMAX_DELAY);
            
            lastBlinkRate = currentBlinkRate;
            
            // Turn on status LED
            digitalWrite(STATUS_LED_PIN, HIGH);
            
            // Debug output every 2 seconds
            if (motionDuration % 2000 < 100) {
                Serial.println("Motion Duration: " + String(motionDuration) + "ms, Blink Rate: " + String(currentBlinkRate));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

// Task running on Core 1 - Enhanced Alert LED Control
void alertLedTask(void *pvParameters) {
    Serial.println("Enhanced Alert LED Task started on Core: " + String(xPortGetCoreID()));
    
    AlertLedData alertData;
    bool ledState = false;
    uint32_t lastBlinkTime = 0;
    uint32_t currentInterval = 0;
    uint8_t currentBlinkRate = 0;
    bool blinkingActive = false;
    
    while(1) {
        // Check for new alert data from queue
        if (xQueueReceive(alertQueue, &alertData, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (alertData.active && alertData.blinkRate > 0) {
                currentInterval = getBlinkInterval(alertData.blinkRate);
                currentBlinkRate = alertData.blinkRate;
                blinkingActive = true;
                
                if (alertData.forceUpdate) {
                    lastBlinkTime = millis(); // Reset blink timing
                    Serial.println("Alert LED: Rate " + String(alertData.blinkRate) + ", Interval: " + String(currentInterval) + "ms");
                }
            } else {
                // Turn off LED when no alert
                digitalWrite(ALERT_LED_PIN, LOW);
                ledState = false;
                currentInterval = 0;
                currentBlinkRate = 0;
                blinkingActive = false;
                Serial.println("Alert LED stopped");
            }
        }
        
        // Handle LED blinking based on current interval
        if (blinkingActive && currentInterval > 0) {
            uint32_t currentTime = millis();
            if (currentTime - lastBlinkTime >= currentInterval) {
                ledState = !ledState;
                digitalWrite(ALERT_LED_PIN, ledState);
                lastBlinkTime = currentTime;
                
                // Debug output for very fast blinking
                if (currentBlinkRate >= 4 && ledState) {
                    Serial.print(".");
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); // Check every 5ms for very responsive blinking
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
                
                // Show duration and alert level
                lcd.setCursor(0, 1);
                String durationStr = String(receivedData.duration / 1000) + "s ";
                String alertStr = "L" + String(receivedData.alertLevel);
                lcd.print(durationStr + alertStr);
            } else {
                // Motion stopped display
                lcd.setCursor(0, 0);
                lcd.print("Motion Monitor");
                lcd.setCursor(0, 1);
                lcd.print("Waiting...");
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
    Serial.println("ESP32 Enhanced Progressive Motion Alert System");
    
    // Initialize pins
    pinMode(PIR_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(ALERT_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ALERT_LED_PIN, LOW);
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("System Starting");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");
    delay(2000);
    
    // Create queues for communication between tasks
    motionQueue = xQueueCreate(10, sizeof(MotionData));
    alertQueue = xQueueCreate(10, sizeof(AlertLedData));
    
    if (motionQueue == NULL || alertQueue == NULL) {
        Serial.println("Failed to create queues!");
        return;
    }
    
    // Create motion detection task on Core 0
    xTaskCreatePinnedToCore(
        motionDetectionTask,    // Task function
        "MotionTask",           // Name for debugging
        4096,                   // Stack size
        NULL,                   // Parameters
        2,                      // Priority
        &motionTaskHandle,      // Task handle
        0                       // Core 0
    );
    
    // Create alert LED task on Core 1
    xTaskCreatePinnedToCore(
        alertLedTask,           // Task function
        "AlertLedTask",         // Name for debugging
        2048,                   // Stack size
        NULL,                   // Parameters
        3,                      // High priority for responsive blinking
        &alertLedTaskHandle,    // Task handle
        1                       // Core 1
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
    
    Serial.println("Enhanced system initialized!");
    Serial.println("- PIR timeout handling: 3s + 2s extension");
    Serial.println("- Improved LED blinking reliability");
    Serial.println("- Enhanced motion duration tracking");
    
    // Final setup display
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    lcd.setCursor(0, 1);
    lcd.print("Enhanced Mode");
    delay(1000);
}

void loop() {
    // Print enhanced system status every 5 seconds
    static uint32_t lastStatusPrint = 0;
    
    if (millis() - lastStatusPrint > 5000) {
        Serial.println("Enhanced System Status:");
        Serial.println("- Motion Active: " + String(motionActive ? "YES" : "NO"));
        if (motionActive) {
            Serial.println("- Motion Duration: " + String(motionDuration) + "ms");
            Serial.println("- Time since last PIR: " + String(millis() - lastMotionDetectedTime) + "ms");
        }
        Serial.println("- Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        lastStatusPrint = millis();
    }
    
    delay(1000);
}
