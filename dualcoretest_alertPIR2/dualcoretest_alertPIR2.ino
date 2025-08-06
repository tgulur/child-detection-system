#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Pin definitions
#define PIR_PIN 12
#define STATUS_LED_PIN 21
#define ALERT_LED_PIN 42

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Task handles
TaskHandle_t motionTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t alertLedTaskHandle;

// Queue handles
QueueHandle_t motionQueue;
QueueHandle_t alertQueue;

// Data structures
typedef struct {
    bool motionDetected;
    uint32_t timestamp;
    uint8_t persistenceCounter;
    uint8_t alertLevel;
    String message;
} MotionData;

typedef struct {
    uint8_t blinkRate;
    bool active;
    bool forceUpdate;
} AlertLedData;

// Counter-based motion tracking variables
volatile bool pirTriggered = false;
uint32_t lastPirTrigger = 0;
uint8_t motionPersistenceCounter = 0;
uint32_t lastCounterUpdate = 0;

// Updated counter system constants for 6-second polling
const uint32_t COUNTER_UPDATE_INTERVAL = 6000;  // Update counter every 6 seconds
const uint32_t PIR_DETECTION_WINDOW = 5500;     // 5.5 second window to detect PIR
const uint8_t MAX_PERSISTENCE_COUNTER = 20;     // Maximum counter value
const uint8_t MIN_COUNTER_FOR_ALERT = 1;        // Minimum counter for alert (changed from 2 to 1)

// Alert thresholds based on counter values
const uint8_t SLOW_BLINK_COUNTER = 3;     // Counter >= 3: Slow blink
const uint8_t MEDIUM_BLINK_COUNTER = 6;   // Counter >= 6: Medium blink  
const uint8_t FAST_BLINK_COUNTER = 10;    // Counter >= 10: Fast blink
const uint8_t CRITICAL_BLINK_COUNTER = 15; // Counter >= 15: Critical blink

// Blink intervals (faster as persistence increases)
const uint32_t SLOW_BLINK_INTERVAL = 1000;     // 1 second
const uint32_t MEDIUM_BLINK_INTERVAL = 500;    // 0.5 second
const uint32_t FAST_BLINK_INTERVAL = 200;      // 0.2 second
const uint32_t CRITICAL_BLINK_INTERVAL = 100;  // 0.1 second

void ARDUINO_ISR_ATTR motionISR() {
    pirTriggered = true;
    lastPirTrigger = millis();
}

uint8_t calculateBlinkRate(uint8_t counter) {
    if (counter >= CRITICAL_BLINK_COUNTER) return 4;
    else if (counter >= FAST_BLINK_COUNTER) return 3;
    else if (counter >= MEDIUM_BLINK_COUNTER) return 2;
    else if (counter >= SLOW_BLINK_COUNTER) return 1;
    else return 0;
}

uint32_t getBlinkInterval(uint8_t rate) {
    switch(rate) {
        case 1: return SLOW_BLINK_INTERVAL;
        case 2: return MEDIUM_BLINK_INTERVAL;
        case 3: return FAST_BLINK_INTERVAL;
        case 4: return CRITICAL_BLINK_INTERVAL;
        default: return 0;
    }
}

// Counter-based motion detection task with persistent status LED control
void motionDetectionTask(void *pvParameters) {
    Serial.println("6-Second Polling Motion Detection Task started on Core: " + String(xPortGetCoreID()));
    
    uint8_t lastBlinkRate = 0;
    lastCounterUpdate = millis();
    
    while(1) {
        uint32_t currentTime = millis();
        
        // Handle PIR interrupt
        if (pirTriggered) {
            pirTriggered = false;
            Serial.println("PIR triggered! Counter before: " + String(motionPersistenceCounter));
            
            // Increment counter on motion detection (with maximum limit)
            if (motionPersistenceCounter < MAX_PERSISTENCE_COUNTER) {
                motionPersistenceCounter++;
            }
            
            Serial.println("PIR triggered! Counter after: " + String(motionPersistenceCounter));
            lastCounterUpdate = currentTime; // Reset counter update timer
        }
        
        // 6-second polling cycle for counter decrement
        if (currentTime - lastCounterUpdate >= COUNTER_UPDATE_INTERVAL) {
            // Check if PIR has been silent during this 6-second interval
            uint32_t timeSinceLastPir = currentTime - lastPirTrigger;
            
            if (timeSinceLastPir >= PIR_DETECTION_WINDOW) {
                // No recent PIR detection in the 6-second window - decrement counter
                if (motionPersistenceCounter > 0) {
                    motionPersistenceCounter--;
                    Serial.println("6-second cycle: Counter decremented to: " + String(motionPersistenceCounter));
                }
            } else {
                Serial.println("6-second cycle: PIR active, counter maintained at: " + String(motionPersistenceCounter));
            }
            
            lastCounterUpdate = currentTime;
        }
        
        // Determine current alert level based on counter
        uint8_t currentBlinkRate = calculateBlinkRate(motionPersistenceCounter);
        bool motionActive = (motionPersistenceCounter >= MIN_COUNTER_FOR_ALERT);
        
        // **PERSISTENT STATUS LED CONTROL**
        // LED on pin 21 stays ON whenever counter > 0 (any motion detected)
        // LED on pin 21 stays OFF when counter = 0 (no motion)
        if (motionPersistenceCounter > 0) {
            digitalWrite(STATUS_LED_PIN, HIGH);  // Keep LED ON for any motion
        } else {
            digitalWrite(STATUS_LED_PIN, LOW);   // Turn LED OFF only when no motion
        }
        
        // Create motion data
        MotionData data;
        data.motionDetected = motionActive;
        data.timestamp = currentTime;
        data.persistenceCounter = motionPersistenceCounter;
        data.alertLevel = currentBlinkRate;
        
        // Generate message based on counter value
        if (motionPersistenceCounter == 0) {
            data.message = "No Motion";
        } else if (motionPersistenceCounter < SLOW_BLINK_COUNTER) {
            data.message = "Motion Brief";
        } else if (motionPersistenceCounter < MEDIUM_BLINK_COUNTER) {
            data.message = "Motion Low";
        } else if (motionPersistenceCounter < FAST_BLINK_COUNTER) {
            data.message = "Motion Medium";
        } else if (motionPersistenceCounter < CRITICAL_BLINK_COUNTER) {
            data.message = "Motion High";
        } else {
            data.message = "Motion CRITICAL";
        }
        
        // Send to display
        xQueueSend(motionQueue, &data, portMAX_DELAY);
        
        // Send to LED controller for blinking LED (pin 42)
        AlertLedData alertData;
        alertData.blinkRate = currentBlinkRate;
        alertData.active = motionActive;
        alertData.forceUpdate = (currentBlinkRate != lastBlinkRate);
        xQueueSend(alertQueue, &alertData, portMAX_DELAY);
        
        lastBlinkRate = currentBlinkRate;
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

// Alert LED task (unchanged - controls pin 42 blinking)
void alertLedTask(void *pvParameters) {
    Serial.println("Alert LED Task started on Core: " + String(xPortGetCoreID()));
    
    AlertLedData alertData;
    bool ledState = false;
    uint32_t lastBlinkTime = 0;
    uint32_t currentInterval = 0;
    bool blinkingActive = false;
    
    while(1) {
        if (xQueueReceive(alertQueue, &alertData, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (alertData.active && alertData.blinkRate > 0) {
                currentInterval = getBlinkInterval(alertData.blinkRate);
                blinkingActive = true;
                
                if (alertData.forceUpdate) {
                    lastBlinkTime = millis();
                    Serial.println("LED: Rate " + String(alertData.blinkRate) + ", Interval: " + String(currentInterval) + "ms");
                }
            } else {
                digitalWrite(ALERT_LED_PIN, LOW);
                ledState = false;
                blinkingActive = false;
            }
        }
        
        if (blinkingActive && currentInterval > 0) {
            uint32_t currentTime = millis();
            if (currentTime - lastBlinkTime >= currentInterval) {
                ledState = !ledState;
                digitalWrite(ALERT_LED_PIN, ledState);
                lastBlinkTime = currentTime;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Display task showing 6-second cycle information
void displayTask(void *pvParameters) {
    Serial.println("Display Task started on Core: " + String(xPortGetCoreID()));
    
    MotionData receivedData;
    uint32_t lastUpdateTime = 0;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("6s Cycle Monitor");
    lcd.setCursor(0, 1);
    lcd.print("Count: 0");
    
    while(1) {
        if (xQueueReceive(motionQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE) {
            lcd.clear();
            
            // Line 1: Motion status message
            lcd.setCursor(0, 0);
            lcd.print(receivedData.message);
            
            // Line 2: Counter value and alert level
            lcd.setCursor(0, 1);
            String counterStr = "C:" + String(receivedData.persistenceCounter);
            String alertStr = " L" + String(receivedData.alertLevel);
            lcd.print(counterStr + alertStr);
            
            lastUpdateTime = millis();
        }
        
        // Show countdown to next 6-second cycle
        if (receivedData.persistenceCounter == 0 && (millis() - lastUpdateTime > 1000)) {
            uint32_t timeToNextCycle = COUNTER_UPDATE_INTERVAL - ((millis() - lastCounterUpdate) % COUNTER_UPDATE_INTERVAL);
            lcd.setCursor(0, 1);
            lcd.print("Next: " + String(timeToNextCycle/1000) + "s     ");
            lastUpdateTime = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 6-Second Polling Motion Detection System with Persistent Status LED");
    
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
    lcd.print("6s Cycle System");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    delay(2000);
    
    // Create queues
    motionQueue = xQueueCreate(10, sizeof(MotionData));
    alertQueue = xQueueCreate(10, sizeof(AlertLedData));
    
    if (motionQueue == NULL || alertQueue == NULL) {
        Serial.println("Failed to create queues!");
        return;
    }
    
    // Create tasks
    xTaskCreatePinnedToCore(motionDetectionTask, "MotionTask", 4096, NULL, 2, &motionTaskHandle, 0);
    xTaskCreatePinnedToCore(alertLedTask, "AlertLedTask", 2048, NULL, 3, &alertLedTaskHandle, 1);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 2048, NULL, 1, &displayTaskHandle, 1);
    
    // Setup PIR interrupt
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, RISING);
    
    Serial.println("6-Second Polling Motion Detection System with Persistent Status LED Initialized");
    Serial.println("- PIR delay: 5 seconds between detections");
    Serial.println("- Counter polling: Every 6 seconds");
    Serial.println("- Detection window: 5.5 seconds");
    Serial.println("- Status LED (pin 21): ON when counter > 0, OFF when counter = 0");
    Serial.println("- Alert LED (pin 42): Blinks based on persistence level");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    lcd.setCursor(0, 1);
    lcd.print("Persistent Mode");
    delay(1000);
}

void loop() {
    static uint32_t lastStatusPrint = 0;
    
    if (millis() - lastStatusPrint > 6000) {
        Serial.println("=== 6-Second Cycle Status ===");
        Serial.println("- Persistence Counter: " + String(motionPersistenceCounter));
        Serial.println("- Status LED (pin 21): " + String(motionPersistenceCounter > 0 ? "ON" : "OFF"));
        Serial.println("- Alert Level: " + String(calculateBlinkRate(motionPersistenceCounter)));
        Serial.println("- Time since last PIR: " + String(millis() - lastPirTrigger) + "ms");
        Serial.println("- Time to next cycle: " + String(COUNTER_UPDATE_INTERVAL - ((millis() - lastCounterUpdate) % COUNTER_UPDATE_INTERVAL)) + "ms");
        Serial.println("- Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        lastStatusPrint = millis();
    }
    
    delay(1000);
}
