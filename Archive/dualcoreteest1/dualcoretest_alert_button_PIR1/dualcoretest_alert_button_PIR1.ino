#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Pin definitions
#define PIR_PIN 12
#define STATUS_LED_PIN 21
#define ALERT_LED_PIN 42
#define STOP_BUTTON_PIN 20

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
    bool alertSystemStopped;
    String message;
} MotionData;

typedef struct {
    uint8_t blinkRate;
    bool active;
    bool forceUpdate;
} AlertLedData;

// Counter-based motion tracking variables
volatile bool pirTriggered = false;
volatile bool stopButtonPressed = false;
uint32_t lastPirTrigger = 0;
uint8_t motionPersistenceCounter = 0;
uint32_t lastCounterUpdate = 0;
bool alertSystemStopped = false;

// Updated counter system constants for 6-second polling
const uint32_t COUNTER_UPDATE_INTERVAL = 6000;  // Update counter every 6 seconds
const uint32_t PIR_DETECTION_WINDOW = 5500;     // 5.5 second window to detect PIR
const uint8_t MAX_PERSISTENCE_COUNTER = 20;     // Maximum counter value
const uint8_t MIN_COUNTER_FOR_ALERT = 1;        // Minimum counter for alert
const uint8_t ALERT_STOP_THRESHOLD = 5;         // Counter threshold to enable stop button

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

// Interrupt service routines
void ARDUINO_ISR_ATTR motionISR() {
    pirTriggered = true;
    lastPirTrigger = millis();
}

void ARDUINO_ISR_ATTR stopButtonISR() {
    // Only allow button interrupt if counter is above threshold
    if (motionPersistenceCounter > ALERT_STOP_THRESHOLD) {
        stopButtonPressed = true;
    }
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

// Enhanced motion detection task with button interrupt handling
void motionDetectionTask(void *pvParameters) {
    Serial.println("Enhanced Motion Detection Task with Button Interrupt started on Core: " + String(xPortGetCoreID()));
    
    uint8_t lastBlinkRate = 0;
    lastCounterUpdate = millis();
    
    while(1) {
        uint32_t currentTime = millis();
        
        // Handle stop button interrupt
        if (stopButtonPressed) {
            stopButtonPressed = false;
            
            if (motionPersistenceCounter > ALERT_STOP_THRESHOLD) {
                alertSystemStopped = true;
                Serial.println("*** ALERT SYSTEM STOPPED BY USER ***");
                Serial.println("Counter was: " + String(motionPersistenceCounter));
                
                // Reset counter to safe level
                motionPersistenceCounter = 0;
                
                // Send immediate update to stop alerts
                AlertLedData stopAlert;
                stopAlert.blinkRate = 0;
                stopAlert.active = false;
                stopAlert.forceUpdate = true;
                xQueueSend(alertQueue, &stopAlert, portMAX_DELAY);
            }
        }
        
        // Handle PIR interrupt (only if alert system not stopped)
        if (pirTriggered && !alertSystemStopped) {
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
                    
                    // Re-enable alert system if counter drops to safe level
                    if (alertSystemStopped && motionPersistenceCounter == 0) {
                        alertSystemStopped = false;
                        Serial.println("Alert system re-enabled - counter at 0");
                    }
                }
            } else if (!alertSystemStopped) {
                Serial.println("6-second cycle: PIR active, counter maintained at: " + String(motionPersistenceCounter));
            }
            
            lastCounterUpdate = currentTime;
        }
        
        // Determine current alert level based on counter (only if system not stopped)
        uint8_t currentBlinkRate = 0;
        bool motionActive = false;
        
        if (!alertSystemStopped) {
            currentBlinkRate = calculateBlinkRate(motionPersistenceCounter);
            motionActive = (motionPersistenceCounter >= MIN_COUNTER_FOR_ALERT);
        }
        
        // **PERSISTENT STATUS LED CONTROL**
        // LED on pin 21 stays ON whenever counter > 0 (any motion detected)
        // LED on pin 21 stays OFF when counter = 0 (no motion) or system stopped
        if (motionPersistenceCounter > 0 && !alertSystemStopped) {
            digitalWrite(STATUS_LED_PIN, HIGH);  // Keep LED ON for any motion
        } else {
            digitalWrite(STATUS_LED_PIN, LOW);   // Turn LED OFF when no motion or system stopped
        }
        
        // Create motion data
        MotionData data;
        data.motionDetected = motionActive;
        data.timestamp = currentTime;
        data.persistenceCounter = motionPersistenceCounter;
        data.alertLevel = currentBlinkRate;
        data.alertSystemStopped = alertSystemStopped;
        
        // Generate message based on system state and counter value
        if (alertSystemStopped) {
            data.message = "ALERT STOPPED";
        } else if (motionPersistenceCounter == 0) {
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
        alertData.active = motionActive && !alertSystemStopped;
        alertData.forceUpdate = (currentBlinkRate != lastBlinkRate) || alertSystemStopped;
        xQueueSend(alertQueue, &alertData, portMAX_DELAY);
        
        lastBlinkRate = currentBlinkRate;
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

// Alert LED task (enhanced to handle system stop)
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
                // Turn off LED when no alert or system stopped
                digitalWrite(ALERT_LED_PIN, LOW);
                ledState = false;
                blinkingActive = false;
                if (alertData.forceUpdate) {
                    Serial.println("Alert LED stopped");
                }
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

// Enhanced display task showing button functionality
void displayTask(void *pvParameters) {
    Serial.println("Display Task started on Core: " + String(xPortGetCoreID()));
    
    MotionData receivedData;
    uint32_t lastUpdateTime = 0;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motion + Button");
    lcd.setCursor(0, 1);
    lcd.print("Count: 0");
    
    while(1) {
        if (xQueueReceive(motionQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE) {
            lcd.clear();
            
            if (receivedData.alertSystemStopped) {
                // Special display for stopped system
                lcd.setCursor(0, 0);
                lcd.print("ALERT STOPPED");
                lcd.setCursor(0, 1);
                lcd.print("By User Button");
            } else {
                // Normal motion display
                lcd.setCursor(0, 0);
                lcd.print(receivedData.message);
                
                // Line 2: Counter value, alert level, and button status
                lcd.setCursor(0, 1);
                String counterStr = "C:" + String(receivedData.persistenceCounter);
                String alertStr = " L" + String(receivedData.alertLevel);
                
                // Add button indicator if counter > 5
                if (receivedData.persistenceCounter > ALERT_STOP_THRESHOLD) {
                    lcd.print(counterStr + " BTN!");
                } else {
                    lcd.print(counterStr + alertStr);
                }
            }
            
            lastUpdateTime = millis();
        }
        
        // Show countdown to next 6-second cycle when no motion and system active
        if (!receivedData.alertSystemStopped && receivedData.persistenceCounter == 0 && (millis() - lastUpdateTime > 1000)) {
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
    Serial.println("ESP32 Motion Detection with Button Interrupt Stop System");
    
    // Initialize pins
    pinMode(PIR_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(ALERT_LED_PIN, OUTPUT);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);  // Button with internal pull-up
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ALERT_LED_PIN, LOW);
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Button Interrupt");
    lcd.setCursor(0, 1);
    lcd.print("System Starting");
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
    
    // Setup interrupts
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, RISING);
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonISR, FALLING);
    
    Serial.println("Motion Detection with Button Interrupt System Initialized");
    Serial.println("- PIR delay: 5 seconds between detections");
    Serial.println("- Counter polling: Every 6 seconds");
    Serial.println("- Stop button active when counter > 5");
    Serial.println("- Button press stops alert system and resets counter");
    Serial.println("- Status LED (pin 21): ON when counter > 0, OFF when counter = 0 or stopped");
    Serial.println("- Alert LED (pin 42): Blinks based on persistence level");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    lcd.setCursor(0, 1);
    lcd.print("Button Mode");
    delay(1000);
}

void loop() {
    static uint32_t lastStatusPrint = 0;
    
    if (millis() - lastStatusPrint > 6000) {
        Serial.println("=== Button Interrupt System Status ===");
        Serial.println("- Persistence Counter: " + String(motionPersistenceCounter));
        Serial.println("- Alert System Stopped: " + String(alertSystemStopped ? "YES" : "NO"));
        Serial.println("- Button Active: " + String(motionPersistenceCounter > ALERT_STOP_THRESHOLD ? "YES" : "NO"));
        Serial.println("- Status LED (pin 21): " + String((motionPersistenceCounter > 0 && !alertSystemStopped) ? "ON" : "OFF"));
        Serial.println("- Alert Level: " + String(calculateBlinkRate(motionPersistenceCounter)));
        Serial.println("- Time since last PIR: " + String(millis() - lastPirTrigger) + "ms");
        Serial.println("- Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        lastStatusPrint = millis();
    }
    
    delay(1000);
}
