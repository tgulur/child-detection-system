#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <AM2302-Sensor.h>

// Pin definitions
#define PIR_PIN 12
#define STATUS_LED_PIN 21
#define ALERT_LED_PIN 42
#define STOP_BUTTON_PIN 20
#define AM2302_PIN 15  // GPIO 15 for AM2302 sensor

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// AM2302 sensor setup
AM2302::AM2302_Sensor am2302{AM2302_PIN};

// Task handles
TaskHandle_t motionTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t alertLedTaskHandle;
TaskHandle_t temperatureTaskHandle;

// Queue handles
QueueHandle_t motionQueue;
QueueHandle_t alertQueue;
QueueHandle_t temperatureQueue;

// Data structures
typedef struct {
    bool motionDetected;
    uint32_t timestamp;
    uint8_t persistenceCounter;
    uint8_t alertLevel;
    bool alertSystemStopped;
    float temperature;
    float humidity;
    String message;
} MotionData;

typedef struct {
    uint8_t blinkRate;
    bool active;
    bool forceUpdate;
} AlertLedData;

typedef struct {
    float temperature;
    float humidity;
    bool validReading;
    uint32_t timestamp;
} TemperatureData;

// Global variables
volatile bool pirTriggered = false;
volatile bool stopButtonPressed = false;
uint32_t lastPirTrigger = 0;
uint8_t motionPersistenceCounter = 0;
uint32_t lastCounterUpdate = 0;
bool alertSystemStopped = false;

// Current environmental data
float currentTemperature = 25.0;  // Default temperature
float currentHumidity = 50.0;     // Default humidity
bool temperatureDataValid = false;

// Updated counter system constants
const uint32_t COUNTER_UPDATE_INTERVAL = 6000;  // Update counter every 6 seconds
const uint32_t PIR_DETECTION_WINDOW = 5500;     // 5.5 second window to detect PIR
const uint8_t MAX_PERSISTENCE_COUNTER = 20;     // Maximum counter value
const uint8_t MIN_COUNTER_FOR_ALERT = 1;        // Minimum counter for alert

// **TEMPERATURE-BASED THRESHOLD ADJUSTMENT**
// Base alert thresholds (used at comfortable temperature ~22°C)
const uint8_t BASE_SLOW_BLINK_COUNTER = 5;
const uint8_t BASE_MEDIUM_BLINK_COUNTER = 8;
const uint8_t BASE_FAST_BLINK_COUNTER = 12;
const uint8_t BASE_CRITICAL_BLINK_COUNTER = 16;

// Temperature thresholds for adjustment
const float COMFORTABLE_TEMP = 22.0;    // Baseline temperature
const float HOT_TEMP_THRESHOLD = 30.0;  // Above this, thresholds decrease significantly
const float CRITICAL_TEMP_THRESHOLD = 35.0; // Above this, maximum sensitivity

// Alert stop threshold (also temperature-adjusted)
uint8_t ALERT_STOP_THRESHOLD = 5;

// Blink intervals
const uint32_t SLOW_BLINK_INTERVAL = 1000;
const uint32_t MEDIUM_BLINK_INTERVAL = 500;
const uint32_t FAST_BLINK_INTERVAL = 200;
const uint32_t CRITICAL_BLINK_INTERVAL = 100;

// **FIXED FUNCTION** - Function to calculate temperature-adjusted thresholds
void calculateTemperatureAdjustedThresholds(float temperature, 
                                           uint8_t &slowThreshold, 
                                           uint8_t &mediumThreshold, 
                                           uint8_t &fastThreshold, 
                                           uint8_t &criticalThreshold) {
    
    float adjustmentFactor = 1.0;
    
    if (temperature >= CRITICAL_TEMP_THRESHOLD) {
        // Very hot: Maximum sensitivity (thresholds reduced by 70%)
        adjustmentFactor = 0.3;
    } else if (temperature >= HOT_TEMP_THRESHOLD) {
        // Hot: High sensitivity (thresholds reduced by 50%)
        adjustmentFactor = 0.5;
    } else if (temperature > COMFORTABLE_TEMP) {
        // Warm: Gradual reduction based on temperature
        float tempDiff = temperature - COMFORTABLE_TEMP;
        float maxTempDiff = HOT_TEMP_THRESHOLD - COMFORTABLE_TEMP;
        adjustmentFactor = 1.0 - (tempDiff / maxTempDiff) * 0.5;
    } else {
        // Cool/comfortable: Use base thresholds
        adjustmentFactor = 1.0;
    }
    
    // **FIXED TYPE CASTING** - Calculate adjusted thresholds with proper type casting
    uint8_t calculatedSlow = (uint8_t)(BASE_SLOW_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedMedium = (uint8_t)(BASE_MEDIUM_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedFast = (uint8_t)(BASE_FAST_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedCritical = (uint8_t)(BASE_CRITICAL_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedStop = (uint8_t)(5 * adjustmentFactor);
    
    // Use max with same types (uint8_t)
    slowThreshold = max((uint8_t)1, calculatedSlow);
    mediumThreshold = max((uint8_t)2, calculatedMedium);
    fastThreshold = max((uint8_t)3, calculatedFast);
    criticalThreshold = max((uint8_t)4, calculatedCritical);
    
    // Adjust stop button threshold too
    ALERT_STOP_THRESHOLD = max((uint8_t)2, calculatedStop);
}

// Interrupt service routines
void ARDUINO_ISR_ATTR motionISR() {
    pirTriggered = true;
    lastPirTrigger = millis();
}

void ARDUINO_ISR_ATTR stopButtonISR() {
    if (motionPersistenceCounter > ALERT_STOP_THRESHOLD) {
        stopButtonPressed = true;
    }
}

uint8_t calculateBlinkRate(uint8_t counter, float temperature) {
    uint8_t slowThreshold, mediumThreshold, fastThreshold, criticalThreshold;
    calculateTemperatureAdjustedThresholds(temperature, slowThreshold, mediumThreshold, 
                                         fastThreshold, criticalThreshold);
    
    if (counter >= criticalThreshold) return 4;
    else if (counter >= fastThreshold) return 3;
    else if (counter >= mediumThreshold) return 2;
    else if (counter >= slowThreshold) return 1;
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

// AM2302 Temperature Reading Task (Core 0)
void temperatureTask(void *pvParameters) {
    Serial.println("AM2302 Temperature Task started on Core: " + String(xPortGetCoreID()));
    
    while(1) {
        TemperatureData tempData;
        
        // Read AM2302 sensor
        auto status = am2302.read();
        
        if (status == 0) {  // AM2302_READ_OK
            tempData.temperature = am2302.get_Temperature();
            tempData.humidity = am2302.get_Humidity();
            tempData.validReading = true;
            tempData.timestamp = millis();
            
            // Update global variables
            currentTemperature = tempData.temperature;
            currentHumidity = tempData.humidity;
            temperatureDataValid = true;
            
            Serial.printf("AM2302: %.1f°C, %.1f%% RH\n", tempData.temperature, tempData.humidity);
        } else {
            tempData.validReading = false;
            tempData.timestamp = millis();
            Serial.println("AM2302 read error: " + String(status));
        }
        
        // Send to queue for display
        xQueueSend(temperatureQueue, &tempData, portMAX_DELAY);
        
        // Wait 3 seconds between readings (AM2302 requires minimum 2 seconds)
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// Enhanced motion detection task with temperature-based thresholds
void motionDetectionTask(void *pvParameters) {
    Serial.println("Enhanced Motion Detection Task with Temperature Control started on Core: " + String(xPortGetCoreID()));
    
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
                Serial.println("Temperature: " + String(currentTemperature) + "°C");
                
                motionPersistenceCounter = 0;
                
                AlertLedData stopAlert;
                stopAlert.blinkRate = 0;
                stopAlert.active = false;
                stopAlert.forceUpdate = true;
                xQueueSend(alertQueue, &stopAlert, portMAX_DELAY);
            }
        }
        
        // Handle PIR interrupt
        if (pirTriggered && !alertSystemStopped) {
            pirTriggered = false;
            Serial.println("PIR triggered! Counter before: " + String(motionPersistenceCounter));
            
            if (motionPersistenceCounter < MAX_PERSISTENCE_COUNTER) {
                motionPersistenceCounter++;
            }
            
            Serial.println("PIR triggered! Counter after: " + String(motionPersistenceCounter));
            lastCounterUpdate = currentTime;
        }
        
        // 6-second polling cycle for counter decrement
        if (currentTime - lastCounterUpdate >= COUNTER_UPDATE_INTERVAL) {
            uint32_t timeSinceLastPir = currentTime - lastPirTrigger;
            
            if (timeSinceLastPir >= PIR_DETECTION_WINDOW) {
                if (motionPersistenceCounter > 0) {
                    motionPersistenceCounter--;
                    Serial.println("6-second cycle: Counter decremented to: " + String(motionPersistenceCounter));
                    
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
        
        // Calculate temperature-adjusted alert level
        uint8_t currentBlinkRate = 0;
        bool motionActive = false;
        
        if (!alertSystemStopped) {
            currentBlinkRate = calculateBlinkRate(motionPersistenceCounter, currentTemperature);
            motionActive = (motionPersistenceCounter >= MIN_COUNTER_FOR_ALERT);
        }
        
        // Status LED control
        if (motionPersistenceCounter > 0 && !alertSystemStopped) {
            digitalWrite(STATUS_LED_PIN, HIGH);
        } else {
            digitalWrite(STATUS_LED_PIN, LOW);
        }
        
        // Create motion data
        MotionData data;
        data.motionDetected = motionActive;
        data.timestamp = currentTime;
        data.persistenceCounter = motionPersistenceCounter;
        data.alertLevel = currentBlinkRate;
        data.alertSystemStopped = alertSystemStopped;
        data.temperature = currentTemperature;
        data.humidity = currentHumidity;
        
        // Generate temperature-aware message
        if (alertSystemStopped) {
            data.message = "ALERT STOPPED";
        } else if (motionPersistenceCounter == 0) {
            data.message = "No Motion";
        } else {
            // Get current thresholds for display
            uint8_t slowThresh, medThresh, fastThresh, critThresh;
            calculateTemperatureAdjustedThresholds(currentTemperature, slowThresh, medThresh, fastThresh, critThresh);
            
            if (currentBlinkRate == 4) data.message = "CRITICAL HOT";
            else if (currentBlinkRate == 3) data.message = "HIGH ALERT";
            else if (currentBlinkRate == 2) data.message = "MEDIUM ALERT";
            else if (currentBlinkRate == 1) data.message = "LOW ALERT";
            else data.message = "Motion Brief";
        }
        
        // Send to display
        xQueueSend(motionQueue, &data, portMAX_DELAY);
        
        // Send to LED controller
        AlertLedData alertData;
        alertData.blinkRate = currentBlinkRate;
        alertData.active = motionActive && !alertSystemStopped;
        alertData.forceUpdate = (currentBlinkRate != lastBlinkRate) || alertSystemStopped;
        xQueueSend(alertQueue, &alertData, portMAX_DELAY);
        
        lastBlinkRate = currentBlinkRate;
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Alert LED task (unchanged)
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

// Enhanced display task showing temperature and adjusted thresholds
void displayTask(void *pvParameters) {
    Serial.println("Display Task started on Core: " + String(xPortGetCoreID()));
    
    MotionData receivedData;
    TemperatureData tempData;
    uint32_t lastUpdateTime = 0;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp Alert Sys");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    
    while(1) {
        // Check for temperature data
        if (xQueueReceive(temperatureQueue, &tempData, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Temperature data received, will be used in motion display
        }
        
        // Check for motion data
        if (xQueueReceive(motionQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE) {
            lcd.clear();
            
            if (receivedData.alertSystemStopped) {
                lcd.setCursor(0, 0);
                lcd.print("ALERT STOPPED");
                lcd.setCursor(0, 1);
                lcd.print("By User Button");
            } else {
                // Line 1: Motion status and temperature
                lcd.setCursor(0, 0);
                String line1 = receivedData.message;
                if (line1.length() > 10) line1 = line1.substring(0, 10);
                line1 += " " + String(receivedData.temperature, 0) + "C";
                lcd.print(line1);
                
                // Line 2: Counter, alert level, and button status
                lcd.setCursor(0, 1);
                String counterStr = "C:" + String(receivedData.persistenceCounter);
                String alertStr = " L" + String(receivedData.alertLevel);
                
                if (receivedData.persistenceCounter > ALERT_STOP_THRESHOLD) {
                    lcd.print(counterStr + " BTN!");
                } else {
                    lcd.print(counterStr + alertStr);
                }
            }
            
            lastUpdateTime = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Motion Detection with AM2302 Temperature-Based Alert Adjustment");
    
    // Initialize pins
    pinMode(PIR_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(ALERT_LED_PIN, OUTPUT);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ALERT_LED_PIN, LOW);
    
    // Initialize AM2302 sensor
    am2302.begin();
    Serial.println("AM2302 sensor initialized on GPIO " + String(AM2302_PIN));
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Temp Alert Sys");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    delay(2000);
    
    // Create queues
    motionQueue = xQueueCreate(10, sizeof(MotionData));
    alertQueue = xQueueCreate(10, sizeof(AlertLedData));
    temperatureQueue = xQueueCreate(5, sizeof(TemperatureData));
    
    if (motionQueue == NULL || alertQueue == NULL || temperatureQueue == NULL) {
        Serial.println("Failed to create queues!");
        return;
    }
    
    // Create tasks
    xTaskCreatePinnedToCore(temperatureTask, "TempTask", 4096, NULL, 2, &temperatureTaskHandle, 0);
    xTaskCreatePinnedToCore(motionDetectionTask, "MotionTask", 4096, NULL, 2, &motionTaskHandle, 0);
    xTaskCreatePinnedToCore(alertLedTask, "AlertLedTask", 2048, NULL, 3, &alertLedTaskHandle, 1);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 2048, NULL, 1, &displayTaskHandle, 1);
    
    // Setup interrupts
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, RISING);
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonISR, FALLING);
    
    Serial.println("Temperature-Based Motion Alert System Initialized");
    Serial.println("- AM2302 sensor on GPIO 15");
    Serial.println("- Temperature-adjusted alert thresholds");
    Serial.println("- Higher temperatures = lower thresholds (more sensitive)");
    Serial.println("- Base thresholds at 22°C: 5,8,12,16");
    Serial.println("- Hot (30°C+): 50% reduction");
    Serial.println("- Critical (35°C+): 70% reduction");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    lcd.setCursor(0, 1);
    lcd.print("Temp Control");
    delay(1000);
}

void loop() {
    static uint32_t lastStatusPrint = 0;
    
    if (millis() - lastStatusPrint > 10000) {
        uint8_t slowThresh, medThresh, fastThresh, critThresh;
        calculateTemperatureAdjustedThresholds(currentTemperature, slowThresh, medThresh, fastThresh, critThresh);
        
        Serial.println("=== Temperature-Based Alert System Status ===");
        Serial.println("- Temperature: " + String(currentTemperature, 1) + "°C");
        Serial.println("- Humidity: " + String(currentHumidity, 1) + "%");
        Serial.println("- Adjusted Thresholds: " + String(slowThresh) + "," + String(medThresh) + "," + String(fastThresh) + "," + String(critThresh));
        Serial.println("- Persistence Counter: " + String(motionPersistenceCounter));
        Serial.println("- Alert System Stopped: " + String(alertSystemStopped ? "YES" : "NO"));
        Serial.println("- Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        lastStatusPrint = millis();
    }
    
    delay(1000);
}
