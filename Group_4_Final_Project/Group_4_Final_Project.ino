/**
 * @file Group_4_Final_Project.ino
 * @brief ESP32 FreeRTOS Child Safety Alert System with Temperature-Based Threshold Adjustment
 * 
 * A sophisticated child presence detection solution for vehicles that combines PIR motion 
 * sensing with environmental monitoring. The system uses Hardware Timer 0 to precisely 
 * control the 6-second counter update cycle and implements temperature-based dynamic 
 * alert threshold adjustment to enhance safety in hot conditions.
 * 
 * @author Tejas Gulur
 * @date June 3, 2025
 * @version 2.1
 */

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <AM2302-Sensor.h>

/** @defgroup pin_config GPIO Pin Configuration
 *  @brief Hardware pin assignments for all system components
 *  @{
 */
#define PIR_PIN 12              ///< PIR motion sensor input pin
#define STATUS_LED_PIN 21       ///< Status LED output pin (motion indicator)
#define ALERT_LED_PIN 42        ///< Alert LED output pin (progressive blinking)
#define STOP_BUTTON_PIN 20      ///< Emergency stop button input pin
#define AM2302_PIN 15           ///< AM2302 temperature/humidity sensor pin
/** @} */

/** @defgroup hardware_objects Hardware Interface Objects
 *  @{
 */
LiquidCrystal_I2C lcd(0x27, 16, 2);        ///< LCD display interface (I2C address 0x27)
AM2302::AM2302_Sensor am2302{AM2302_PIN};  ///< Temperature/humidity sensor interface
/** @} */

/** @defgroup freertos_handles FreeRTOS Task and Queue Handles
 *  @{
 */
TaskHandle_t motionTaskHandle;      ///< Motion detection task handle
TaskHandle_t displayTaskHandle;     ///< Display management task handle
TaskHandle_t alertLedTaskHandle;    ///< Alert LED control task handle
TaskHandle_t temperatureTaskHandle; ///< Temperature monitoring task handle

QueueHandle_t motionQueue;      ///< Motion data communication queue
QueueHandle_t alertQueue;       ///< Alert control communication queue
QueueHandle_t temperatureQueue; ///< Temperature data communication queue
/** @} */

/** @defgroup data_structures System Data Structures
 *  @{
 */

/**
 * @brief Motion detection and system status data
 * 
 * Contains comprehensive information about motion detection state,
 * environmental conditions, and system status for inter-task communication.
 */
typedef struct {
    bool motionDetected;        ///< Current motion detection state
    uint32_t timestamp;         ///< Data timestamp (millis())
    uint8_t persistenceCounter; ///< Motion persistence counter (0-20)
    uint8_t alertLevel;         ///< Current alert level (0-4)
    bool alertSystemStopped;    ///< Emergency stop status
    float temperature;          ///< Current temperature (°C)
    float humidity;             ///< Current humidity (%)
    String message;             ///< Status message for display
} MotionData;

/**
 * @brief Alert LED control data
 * 
 * Controls the progressive alert LED behavior including blink rate
 * and activation state.
 */
typedef struct {
    uint8_t blinkRate;  ///< LED blink rate (0=off, 1-4=increasing speed)
    bool active;        ///< LED activation state
    bool forceUpdate;   ///< Force immediate LED state update
} AlertLedData;

/**
 * @brief Temperature and humidity sensor data
 * 
 * Environmental data from the AM2302 sensor with validity flags.
 */
typedef struct {
    float temperature;  ///< Temperature reading (°C)
    float humidity;     ///< Humidity reading (%)
    bool validReading;  ///< Data validity flag
    uint32_t timestamp; ///< Reading timestamp (millis())
} TemperatureData;
/** @} */

/** @defgroup global_state Global System State Variables
 *  @{
 */
volatile bool pirTriggered = false;         ///< PIR sensor interrupt flag
volatile bool stopButtonPressed = false;   ///< Stop button interrupt flag
uint32_t lastPirTrigger = 0;               ///< Last PIR trigger timestamp
uint8_t motionPersistenceCounter = 0;      ///< Motion persistence counter
bool alertSystemStopped = false;           ///< System stop state

float currentTemperature = 25.0;           ///< Current temperature reading
float currentHumidity = 50.0;              ///< Current humidity reading
bool temperatureDataValid = false;         ///< Temperature data validity
/** @} */

/** @defgroup timer_hardware Hardware Timer Configuration
 *  @brief ESP32 Timer 0 configuration for 6-second counter updates
 *  @{
 */
hw_timer_t *timer0 = NULL;                          ///< Hardware timer handle
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; ///< Timer mutex for ISR safety
volatile bool counterUpdateFlag = false;            ///< Timer ISR flag for counter updates

/**
 * @brief Hardware Timer 0 interrupt service routine
 * 
 * Triggers every 6 seconds to signal the motion detection task to update
 * the persistence counter. Replaces software timing with precise hardware timing.
 * 
 * @note Executes in interrupt context - minimal processing only
 * @note Sets counterUpdateFlag for main task processing
 */
void ARDUINO_ISR_ATTR onTimer0() {
    portENTER_CRITICAL_ISR(&timerMux);
    counterUpdateFlag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}
/** @} */

/** @defgroup system_constants System Configuration Constants
 *  @{
 */
const uint32_t PIR_DETECTION_WINDOW = 5500;     ///< PIR detection window (5.5 seconds)
const uint8_t MAX_PERSISTENCE_COUNTER = 20;     ///< Maximum counter value
const uint8_t MIN_COUNTER_FOR_ALERT = 1;        ///< Minimum counter for alerts

const uint8_t BASE_SLOW_BLINK_COUNTER = 5;      ///< Base slow blink threshold
const uint8_t BASE_MEDIUM_BLINK_COUNTER = 8;    ///< Base medium blink threshold
const uint8_t BASE_FAST_BLINK_COUNTER = 12;     ///< Base fast blink threshold
const uint8_t BASE_CRITICAL_BLINK_COUNTER = 16; ///< Base critical blink threshold

const float COMFORTABLE_TEMP = 22.0;            ///< Comfortable temperature baseline
const float HOT_TEMP_THRESHOLD = 30.0;          ///< Hot temperature threshold
const float CRITICAL_TEMP_THRESHOLD = 35.0;     ///< Critical temperature threshold

uint8_t ALERT_STOP_THRESHOLD = 5;               ///< Emergency stop activation threshold

const uint32_t SLOW_BLINK_INTERVAL = 1000;      ///< Slow blink interval (ms)
const uint32_t MEDIUM_BLINK_INTERVAL = 500;     ///< Medium blink interval (ms)
const uint32_t FAST_BLINK_INTERVAL = 200;       ///< Fast blink interval (ms)
const uint32_t CRITICAL_BLINK_INTERVAL = 100;   ///< Critical blink interval (ms)
/** @} */

/**
 * @brief Calculate temperature-adjusted alert thresholds
 * 
 * Dynamically adjusts alert thresholds based on ambient temperature to make
 * the system more sensitive in hot conditions when danger to children increases.
 * 
 * @param temperature Current ambient temperature in Celsius
 * @param[out] slowThreshold Calculated slow alert threshold
 * @param[out] mediumThreshold Calculated medium alert threshold  
 * @param[out] fastThreshold Calculated fast alert threshold
 * @param[out] criticalThreshold Calculated critical alert threshold
 * 
 * @note Thresholds decrease as temperature increases (higher sensitivity)
 * @note Minimum thresholds enforced to prevent zero values
 */
void calculateTemperatureAdjustedThresholds(float temperature, uint8_t &slowThreshold, uint8_t &mediumThreshold, uint8_t &fastThreshold, uint8_t &criticalThreshold) {
    float adjustmentFactor = 1.0;
    
    if (temperature >= CRITICAL_TEMP_THRESHOLD) {
        adjustmentFactor = 0.3;  // 70% reduction for critical temperatures
    } else if (temperature >= HOT_TEMP_THRESHOLD) {
        adjustmentFactor = 0.5;  // 50% reduction for hot temperatures
    } else if (temperature > COMFORTABLE_TEMP) {
        float tempDiff = temperature - COMFORTABLE_TEMP;
        float maxTempDiff = HOT_TEMP_THRESHOLD - COMFORTABLE_TEMP;
        adjustmentFactor = 1.0 - (tempDiff / maxTempDiff) * 0.5;
    } else {
        adjustmentFactor = 1.0;  // No adjustment for comfortable temperatures
    }
    
    // Calculate adjusted thresholds with minimum value enforcement
    uint8_t calculatedSlow = (uint8_t)(BASE_SLOW_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedMedium = (uint8_t)(BASE_MEDIUM_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedFast = (uint8_t)(BASE_FAST_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedCritical = (uint8_t)(BASE_CRITICAL_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedStop = (uint8_t)(5 * adjustmentFactor);
    
    slowThreshold = (calculatedSlow > 1) ? calculatedSlow : 1;
    mediumThreshold = (calculatedMedium > 2) ? calculatedMedium : 2;
    fastThreshold = (calculatedFast > 3) ? calculatedFast : 3;
    criticalThreshold = (calculatedCritical > 4) ? calculatedCritical : 4;
    
    ALERT_STOP_THRESHOLD = (calculatedStop > 2) ? calculatedStop : 2;
}

/** @defgroup interrupt_handlers Hardware Interrupt Service Routines
 *  @{
 */

/**
 * @brief PIR motion sensor interrupt handler
 * 
 * Triggered on rising edge of PIR sensor output. Sets motion detection
 * flag and records timestamp for processing by motion detection task.
 * 
 * @note Executes in interrupt context - minimal processing only
 */
void ARDUINO_ISR_ATTR motionISR() {
    pirTriggered = true;
    lastPirTrigger = millis();
}

/**
 * @brief Emergency stop button interrupt handler
 * 
 * Triggered when user presses emergency stop button. Only active when
 * motion persistence counter exceeds the alert stop threshold.
 * 
 * @note Executes in interrupt context - minimal processing only
 * @note Button only functional during high alert conditions
 */
void ARDUINO_ISR_ATTR stopButtonISR() {
    if (motionPersistenceCounter > ALERT_STOP_THRESHOLD) {
        stopButtonPressed = true;
    }
}
/** @} */

/**
 * @brief Calculate LED blink rate based on motion persistence and temperature
 * 
 * Determines appropriate LED blink rate by evaluating motion persistence
 * counter against temperature-adjusted thresholds.
 * 
 * @param counter Current motion persistence counter value
 * @param temperature Current ambient temperature for threshold adjustment
 * @return uint8_t Blink rate (0=off, 1=slow, 2=medium, 3=fast, 4=critical)
 */
uint8_t calculateBlinkRate(uint8_t counter, float temperature) {
    uint8_t slowThreshold, mediumThreshold, fastThreshold, criticalThreshold;
    calculateTemperatureAdjustedThresholds(temperature, slowThreshold, mediumThreshold, fastThreshold, criticalThreshold);
    
    if (counter >= criticalThreshold) return 4;
    else if (counter >= fastThreshold) return 3;
    else if (counter >= mediumThreshold) return 2;
    else if (counter >= slowThreshold) return 1;
    else return 0;
}

/**
 * @brief Get LED blink interval for specified rate
 * 
 * Converts blink rate enumeration to corresponding time interval in milliseconds.
 * 
 * @param rate Blink rate (0-4)
 * @return uint32_t Blink interval in milliseconds (0 for off)
 */
uint32_t getBlinkInterval(uint8_t rate) {
    switch(rate) {
        case 1: return SLOW_BLINK_INTERVAL;
        case 2: return MEDIUM_BLINK_INTERVAL;
        case 3: return FAST_BLINK_INTERVAL;
        case 4: return CRITICAL_BLINK_INTERVAL;
        default: return 0;
    }
}

/** @defgroup freertos_tasks FreeRTOS Task Implementations
 *  @{
 */

/**
 * @brief Temperature and humidity monitoring task
 * 
 * Reads AM2302 sensor data every 3 seconds and processes temperature-based
 * threshold calculations at 64Hz. Updates global temperature variables and
 * sends data to other tasks via temperature queue.
 * 
 * @param pvParameters Unused FreeRTOS parameter
 * 
 * @note Runs at 64Hz (15ms period) on Core 0
 * @note Sensor reading limited to 3-second intervals due to hardware constraints
 */
void temperatureTask(void *pvParameters) {
    Serial.println("AM2302 Temperature Task started at 64Hz on Core: " + String(xPortGetCoreID()));
    uint32_t lastSensorRead = 0;
    
    while(1) {
        uint32_t currentTime = millis();
        
        // Read sensor every 3 seconds (hardware limitation)
        if (currentTime - lastSensorRead >= 3000) {
            TemperatureData tempData;
            auto status = am2302.read();
            
            if (status == 0) {
                tempData.temperature = am2302.get_Temperature();
                tempData.humidity = am2302.get_Humidity();
                tempData.validReading = true;
                tempData.timestamp = millis();
                
                currentTemperature = tempData.temperature;
                currentHumidity = tempData.humidity;
                temperatureDataValid = true;
                
                Serial.printf("AM2302: %.1f°C, %.1f%% RH\n", tempData.temperature, tempData.humidity);
                xQueueSend(temperatureQueue, &tempData, 0);
            } else {
                tempData.validReading = false;
                tempData.timestamp = millis();
            }
            lastSensorRead = currentTime;
        }
        
        // Process temperature calculations at 64Hz
        uint8_t slowThresh, medThresh, fastThresh, critThresh;
        calculateTemperatureAdjustedThresholds(currentTemperature, slowThresh, medThresh, fastThresh, critThresh);
        
        vTaskDelay(pdMS_TO_TICKS(15)); // 64Hz frequency
    }
}

/**
 * @brief Motion detection and persistence tracking task
 * 
 * Handles PIR sensor interrupts, maintains motion persistence counter using
 * Hardware Timer 0 for 6-second updates, and manages emergency stop functionality.
 * Implements temperature-adjusted alert level calculation and system state management.
 * 
 * @param pvParameters Unused FreeRTOS parameter
 * 
 * @note Runs at 50Hz (20ms period) on Core 0
 * @note Uses Hardware Timer 0 for precise 6-second counter updates
 * @note Implements counter-based persistence to handle PIR sensor limitations
 */
void motionDetectionTask(void *pvParameters) {
    Serial.println("Timer-Controlled Motion Detection Task started at 50Hz on Core: " + String(xPortGetCoreID()));
    uint8_t lastBlinkRate = 0;
    
    while(1) {
        uint32_t currentTime = millis();
        
        // Handle emergency stop button
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
                xQueueSend(alertQueue, &stopAlert, 0);
            }
        }
        
        // Handle PIR sensor interrupts
        if (pirTriggered && !alertSystemStopped) {
            pirTriggered = false;
            Serial.println("PIR triggered! Counter before: " + String(motionPersistenceCounter));
            
            if (motionPersistenceCounter < MAX_PERSISTENCE_COUNTER) {
                motionPersistenceCounter++;
            }
            Serial.println("PIR triggered! Counter after: " + String(motionPersistenceCounter));
        }
        
        // Hardware Timer 0 controlled 6-second counter updates
        if (counterUpdateFlag) {
            portENTER_CRITICAL(&timerMux);
            counterUpdateFlag = false;
            portEXIT_CRITICAL(&timerMux);
            
            uint32_t timeSinceLastPir = currentTime - lastPirTrigger;
            
            if (timeSinceLastPir >= PIR_DETECTION_WINDOW) {
                if (motionPersistenceCounter > 0) {
                    motionPersistenceCounter--;
                    Serial.println("Timer 6-second cycle: Counter decremented to: " + String(motionPersistenceCounter));
                    
                    if (alertSystemStopped && motionPersistenceCounter == 0) {
                        alertSystemStopped = false;
                        Serial.println("Alert system re-enabled - counter at 0");
                    }
                }
            } else if (!alertSystemStopped) {
                Serial.println("Timer 6-second cycle: PIR active, counter maintained at: " + String(motionPersistenceCounter));
            }
        }
        
        // Calculate current alert state
        uint8_t currentBlinkRate = 0;
        bool motionActive = false;
        
        if (!alertSystemStopped) {
            currentBlinkRate = calculateBlinkRate(motionPersistenceCounter, currentTemperature);
            motionActive = (motionPersistenceCounter >= MIN_COUNTER_FOR_ALERT);
        }
        
        // Control status LED
        if (motionPersistenceCounter > 0 && !alertSystemStopped) {
            digitalWrite(STATUS_LED_PIN, HIGH);
        } else {
            digitalWrite(STATUS_LED_PIN, LOW);
        }
        
        // Prepare motion data for other tasks
        MotionData data;
        data.motionDetected = motionActive;
        data.timestamp = currentTime;
        data.persistenceCounter = motionPersistenceCounter;
        data.alertLevel = currentBlinkRate;
        data.alertSystemStopped = alertSystemStopped;
        data.temperature = currentTemperature;
        data.humidity = currentHumidity;
        
        // Generate status message
        if (alertSystemStopped) {
            data.message = "ALERT STOPPED";
        } else if (motionPersistenceCounter == 0) {
            data.message = "No Motion";
        } else {
            if (currentBlinkRate == 4) data.message = "CRITICAL HOT";
            else if (currentBlinkRate == 3) data.message = "HIGH ALERT";
            else if (currentBlinkRate == 2) data.message = "MEDIUM ALERT";
            else if (currentBlinkRate == 1) data.message = "LOW ALERT";
            else data.message = "Motion Brief";
        }
        
        // Send data to other tasks
        xQueueSend(motionQueue, &data, 0);
        
        AlertLedData alertData;
        alertData.blinkRate = currentBlinkRate;
        alertData.active = motionActive && !alertSystemStopped;
        alertData.forceUpdate = (currentBlinkRate != lastBlinkRate) || alertSystemStopped;
        xQueueSend(alertQueue, &alertData, 0);
        
        lastBlinkRate = currentBlinkRate;
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz frequency
    }
}

/**
 * @brief Progressive alert LED control task
 * 
 * Manages the alert LED blinking patterns based on motion persistence level.
 * Provides visual feedback with different blink rates corresponding to alert severity.
 * 
 * @param pvParameters Unused FreeRTOS parameter
 * 
 * @note Runs at 200Hz (5ms period) on Core 1 for smooth LED control
 * @note Receives commands via alert queue from motion detection task
 */
void alertLedTask(void *pvParameters) {
    Serial.println("Alert LED Task started at 200Hz on Core: " + String(xPortGetCoreID()));
    AlertLedData alertData;
    bool ledState = false;
    uint32_t lastBlinkTime = 0;
    uint32_t currentInterval = 0;
    bool blinkingActive = false;
    
    while(1) {
        if (xQueueReceive(alertQueue, &alertData, pdMS_TO_TICKS(1)) == pdTRUE) {
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
        
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz frequency
    }
}

/**
 * @brief LCD display management task
 * 
 * Updates the LCD display with current system status, motion information,
 * environmental data, and user interface elements. Provides comprehensive
 * system status visibility to users.
 * 
 * @param pvParameters Unused FreeRTOS parameter
 * 
 * @note Runs at 32Hz (31ms period) on Core 1
 * @note Receives data from motion and temperature tasks via queues
 */
void displayTask(void *pvParameters) {
    Serial.println("Display Task started at 32Hz on Core: " + String(xPortGetCoreID()));
    MotionData receivedData;
    TemperatureData tempData;
    uint32_t lastUpdateTime = 0;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Init");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    
    while(1) {
        // Check for temperature data updates
        if (xQueueReceive(temperatureQueue, &tempData, pdMS_TO_TICKS(1)) == pdTRUE) {
            // Temperature data received and processed
        }
        
        // Check for motion data updates
        if (xQueueReceive(motionQueue, &receivedData, pdMS_TO_TICKS(10)) == pdTRUE) {
            lcd.clear();
            
            if (receivedData.alertSystemStopped) {
                lcd.setCursor(0, 0);
                lcd.print("ALERT STOPPED");
                lcd.setCursor(0, 1);
                lcd.print("By User Button");
            } else {
                // Display motion status and temperature
                lcd.setCursor(0, 0);
                String line1 = receivedData.message;
                if (line1.length() > 10) line1 = line1.substring(0, 10);
                line1 += " " + String(receivedData.temperature, 0) + "C";
                lcd.print(line1);
                
                // Display counter and alert level
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
        
        vTaskDelay(pdMS_TO_TICKS(31)); // 32Hz frequency
    }
}
/** @} */

/**
 * @brief System initialization and hardware setup
 * 
 * Initializes all hardware components, creates FreeRTOS tasks and queues,
 * configures Hardware Timer 0 for 6-second counter updates, and sets up
 * interrupt handlers for PIR sensor and emergency stop button.
 * 
 * @note Configures dual-core task distribution for optimal performance
 * @note Sets up Hardware Timer 0 for precise 6-second timing
 */
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 System Init");
    
    // Initialize GPIO pins
    pinMode(PIR_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(ALERT_LED_PIN, OUTPUT);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ALERT_LED_PIN, LOW);
    
    // Initialize sensors
    am2302.begin();
    
    // Initialize LCD display
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("System Init");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");
    delay(2000);

    // Configure Hardware Timer 0 for 6-second counter updates
    timer0 = timerBegin(1000000);  // 1MHz base frequency
    timerAttachInterrupt(timer0, &onTimer0);
    timerAlarm(timer0, 6000000, true, 0);  // 6 seconds, auto-reload
    timerStart(timer0);
    Serial.println("Hardware Timer 0 initialized for 6-second counter updates");

    // Create FreeRTOS queues
    motionQueue = xQueueCreate(10, sizeof(MotionData));
    alertQueue = xQueueCreate(10, sizeof(AlertLedData));
    temperatureQueue = xQueueCreate(5, sizeof(TemperatureData));
    
    if (motionQueue == NULL || alertQueue == NULL || temperatureQueue == NULL) {
        Serial.println("Failed to create queues!");
        return;
    }
    
    // Create tasks with core assignments
    xTaskCreatePinnedToCore(temperatureTask, "TempTask64Hz", 4096, NULL, 2, &temperatureTaskHandle, 0);
    xTaskCreatePinnedToCore(motionDetectionTask, "MotionTask50Hz", 4096, NULL, 2, &motionTaskHandle, 0);
    xTaskCreatePinnedToCore(alertLedTask, "AlertLedTask200Hz", 2048, NULL, 3, &alertLedTaskHandle, 1);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask32Hz", 2048, NULL, 1, &displayTaskHandle, 1);
    
    // Configure hardware interrupts
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, RISING);
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonISR, FALLING);
    
    Serial.println("- Counter Update System Initialized");
    Serial.println("- Hardware Timer 0: Controls 6-second counter updates");
    Serial.println("- Temperature Task: 64Hz");
    Serial.println("- Motion Detection Task: 50Hz");
    Serial.println("- Alert LED Task: 200Hz");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    lcd.setCursor(0, 1);
    lcd.print("Timer0 Mode");
    delay(1000);
}

/**
 * @brief Main system monitoring loop
 * 
 * Provides periodic system status reporting and health monitoring.
 * Runs on Core 1 while FreeRTOS tasks handle all main functionality.
 * 
 * @note Reports system status every 10 seconds
 * @note All primary functionality handled by FreeRTOS tasks
 */
void loop() {
    static uint32_t lastStatusPrint = 0;
    
    if (millis() - lastStatusPrint > 10000) {
        uint8_t slowThresh, medThresh, fastThresh, critThresh;
        calculateTemperatureAdjustedThresholds(currentTemperature, slowThresh, medThresh, fastThresh, critThresh);
        
        Serial.println("=== System Status ===");
        Serial.println("- Information Status");
        Serial.println("- Temperature: " + String(currentTemperature, 1) + "°C");
        Serial.println("- Persistence Counter: " + String(motionPersistenceCounter));
        Serial.println("- Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        lastStatusPrint = millis();
    }
    
    delay(1000);
}
