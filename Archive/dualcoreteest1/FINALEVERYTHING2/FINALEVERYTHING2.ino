/**
 * @file child_safety_alert_system.cpp
 * @brief ESP32 FreeRTOS Child Safety Alert System with Temperature-Based Threshold Adjustment
 * 
 * This system implements a sophisticated child presence detection solution for vehicles
 * that combines PIR motion sensing with environmental monitoring to create a comprehensive
 * safety alert system. The system uses temperature data to dynamically adjust alert
 * thresholds, making it more sensitive in hot conditions when the danger to children
 * is greatest.
 * 
 * Key Features:
 * - Dual-core ESP32 implementation with FreeRTOS task management
 * - Counter-based persistent motion detection with PIR sensor timeout handling
 * - Temperature-based dynamic alert threshold adjustment
 * - Progressive LED alert system with multiple blink rates
 * - User-controllable emergency stop functionality
 * - Multi-queue data flow architecture
 * - High-frequency task execution (50Hz, 64Hz, 200Hz)
 * 
 * @author Tejas Gulur
 * @date May 29, 2025
 * @version 1.0
 */

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <AM2302-Sensor.h>

/** @defgroup pin_definitions Pin Definitions
 *  @brief GPIO pin assignments for all system components
 *  @{
 */
#define PIR_PIN 12              ///< GPIO pin for PIR motion sensor input
#define STATUS_LED_PIN 21       ///< GPIO pin for status LED (on when motion detected)
#define ALERT_LED_PIN 42        ///< GPIO pin for progressive alert LED (variable blink rate)
#define STOP_BUTTON_PIN 20      ///< GPIO pin for emergency stop button input
#define AM2302_PIN 15           ///< GPIO pin for AM2302 temperature/humidity sensor
/** @} */

/** @defgroup hardware_objects Hardware Interface Objects
 *  @brief Initialized hardware interface objects
 *  @{
 */
LiquidCrystal_I2C lcd(0x27, 16, 2);        ///< LCD display object (I2C address 0x27, 16x2 characters)
AM2302::AM2302_Sensor am2302{AM2302_PIN};  ///< AM2302 temperature/humidity sensor object
/** @} */

/** @defgroup task_handles FreeRTOS Task Handles
 *  @brief Task handles for FreeRTOS task management
 *  @{
 */
TaskHandle_t motionTaskHandle;      ///< Handle for motion detection task (50Hz, Core 0)
TaskHandle_t displayTaskHandle;     ///< Handle for display management task (32Hz, Core 1)
TaskHandle_t alertLedTaskHandle;    ///< Handle for alert LED control task (200Hz, Core 1)
TaskHandle_t temperatureTaskHandle; ///< Handle for temperature monitoring task (64Hz, Core 0)
/** @} */

/** @defgroup queue_handles FreeRTOS Queue Handles
 *  @brief Queue handles for inter-task communication
 *  @{
 */
QueueHandle_t motionQueue;      ///< Queue for motion detection data transfer
QueueHandle_t alertQueue;       ///< Queue for alert LED control commands
QueueHandle_t temperatureQueue; ///< Queue for temperature sensor data transfer
/** @} */

/** @defgroup data_structures Data Structures
 *  @brief Data structures for inter-task communication
 *  @{
 */

/**
 * @struct MotionData
 * @brief Comprehensive motion detection and system status data structure
 * 
 * This structure contains all relevant information about the current motion
 * detection state, environmental conditions, and system status. It is passed
 * between tasks to maintain system-wide awareness of current conditions.
 */
typedef struct {
    bool motionDetected;        ///< True if motion is currently active (counter >= MIN_COUNTER_FOR_ALERT)
    uint32_t timestamp;         ///< Timestamp when this data was created (millis())
    uint8_t persistenceCounter; ///< Current motion persistence counter value (0-20)
    uint8_t alertLevel;         ///< Current alert level (0=none, 1=low, 2=medium, 3=high, 4=critical)
    bool alertSystemStopped;    ///< True if user has stopped the alert system via button
    float temperature;          ///< Current temperature reading in Celsius
    float humidity;             ///< Current humidity reading in percentage
    String message;             ///< Human-readable status message for display
} MotionData;

/**
 * @struct AlertLedData
 * @brief Alert LED control data structure
 * 
 * This structure controls the behavior of the progressive alert LED,
 * including blink rate, activation state, and update flags.
 */
typedef struct {
    uint8_t blinkRate;  ///< LED blink rate (0=off, 1=slow, 2=medium, 3=fast, 4=critical)
    bool active;        ///< True if LED should be actively blinking
    bool forceUpdate;   ///< True to force immediate LED state update
} AlertLedData;

/**
 * @struct TemperatureData
 * @brief Temperature and humidity sensor data structure
 * 
 * Contains environmental data from the AM2302 sensor along with
 * validity flags and timing information.
 */
typedef struct {
    float temperature;  ///< Temperature reading in Celsius
    float humidity;     ///< Humidity reading in percentage
    bool validReading;  ///< True if sensor reading was successful
    uint32_t timestamp; ///< Timestamp of sensor reading (millis())
} TemperatureData;
/** @} */

/** @defgroup global_variables Global Variables
 *  @brief Global system state variables
 *  @{
 */
volatile bool pirTriggered = false;         ///< ISR flag: PIR sensor triggered
volatile bool stopButtonPressed = false;   ///< ISR flag: Emergency stop button pressed
uint32_t lastPirTrigger = 0;               ///< Timestamp of last PIR trigger (millis())
uint8_t motionPersistenceCounter = 0;      ///< Current motion persistence counter (0-20)
uint32_t lastCounterUpdate = 0;            ///< Timestamp of last counter update (millis())
bool alertSystemStopped = false;           ///< True if alert system stopped by user

float currentTemperature = 25.0;           ///< Current temperature reading (default 25°C)
float currentHumidity = 50.0;              ///< Current humidity reading (default 50%)
bool temperatureDataValid = false;         ///< True if temperature data is valid
/** @} */

/** @defgroup timing_constants Timing Constants
 *  @brief System timing configuration constants
 *  @{
 */
const uint32_t COUNTER_UPDATE_INTERVAL = 6000;  ///< Counter update interval in milliseconds (6 seconds)
const uint32_t PIR_DETECTION_WINDOW = 5500;     ///< PIR detection window in milliseconds (5.5 seconds)
const uint8_t MAX_PERSISTENCE_COUNTER = 20;     ///< Maximum value for persistence counter
const uint8_t MIN_COUNTER_FOR_ALERT = 1;        ///< Minimum counter value to trigger alerts
/** @} */

/** @defgroup temperature_thresholds Temperature-Based Threshold Configuration
 *  @brief Constants for temperature-based alert threshold adjustment
 *  @{
 */
const uint8_t BASE_SLOW_BLINK_COUNTER = 5;      ///< Base threshold for slow blink alert (at 22°C)
const uint8_t BASE_MEDIUM_BLINK_COUNTER = 8;    ///< Base threshold for medium blink alert (at 22°C)
const uint8_t BASE_FAST_BLINK_COUNTER = 12;     ///< Base threshold for fast blink alert (at 22°C)
const uint8_t BASE_CRITICAL_BLINK_COUNTER = 16; ///< Base threshold for critical blink alert (at 22°C)

const float COMFORTABLE_TEMP = 22.0;            ///< Baseline comfortable temperature in Celsius
const float HOT_TEMP_THRESHOLD = 30.0;          ///< Temperature above which thresholds decrease significantly
const float CRITICAL_TEMP_THRESHOLD = 35.0;     ///< Temperature above which maximum sensitivity is applied

uint8_t ALERT_STOP_THRESHOLD = 5;               ///< Counter threshold to enable emergency stop button
/** @} */

/** @defgroup led_timing LED Timing Constants
 *  @brief Timing constants for LED blink patterns
 *  @{
 */
const uint32_t SLOW_BLINK_INTERVAL = 1000;      ///< Slow blink interval in milliseconds
const uint32_t MEDIUM_BLINK_INTERVAL = 500;     ///< Medium blink interval in milliseconds
const uint32_t FAST_BLINK_INTERVAL = 200;       ///< Fast blink interval in milliseconds
const uint32_t CRITICAL_BLINK_INTERVAL = 100;   ///< Critical blink interval in milliseconds
/** @} */

/**
 * @brief Calculate temperature-adjusted alert thresholds
 * 
 * This function dynamically adjusts alert thresholds based on current temperature
 * to make the system more sensitive in hot conditions. Higher temperatures result
 * in lower thresholds, meaning alerts trigger with less sustained motion.
 * 
 * Temperature adjustment factors:
 * - Below 22°C: Use base thresholds (no adjustment)
 * - 22-30°C: Gradual reduction up to 50%
 * - 30-35°C: 50% reduction in thresholds
 * - Above 35°C: 70% reduction in thresholds (maximum sensitivity)
 * 
 * @param temperature Current temperature in Celsius
 * @param[out] slowThreshold Calculated threshold for slow blink alert
 * @param[out] mediumThreshold Calculated threshold for medium blink alert
 * @param[out] fastThreshold Calculated threshold for fast blink alert
 * @param[out] criticalThreshold Calculated threshold for critical blink alert
 * 
 * @note All output thresholds are guaranteed to be at least 1, 2, 3, and 4 respectively
 * @note Also updates the global ALERT_STOP_THRESHOLD variable
 */
void calculateTemperatureAdjustedThresholds(float temperature, 
                                           uint8_t &slowThreshold, 
                                           uint8_t &mediumThreshold, 
                                           uint8_t &fastThreshold, 
                                           uint8_t &criticalThreshold) {
    
    float adjustmentFactor = 1.0;
    
    if (temperature >= CRITICAL_TEMP_THRESHOLD) {
        adjustmentFactor = 0.3;  // 70% reduction for critical temperatures
    } else if (temperature >= HOT_TEMP_THRESHOLD) {
        adjustmentFactor = 0.5;  // 50% reduction for hot temperatures
    } else if (temperature > COMFORTABLE_TEMP) {
        // Gradual reduction between comfortable and hot temperatures
        float tempDiff = temperature - COMFORTABLE_TEMP;
        float maxTempDiff = HOT_TEMP_THRESHOLD - COMFORTABLE_TEMP;
        adjustmentFactor = 1.0 - (tempDiff / maxTempDiff) * 0.5;
    } else {
        adjustmentFactor = 1.0;  // No adjustment for comfortable temperatures
    }
    
    // Calculate adjusted thresholds with proper type casting
    uint8_t calculatedSlow = (uint8_t)(BASE_SLOW_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedMedium = (uint8_t)(BASE_MEDIUM_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedFast = (uint8_t)(BASE_FAST_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedCritical = (uint8_t)(BASE_CRITICAL_BLINK_COUNTER * adjustmentFactor);
    uint8_t calculatedStop = (uint8_t)(5 * adjustmentFactor);
    
    // Apply minimum thresholds to prevent zero values
    slowThreshold = (calculatedSlow > 1) ? calculatedSlow : 1;
    mediumThreshold = (calculatedMedium > 2) ? calculatedMedium : 2;
    fastThreshold = (calculatedFast > 3) ? calculatedFast : 3;
    criticalThreshold = (calculatedCritical > 4) ? calculatedCritical : 4;
    
    // Update global stop button threshold
    ALERT_STOP_THRESHOLD = (calculatedStop > 2) ? calculatedStop : 2;
}

/** @defgroup interrupt_handlers Interrupt Service Routines
 *  @brief Hardware interrupt handlers for sensors and user input
 *  @{
 */

/**
 * @brief PIR motion sensor interrupt service routine
 * 
 * This ISR is triggered on the rising edge of the PIR sensor output.
 * It sets flags and timestamps for processing by the motion detection task.
 * 
 * @note This function runs in interrupt context - keep it minimal and fast
 * @note Uses ARDUINO_ISR_ATTR for proper interrupt handling on ESP32
 */
void ARDUINO_ISR_ATTR motionISR() {
    pirTriggered = true;
    lastPirTrigger = millis();
}

/**
 * @brief Emergency stop button interrupt service routine
 * 
 * This ISR is triggered when the user presses the emergency stop button.
 * The button is only active when the motion persistence counter exceeds
 * the ALERT_STOP_THRESHOLD to prevent accidental activation.
 * 
 * @note This function runs in interrupt context - keep it minimal and fast
 * @note Button is only functional when motion counter > ALERT_STOP_THRESHOLD
 */
void ARDUINO_ISR_ATTR stopButtonISR() {
    if (motionPersistenceCounter > ALERT_STOP_THRESHOLD) {
        stopButtonPressed = true;
    }
}
/** @} */

/**
 * @brief Calculate LED blink rate based on motion counter and temperature
 * 
 * Determines the appropriate LED blink rate by first calculating temperature-adjusted
 * thresholds and then comparing the current motion persistence counter against them.
 * 
 * @param counter Current motion persistence counter value
 * @param temperature Current temperature for threshold adjustment
 * @return uint8_t Blink rate (0=off, 1=slow, 2=medium, 3=fast, 4=critical)
 * 
 * @see calculateTemperatureAdjustedThresholds()
 */
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

/**
 * @brief Get LED blink interval for specified blink rate
 * 
 * Converts a blink rate number into the corresponding time interval
 * for LED on/off cycles.
 * 
 * @param rate Blink rate (0-4)
 * @return uint32_t Blink interval in milliseconds (0 if rate is 0)
 */
uint32_t getBlinkInterval(uint8_t rate) {
    switch(rate) {
        case 1: return SLOW_BLINK_INTERVAL;     // 1000ms
        case 2: return MEDIUM_BLINK_INTERVAL;   // 500ms
        case 3: return FAST_BLINK_INTERVAL;     // 200ms
        case 4: return CRITICAL_BLINK_INTERVAL; // 100ms
        default: return 0;                      // LED off
    }
}

/** @defgroup freertos_tasks FreeRTOS Task Functions
 *  @brief Main task functions for the FreeRTOS system
 *  @{
 */

/**
 * @brief AM2302 temperature and humidity monitoring task (64Hz, Core 0)
 * 
 * This task reads the AM2302 sensor every 3 seconds (sensor limitation) but
 * processes temperature-based threshold calculations at 64Hz to maintain
 * high-frequency system responsiveness. The task updates global temperature
 * and humidity variables and sends data to other tasks via queues.
 * 
 * Task Specifications:
 * - Frequency: 64Hz (15.6ms period)
 * - Core: 0 (sensor processing core)
 * - Priority: 2
 * - Stack: 4096 bytes
 * 
 * @param pvParameters Unused FreeRTOS parameter (NULL)
 * 
 * @note Sensor reading limited to every 3 seconds due to AM2302 hardware constraints
 * @note Threshold calculations performed at full 64Hz for system responsiveness
 */
void temperatureTask(void *pvParameters) {
    Serial.println("AM2302 Temperature Task started at 64Hz on Core: " + String(xPortGetCoreID()));
    
    uint32_t lastSensorRead = 0;
    
    while(1) {
        uint32_t currentTime = millis();
        
        // Read AM2302 sensor every 3 seconds (sensor limitation)
        if (currentTime - lastSensorRead >= 3000) {
            TemperatureData tempData;
            
            auto status = am2302.read();
            
            if (status == 0) {  // AM2302_READ_OK
                tempData.temperature = am2302.get_Temperature();
                tempData.humidity = am2302.get_Humidity();
                tempData.validReading = true;
                tempData.timestamp = millis();
                
                // Update global variables for other tasks
                currentTemperature = tempData.temperature;
                currentHumidity = tempData.humidity;
                temperatureDataValid = true;
                
                Serial.printf("AM2302: %.1f°C, %.1f%% RH\n", tempData.temperature, tempData.humidity);
                
                // Send to display task
                xQueueSend(temperatureQueue, &tempData, 0);
            } else {
                tempData.validReading = false;
                tempData.timestamp = millis();
                Serial.println("AM2302 read error: " + String(status));
            }
            
            lastSensorRead = currentTime;
        }
        
        // Process temperature-based threshold calculations at 64Hz
        uint8_t slowThresh, medThresh, fastThresh, critThresh;
        calculateTemperatureAdjustedThresholds(currentTemperature, slowThresh, medThresh, fastThresh, critThresh);
        
        // 64Hz frequency - task runs every 15.625ms
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

/**
 * @brief Enhanced motion detection and processing task (50Hz, Core 0)
 * 
 * This is the core task that handles PIR motion detection, maintains the motion
 * persistence counter, processes emergency stop button input, and coordinates
 * the overall system response. It implements a sophisticated counter-based
 * approach to handle PIR sensor timeout limitations.
 * 
 * Key Responsibilities:
 * - Process PIR sensor interrupts and maintain motion persistence counter
 * - Handle emergency stop button functionality
 * - Implement 6-second polling cycle for counter decrement
 * - Calculate temperature-adjusted alert levels
 * - Control status LED based on motion state
 * - Send motion data to display and alert tasks
 * 
 * Task Specifications:
 * - Frequency: 50Hz (20ms period)
 * - Core: 0 (sensor processing core)
 * - Priority: 2
 * - Stack: 4096 bytes
 * 
 * @param pvParameters Unused FreeRTOS parameter (NULL)
 * 
 * @note Implements counter-based persistence to handle PIR sensor timeouts
 * @note Emergency stop only functional when counter > ALERT_STOP_THRESHOLD
 */
void motionDetectionTask(void *pvParameters) {
    Serial.println("Enhanced Motion Detection Task started at 50Hz on Core: " + String(xPortGetCoreID()));
    
    uint8_t lastBlinkRate = 0;
    lastCounterUpdate = millis();
    
    while(1) {
        uint32_t currentTime = millis();
        
        // Handle emergency stop button interrupt
        if (stopButtonPressed) {
            stopButtonPressed = false;
            
            if (motionPersistenceCounter > ALERT_STOP_THRESHOLD) {
                alertSystemStopped = true;
                Serial.println("*** ALERT SYSTEM STOPPED BY USER ***");
                Serial.println("Counter was: " + String(motionPersistenceCounter));
                Serial.println("Temperature: " + String(currentTemperature) + "°C");
                
                motionPersistenceCounter = 0;
                
                // Send immediate stop command to alert LED
                AlertLedData stopAlert;
                stopAlert.blinkRate = 0;
                stopAlert.active = false;
                stopAlert.forceUpdate = true;
                xQueueSend(alertQueue, &stopAlert, 0);
            }
        }
        
        // Handle PIR sensor interrupt
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
                    
                    // Re-enable alert system if counter reaches zero
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
        
        // Control status LED (on when any motion detected, off when stopped or no motion)
        if (motionPersistenceCounter > 0 && !alertSystemStopped) {
            digitalWrite(STATUS_LED_PIN, HIGH);
        } else {
            digitalWrite(STATUS_LED_PIN, LOW);
        }
        
        // Create comprehensive motion data structure
        MotionData data;
        data.motionDetected = motionActive;
        data.timestamp = currentTime;
        data.persistenceCounter = motionPersistenceCounter;
        data.alertLevel = currentBlinkRate;
        data.alertSystemStopped = alertSystemStopped;
        data.temperature = currentTemperature;
        data.humidity = currentHumidity;
        
        // Generate appropriate status message
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
        
        // Send alert control data to LED task
        AlertLedData alertData;
        alertData.blinkRate = currentBlinkRate;
        alertData.active = motionActive && !alertSystemStopped;
        alertData.forceUpdate = (currentBlinkRate != lastBlinkRate) || alertSystemStopped;
        xQueueSend(alertQueue, &alertData, 0);
        
        lastBlinkRate = currentBlinkRate;
        
        // 50Hz frequency - task runs every 20ms
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Alert LED control task (200Hz, Core 1)
 * 
 * This high-frequency task manages the progressive alert LED on pin 42,
 * providing smooth blinking patterns that correspond to different alert levels.
 * The high frequency ensures responsive LED control and smooth visual feedback.
 * 
 * LED Blink Patterns:
 * - Rate 0: LED off (no alert)
 * - Rate 1: Slow blink (1000ms interval)
 * - Rate 2: Medium blink (500ms interval)
 * - Rate 3: Fast blink (200ms interval)
 * - Rate 4: Critical blink (100ms interval)
 * 
 * Task Specifications:
 * - Frequency: 200Hz (5ms period)
 * - Core: 1 (user interface core)
 * - Priority: 3 (high priority for responsive UI)
 * - Stack: 2048 bytes
 * 
 * @param pvParameters Unused FreeRTOS parameter (NULL)
 * 
 * @note High frequency ensures smooth LED transitions and responsive control
 * @note Receives commands from motion detection task via alertQueue
 */
void alertLedTask(void *pvParameters) {
    Serial.println("Alert LED Task started at 200Hz on Core: " + String(xPortGetCoreID()));
    
    AlertLedData alertData;
    bool ledState = false;
    uint32_t lastBlinkTime = 0;
    uint32_t currentInterval = 0;
    bool blinkingActive = false;
    
    while(1) {
        // Check for new alert commands
        if (xQueueReceive(alertQueue, &alertData, pdMS_TO_TICKS(1)) == pdTRUE) {
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
            }
        }
        
        // Handle LED blinking based on current interval
        if (blinkingActive && currentInterval > 0) {
            uint32_t currentTime = millis();
            if (currentTime - lastBlinkTime >= currentInterval) {
                ledState = !ledState;
                digitalWrite(ALERT_LED_PIN, ledState);
                lastBlinkTime = currentTime;
            }
        }
        
        // 200Hz frequency - task runs every 5ms for smooth LED control
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief LCD display management task (32Hz, Core 1)
 * 
 * This task manages the LCD display, showing current system status, motion
 * information, environmental data, and user interface elements. It receives
 * data from both motion detection and temperature monitoring tasks.
 * 
 * Display Information:
 * - Line 1: Motion status message and current temperature
 * - Line 2: Motion counter, alert level, and button status
 * - Special displays for system stopped state
 * 
 * Task Specifications:
 * - Frequency: 32Hz (31ms period)
 * - Core: 1 (user interface core)
 * - Priority: 1
 * - Stack: 2048 bytes
 * 
 * @param pvParameters Unused FreeRTOS parameter (NULL)
 * 
 * @note Receives data from multiple tasks via motionQueue and temperatureQueue
 * @note Provides comprehensive system status information to user
 */
void displayTask(void *pvParameters) {
    Serial.println("Display Task started at 32Hz on Core: " + String(xPortGetCoreID()));
    
    MotionData receivedData;
    TemperatureData tempData;
    uint32_t lastUpdateTime = 0;
    
    // Initialize display
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp Alert Sys");
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
                // Special display for stopped system
                lcd.setCursor(0, 0);
                lcd.print("ALERT STOPPED");
                lcd.setCursor(0, 1);
                lcd.print("By User Button");
            } else {
                // Normal operation display
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
                
                // Show button availability when counter > threshold
                if (receivedData.persistenceCounter > ALERT_STOP_THRESHOLD) {
                    lcd.print(counterStr + " BTN!");
                } else {
                    lcd.print(counterStr + alertStr);
                }
            }
            
            lastUpdateTime = millis();
        }
        
        // 32Hz frequency - task runs every 31ms
        vTaskDelay(pdMS_TO_TICKS(31));
    }
}
/** @} */

/**
 * @brief System initialization and setup function
 * 
 * Initializes all hardware components, creates FreeRTOS tasks and queues,
 * sets up interrupt handlers, and prepares the system for operation.
 * 
 * Initialization Sequence:
 * 1. Initialize serial communication and pins
 * 2. Initialize AM2302 sensor and LCD display
 * 3. Create FreeRTOS queues for inter-task communication
 * 4. Create and pin tasks to specific CPU cores
 * 5. Set up hardware interrupts for PIR and button
 * 6. Display system ready message
 * 
 * @note This function runs once at system startup
 * @note Tasks are pinned to specific cores for optimal performance
 */
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Motion Detection with 50Hz/64Hz Task Frequencies");
    
    // Initialize GPIO pins
    pinMode(PIR_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(ALERT_LED_PIN, OUTPUT);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ALERT_LED_PIN, LOW);
    
    // Initialize AM2302 sensor
    am2302.begin();
    Serial.println("AM2302 sensor initialized on GPIO " + String(AM2302_PIN));
    
    // Initialize LCD display
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("System Initialization");
    lcd.setCursor(0, 1);
    lcd.print("50Hz/64Hz");
    delay(2000);
    
    // Create FreeRTOS queues for inter-task communication
    motionQueue = xQueueCreate(10, sizeof(MotionData));
    alertQueue = xQueueCreate(10, sizeof(AlertLedData));
    temperatureQueue = xQueueCreate(5, sizeof(TemperatureData));
    
    if (motionQueue == NULL || alertQueue == NULL || temperatureQueue == NULL) {
        Serial.println("Failed to create queues!");
        return;
    }
    
    // Create tasks pinned to specific cores
    // Core 0 tasks (sensor processing)
    xTaskCreatePinnedToCore(temperatureTask, "TempTask64Hz", 4096, NULL, 2, &temperatureTaskHandle, 0);
    xTaskCreatePinnedToCore(motionDetectionTask, "MotionTask50Hz", 4096, NULL, 2, &motionTaskHandle, 0);
    
    // Core 1 tasks (user interface)
    xTaskCreatePinnedToCore(alertLedTask, "AlertLedTask200Hz", 2048, NULL, 3, &alertLedTaskHandle, 1);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask32Hz", 2048, NULL, 1, &displayTaskHandle, 1);
    
    // Setup hardware interrupts
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, RISING);
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonISR, FALLING);
    
    Serial.println("High-Frequency Task System Initialized");
    Serial.println("- Temperature Task: 64Hz (15.6ms period)");
    Serial.println("- Motion Detection Task: 50Hz (20ms period)");
    Serial.println("- Alert LED Task: 200Hz (5ms period)");
    Serial.println("- Display Task: 32Hz (31ms period)");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    lcd.setCursor(0, 1);
    lcd.print("High Freq Mode");
    delay(1000);
}

/**
 * @brief Main loop function (runs on Core 1)
 * 
 * The main loop provides periodic system status reporting and monitoring.
 * Since all main functionality is handled by FreeRTOS tasks, this loop
 * primarily serves as a system health monitor and debug output generator.
 * 
 * Status Report Contents:
 * - Current task frequencies
 * - Environmental conditions (temperature, humidity)
 * - Adjusted alert thresholds based on temperature
 * - Motion persistence counter value
 * - System memory usage
 * 
 * @note Reports system status every 10 seconds
 * @note All main functionality handled by FreeRTOS tasks
 */
void loop() {
    static uint32_t lastStatusPrint = 0;
    
    if (millis() - lastStatusPrint > 10000) {
        uint8_t slowThresh, medThresh, fastThresh, critThresh;
        calculateTemperatureAdjustedThresholds(currentTemperature, slowThresh, medThresh, fastThresh, critThresh);
        
        Serial.println("=== System Status ===");
        Serial.println("- Temperature Task: 64Hz");
        Serial.println("- Motion Task: 50Hz");
        Serial.println("- Temperature: " + String(currentTemperature, 1) + "°C");
        Serial.println("- Humidity: " + String(currentHumidity, 1) + "%");
        Serial.println("- Adjusted Thresholds: " + String(slowThresh) + "," + String(medThresh) + "," + String(fastThresh) + "," + String(critThresh));
        Serial.println("- Persistence Counter: " + String(motionPersistenceCounter));
        Serial.println("- Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        lastStatusPrint = millis();
    }
    
    delay(1000);
}
