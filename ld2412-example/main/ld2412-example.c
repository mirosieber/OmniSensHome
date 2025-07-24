#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"

#include "ld2412.h"
#include "uart_config.h"
#include "button_config.h"
#include "enhanced_ld2412.h"

static const char *TAG = "LD2412_ADVANCED_DEMO";

/* CONSTANTS -----------------------------------------------------------------*/
#define CONNECTIVITY_CHECK_INTERVAL_MS     5000
#define FIRMWARE_READ_INTERVAL_MS          10000
#define SENSOR_DATA_TIMEOUT_MS             3000
#define MAX_CONSECUTIVE_ERRORS             5
#define CONNECTIVITY_TIMEOUT_MS            2000

/* VARIABLES -----------------------------------------------------------------*/
QueueHandle_t system_queue;
static TimerHandle_t connectivity_timer;
static TimerHandle_t firmware_timer;
static TimerHandle_t watchdog_timer;

// Status tracking variables
static bool sensor_connected = false;
static bool firmware_info_received = false;
static uint32_t data_packet_count = 0;
static uint32_t error_count = 0;
static uint32_t consecutive_errors = 0;
static uint64_t last_data_timestamp = 0;
static bool engineering_mode_active = false;

/* PRIVATE FUNCTIONS DECLARATION ---------------------------------------------*/
static void sensor_data_task(void *param);
static void connectivity_check_task(void *param);
static void system_monitor_task(void *param);
static void connectivity_timer_callback(TimerHandle_t xTimer);
static void firmware_timer_callback(TimerHandle_t xTimer);
static void watchdog_timer_callback(TimerHandle_t xTimer);
static void print_system_status(void);
static void handle_sensor_error(const char* error_msg);
static bool check_sensor_connectivity(void);
static void reset_error_counters(void);
void button_single_click_cb(void *arg, void *usr_data);
void button_double_click_cb(void *arg, void *usr_data);
void button_long_press_cb(void *arg, void *usr_data);

/**
 * @brief Enhanced sensor data processing task with comprehensive error handling
 */
void sensor_data_task(void* param)
{
    ESP_LOGI(TAG, "=== Sensor Data Task Started ===");
    
    system_packet system_buffer = {0};
    uint32_t no_data_count = 0;
    
    while(1)
    {
        // Wait for data with timeout
        if(xQueueReceive(system_queue, (void *)&system_buffer, pdMS_TO_TICKS(1000)))
        {
            // Reset error counters on successful data reception
            no_data_count = 0;
            consecutive_errors = 0;
            data_packet_count++;
            last_data_timestamp = esp_timer_get_time();
            
            // Validate data packet
            if(system_buffer.packet_size == 3)
            {
                uint8_t target_state = system_buffer.data[0];
                int16_t moving_target = system_buffer.data[1];
                int16_t stationary_target = system_buffer.data[2];
                
                // Log detailed information
                ESP_LOGI(TAG, "📊 DATA #%lu | Target State: %d | Moving: %d cm | Stationary: %d cm | Time: %lld us", 
                         data_packet_count, target_state, moving_target, stationary_target, last_data_timestamp);
                
                // Analyze data quality and patterns
                if(target_state > 2)
                {
                    ESP_LOGW(TAG, "⚠️  Unusual target state detected: %d", target_state);
                }
                
                if(moving_target > 6000 || stationary_target > 6000)
                {
                    ESP_LOGW(TAG, "⚠️  Distance reading seems out of normal range");
                }
                
                // Mark sensor as connected if we're receiving valid data
                if(!sensor_connected)
                {
                    sensor_connected = true;
                    ESP_LOGI(TAG, "✅ Sensor connectivity restored!");
                    reset_error_counters();
                }
            }
            else
            {
                handle_sensor_error("Invalid data packet size received");
            }
        }
        else
        {
            // No data received within timeout
            no_data_count++;
            
            if(no_data_count > 3)
            {
                ESP_LOGW(TAG, "⚠️  No sensor data received for %lu seconds", no_data_count);
                sensor_connected = false;
                
                if(no_data_count > 10)
                {
                    handle_sensor_error("Extended data timeout - sensor may be disconnected");
                    no_data_count = 0; // Reset to avoid spam
                }
            }
        }
    }
}

/**
 * @brief Connectivity monitoring task
 */
void connectivity_check_task(void* param)
{
    ESP_LOGI(TAG, "=== Connectivity Check Task Started ===");
    
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(CONNECTIVITY_CHECK_INTERVAL_MS));
        
        ESP_LOGI(TAG, "🔍 Performing connectivity check...");
        
        if(check_sensor_connectivity())
        {
            ESP_LOGI(TAG, "✅ Sensor connectivity OK");
        }
        else
        {
            ESP_LOGW(TAG, "❌ Sensor connectivity issue detected");
            handle_sensor_error("Connectivity check failed");
        }
    }
}

/**
 * @brief System monitoring and status reporting task
 */
void system_monitor_task(void* param)
{
    ESP_LOGI(TAG, "=== System Monitor Task Started ===");
    
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(30000)); // Every 30 seconds
        print_system_status();
    }
}

/**
 * @brief Timer callback for periodic connectivity checks
 */
static void connectivity_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "⏰ Connectivity timer triggered");
    
    // Request firmware version to test communication
    read_firmware_version();
}

/**
 * @brief Timer callback for periodic firmware information requests
 */
static void firmware_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "⏰ Firmware info timer triggered");
    read_firmware_version();
}

/**
 * @brief Watchdog timer callback to detect system hang
 */
static void watchdog_timer_callback(TimerHandle_t xTimer)
{
    uint64_t current_time = esp_timer_get_time();
    uint64_t time_since_last_data = current_time - last_data_timestamp;
    
    if(time_since_last_data > (SENSOR_DATA_TIMEOUT_MS * 1000))
    {
        ESP_LOGW(TAG, "🚨 WATCHDOG: No sensor data for %llu ms", time_since_last_data / 1000);
        handle_sensor_error("Watchdog timeout - no data received");
    }
}

/**
 * @brief Print comprehensive system status
 */
static void print_system_status(void)
{
    ESP_LOGI(TAG, "\n" \
                  "╔══════════════════════════════════════════════════╗\n" \
                  "║                SYSTEM STATUS REPORT              ║\n" \
                  "╠══════════════════════════════════════════════════╣\n" \
                  "║ Sensor Connected:      %-25s ║\n" \
                  "║ Engineering Mode:      %-25s ║\n" \
                  "║ Firmware Info Rcvd:    %-25s ║\n" \
                  "║ Total Data Packets:    %-25lu ║\n" \
                  "║ Total Errors:          %-25lu ║\n" \
                  "║ Consecutive Errors:    %-25lu ║\n" \
                  "║ Last Data Time:        %lld us%-10s ║\n" \
                  "║ Free Heap:             %-20lu KB ║\n" \
                  "║ Uptime:                %lld us%-10s ║\n" \
                  "╚══════════════════════════════════════════════════╝",
                  sensor_connected ? "YES" : "NO",
                  engineering_mode_active ? "ACTIVE" : "INACTIVE", 
                  firmware_info_received ? "YES" : "NO",
                  data_packet_count,
                  error_count,
                  consecutive_errors,
                  last_data_timestamp, "",
                  esp_get_free_heap_size() / 1024,
                  esp_timer_get_time(), "");
}

/**
 * @brief Handle sensor errors with escalating response
 */
static void handle_sensor_error(const char* error_msg)
{
    error_count++;
    consecutive_errors++;
    
    ESP_LOGE(TAG, "🚨 SENSOR ERROR #%lu: %s", error_count, error_msg);
    
    if(consecutive_errors >= MAX_CONSECUTIVE_ERRORS)
    {
        ESP_LOGE(TAG, "🚨 CRITICAL: %lu consecutive errors detected!", consecutive_errors);
        ESP_LOGE(TAG, "🔄 Attempting sensor recovery...");
        
        // Attempt recovery
        sensor_connected = false;
        
        // Try to restart communication
        vTaskDelay(pdMS_TO_TICKS(1000));
        read_firmware_version();
        
        consecutive_errors = 0; // Reset after recovery attempt
    }
}

/**
 * @brief Check sensor connectivity by requesting firmware version
 */
static bool check_sensor_connectivity(void)
{
    enhanced_ld2412_reset_firmware_flag();
    
    ESP_LOGI(TAG, "🔍 Requesting firmware version for connectivity test...");
    read_firmware_version();
    
    // Wait for response
    vTaskDelay(pdMS_TO_TICKS(CONNECTIVITY_TIMEOUT_MS));
    
    return enhanced_ld2412_get_firmware_received();
}

/**
 * @brief Reset error counters after successful operation
 */
static void reset_error_counters(void)
{
    consecutive_errors = 0;
    ESP_LOGI(TAG, "✅ Error counters reset - sensor operation normal");
}

/**
 * @brief Button single click callback - Toggle engineering mode
 */
void button_single_click_cb(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "🔘 Button single click - Toggling engineering mode");
    engineering_mode_active = !engineering_mode_active;
    control_engineering_mode();
}

/**
 * @brief Button double click callback - Request firmware version
 */
void button_double_click_cb(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "🔘 Button double click - Requesting firmware version");
    read_firmware_version();
}

/**
 * @brief Button long press callback - Print system status
 */
void button_long_press_cb(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "🔘 Button long press - Printing system status");
    print_system_status();
}

void app_main(void)
{
    ESP_LOGI(TAG, "\n" \
                  "╔══════════════════════════════════════════════════╗\n" \
                  "║        LD2412 ADVANCED SENSOR DEMO v2.0          ║\n" \
                  "║          Enhanced with Error Handling            ║\n" \
                  "╚══════════════════════════════════════════════════╝");
    
    // Initialize UART communication
    ESP_LOGI(TAG, "🔧 Initializing UART communication...");
    uart_config();
    
    // Initialize button with enhanced callbacks
    ESP_LOGI(TAG, "🔧 Initializing button interface...");
    button_init(BOOT_BUTTON_NUM);
    
    // Create timers for periodic operations
    ESP_LOGI(TAG, "🔧 Creating system timers...");
    connectivity_timer = xTimerCreate("ConnectivityTimer", 
                                    pdMS_TO_TICKS(CONNECTIVITY_CHECK_INTERVAL_MS),
                                    pdTRUE, NULL, connectivity_timer_callback);
    
    firmware_timer = xTimerCreate("FirmwareTimer",
                                pdMS_TO_TICKS(FIRMWARE_READ_INTERVAL_MS), 
                                pdTRUE, NULL, firmware_timer_callback);
    
    watchdog_timer = xTimerCreate("WatchdogTimer",
                                pdMS_TO_TICKS(SENSOR_DATA_TIMEOUT_MS),
                                pdTRUE, NULL, watchdog_timer_callback);
    
    // Start timers
    if(connectivity_timer && firmware_timer && watchdog_timer)
    {
        xTimerStart(connectivity_timer, 0);
        xTimerStart(firmware_timer, 0);
        xTimerStart(watchdog_timer, 0);
        ESP_LOGI(TAG, "✅ All timers started successfully");
    }
    else
    {
        ESP_LOGE(TAG, "❌ Failed to create system timers!");
    }
    
    // Create and start tasks
    ESP_LOGI(TAG, "🔧 Creating system tasks...");
    
    BaseType_t task_result = pdPASS;
    
    task_result &= xTaskCreate(uart_event_task, "uart_event_task", 12000, NULL, 5, NULL);
    task_result &= xTaskCreate(uart_transmission_task, "uart_tx_task", 8000, NULL, 4, NULL);
    task_result &= xTaskCreate(uart_reception_task, "uart_rx_task", 10000, NULL, 4, NULL);
    task_result &= xTaskCreate(sensor_data_task, "sensor_data_task", 8000, NULL, 3, NULL);
    task_result &= xTaskCreate(connectivity_check_task, "connectivity_task", 6000, NULL, 2, NULL);
    task_result &= xTaskCreate(system_monitor_task, "system_monitor", 6000, NULL, 1, NULL);
    
    if(task_result == pdPASS)
    {
        ESP_LOGI(TAG, "✅ All tasks created successfully");
    }
    else
    {
        ESP_LOGE(TAG, "❌ Failed to create one or more tasks!");
    }
    
    // Initial sensor setup
    ESP_LOGI(TAG, "🔧 Performing initial sensor setup...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Allow system to stabilize
    
    // Request initial firmware version
    ESP_LOGI(TAG, "📡 Requesting initial firmware version...");
    read_firmware_version();
    
    ESP_LOGI(TAG, "🚀 System initialization complete!");
    ESP_LOGI(TAG, "📋 Button Functions:");
    ESP_LOGI(TAG, "   • Single Click: Toggle Engineering Mode");
    ESP_LOGI(TAG, "   • Double Click: Request Firmware Version");
    ESP_LOGI(TAG, "   • Long Press:   Print System Status");
}