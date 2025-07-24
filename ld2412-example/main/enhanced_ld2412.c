/**
 * @file enhanced_ld2412.c
 * @brief Enhanced LD2412 driver implementation with improved error handling and diagnostics
 */

#include "enhanced_ld2412.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ENHANCED_LD2412";

// Static variables for tracking sensor state
static bool firmware_info_received = false;
static char last_firmware_version[64] = {0};
static char last_firmware_type[16] = {0};

/* ENHANCED FUNCTIONALITY IMPLEMENTATIONS ------------------------------------*/

void enhanced_ld2412_set_firmware_received(bool received)
{
    firmware_info_received = received;
    if(received)
    {
        ESP_LOGI(TAG, "✅ Firmware info reception confirmed");
    }
}

bool enhanced_ld2412_get_firmware_received(void)
{
    return firmware_info_received;
}

void enhanced_ld2412_reset_firmware_flag(void)
{
    firmware_info_received = false;
    ESP_LOGD(TAG, "🔄 Firmware reception flag reset");
}

void enhanced_ld2412_parse_command_ack_frame(const uint8_t* frame_data)
{
    if(!frame_data)
    {
        ESP_LOGE(TAG, "❌ NULL frame data provided to ACK parser");
        return;
    }
    
    // Enable engineering mode 
    if (frame_data[6] == 0x62 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) 
    {
        ESP_LOGI(TAG, "✅ Engineering mode ENABLED successfully");
    } 
    // Close engineering mode 
    else if (frame_data[6] == 0x63 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) 
    {
        ESP_LOGI(TAG, "✅ Engineering mode DISABLED successfully");
    }
    // Read Firmware Version 
    else if (frame_data[6] == 0xA0 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) 
    {
        // Store firmware information
        snprintf(last_firmware_type, sizeof(last_firmware_type), "%02X.%02X", frame_data[11], frame_data[10]);
        snprintf(last_firmware_version, sizeof(last_firmware_version), "V%02X.%02X.%02X.%02X.%02X.%02X", 
                frame_data[13], frame_data[12], frame_data[17], frame_data[16], frame_data[15], frame_data[14]);
        
        ESP_LOGI(TAG, "📋 LD2412 Firmware Type: %s", last_firmware_type);
        ESP_LOGI(TAG, "📋 LD2412 Firmware Version: %s", last_firmware_version);
        
        // Set firmware received flag
        enhanced_ld2412_set_firmware_received(true);
    } 
    // Set serial port baud rate 
    else if (frame_data[6] == 0xA1 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) 
    {
        ESP_LOGI(TAG, "✅ Serial baud rate set successfully - module restart required");
    }
    // Module restart
    else if (frame_data[6] == 0xA3 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) 
    {
        ESP_LOGI(TAG, "✅ Module restart command acknowledged");
    }
    else
    {
        ESP_LOGW(TAG, "⚠️  Unknown ACK command: 0x%02X%02X, Status: 0x%02X%02X", 
                frame_data[7], frame_data[6], frame_data[9], frame_data[8]);
    }
}

uint8_t enhanced_ld2412_parse_target_data_frame(const uint8_t* frame_data, int16_t* movement_distance, int16_t* static_distance)
{
    if(!frame_data || !movement_distance || !static_distance)
    {
        ESP_LOGE(TAG, "❌ NULL pointer provided to target data parser");
        return 100;
    }
    
    // Normal mode target data 
    if (frame_data[6] == 0x02 && frame_data[7] == 0xAA && frame_data[15] == 0x55) 
    {
        *movement_distance = frame_data[9] + (frame_data[10] << 8);
        *static_distance = frame_data[12] + (frame_data[13] << 8);
        
        uint8_t target_state = frame_data[8];
        
        // Additional validation
        if(target_state > 2)
        {
            ESP_LOGW(TAG, "⚠️  Unusual target state in normal mode: %d", target_state);
        }
        
        if(*movement_distance > 6000 || *static_distance > 6000)
        {
            ESP_LOGW(TAG, "⚠️  Distance readings seem excessive: Moving=%d, Static=%d", 
                    *movement_distance, *static_distance);
        }
        
        return target_state; // Normal mode
    } 
    // Engineering mode target data
    else if (frame_data[6] == 0x01 && frame_data[7] == 0xAA && frame_data[45] == 0x55 && frame_data[46] == 0x00) 
    {
        ESP_LOGI(TAG, "🔧 Engineering mode data received, length: %d", frame_data[4] + (frame_data[5] << 8));
        // TODO: Implement full engineering mode parsing
        return frame_data[8]; // Engineering mode
    } 
    else 
    {
        ESP_LOGW(TAG, "⚠️  Invalid target frame format detected");
        ESP_LOGW(TAG, "    Header bytes: 0x%02X 0x%02X, Expected: 0x02 0xAA or 0x01 0xAA", frame_data[6], frame_data[7]);
        return 100; // Error code
    }
}

esp_err_t enhanced_ld2412_check_connectivity(void)
{
    ESP_LOGI(TAG, "🔍 Checking sensor connectivity...");
    
    // Reset flag before test
    enhanced_ld2412_reset_firmware_flag();
    
    // Send firmware version request
    read_firmware_version();
    
    // Wait for response (this would be better implemented with a timeout mechanism)
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    if(enhanced_ld2412_get_firmware_received())
    {
        ESP_LOGI(TAG, "✅ Sensor connectivity confirmed");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "❌ Sensor connectivity test failed");
        return ESP_FAIL;
    }
}

esp_err_t enhanced_ld2412_get_status_string(char* buffer, size_t buffer_size)
{
    if(!buffer || buffer_size < 100)
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    snprintf(buffer, buffer_size,
        "LD2412 Status:\n"
        "  Firmware Type: %s\n"
        "  Firmware Version: %s\n"
        "  Connectivity: %s\n"
        "  Last Response: %s",
        strlen(last_firmware_type) > 0 ? last_firmware_type : "Unknown",
        strlen(last_firmware_version) > 0 ? last_firmware_version : "Unknown",
        firmware_info_received ? "OK" : "Failed",
        firmware_info_received ? "Recent" : "None"
    );
    
    return ESP_OK;
}
