/**
 * @file enhanced_uart_config.c
 * @brief Enhanced UART configuration implementation
 */

#include "enhanced_uart_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "ENHANCED_UART";

// External variables from original UART config
extern QueueHandle_t system_queue;
extern QueueHandle_t uartRxStore_queue;
extern uartHandler_t hUart;

/**
 * @brief Enhanced UART reception task with improved error handling and parsing
 */
void enhanced_uart_reception_task(void *pvParameters) 
{
    ESP_LOGI(TAG, "=== Enhanced UART Reception Task Started ===");
    
    uartHandler_t uartHandler = {0};
    uint8_t target_type = 0;
    int16_t moving_target = 0;
    int16_t stationary_target = 0;
    system_packet system_buffer = {0};
    uint32_t frame_count = 0;
    uint32_t error_count = 0;

    system_queue = xQueueCreate(10, sizeof(system_packet));
    
    if(!system_queue)
    {
        ESP_LOGE(TAG, "❌ Failed to create system queue");
        vTaskDelete(NULL);
        return;
    }
    
    for(;;) 
    {
        // Waiting for UART packet to get received
        if(xQueueReceive(uartRxStore_queue, (void *)&uartHandler, portMAX_DELAY)) 
        {
            frame_count++;
            bool frame_processed = false;
            
            // Validate minimum frame size
            if(hUart.uart_rxPacketSize < 8)
            {
                ESP_LOGW(TAG, "⚠️  Frame #%lu too small: %d bytes", frame_count, hUart.uart_rxPacketSize);
                error_count++;
                continue;
            }
            
            // Target Data Header Check
            if ((hUart.uart_rxBuffer[0] == 0xF4) && (hUart.uart_rxBuffer[1] == 0xF3) && 
                (hUart.uart_rxBuffer[2] == 0xF2) && (hUart.uart_rxBuffer[3] == 0xF1) &&
                // Target Data End Check
                (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-4] == 0xF8) && 
                (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-3] == 0xF7) && 
                (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-2] == 0xF6) && 
                (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-1] == 0xF5))
            {
                // Parse target frame data using enhanced parser
                target_type = enhanced_ld2412_parse_target_data_frame(hUart.uart_rxBuffer, 
                                                                     &moving_target, 
                                                                     &stationary_target);
                
                if(target_type != 100) 
                {
                    system_buffer.data[0] = target_type;                // Target state 
                    system_buffer.data[1] = moving_target;              // Moving target 
                    system_buffer.data[2] = stationary_target;          // Stationary target 
                    system_buffer.packet_size = 3;
                    
                    if(xQueueSendToBack(system_queue, &system_buffer, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        frame_processed = true;
                        ESP_LOGD(TAG, "📊 Target frame #%lu processed successfully", frame_count);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "⚠️  Failed to send target data to queue");
                        error_count++;
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "⚠️  Target frame #%lu parsing failed", frame_count);
                    error_count++;
                }
            }  
            // Command Data Header Check
            else if ((hUart.uart_rxBuffer[0] == 0xFD) && (hUart.uart_rxBuffer[1] == 0xFC) && 
                     (hUart.uart_rxBuffer[2] == 0xFB) && (hUart.uart_rxBuffer[3] == 0xFA) &&
                     // Command Data End Check
                     (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-4] == 0x04) && 
                     (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-3] == 0x03) && 
                     (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-2] == 0x02) && 
                     (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-1] == 0x01))
            {
                // Parse ACK command frame data using enhanced parser
                enhanced_ld2412_parse_command_ack_frame(hUart.uart_rxBuffer);
                frame_processed = true;
                ESP_LOGD(TAG, "📡 Command ACK frame #%lu processed successfully", frame_count);
            }
            else
            {
                ESP_LOGW(TAG, "⚠️  Frame #%lu: Unknown frame format", frame_count);
                ESP_LOGW(TAG, "    Header: 0x%02X%02X%02X%02X, Size: %d bytes", 
                        hUart.uart_rxBuffer[0], hUart.uart_rxBuffer[1], 
                        hUart.uart_rxBuffer[2], hUart.uart_rxBuffer[3], 
                        hUart.uart_rxPacketSize);
                error_count++;
            }
            
            // Periodic statistics logging
            if(frame_count % 100 == 0)
            {
                ESP_LOGI(TAG, "📈 Stats: %lu frames processed, %lu errors (%.1f%% error rate)", 
                        frame_count, error_count, 
                        frame_count > 0 ? (float)error_count * 100.0f / frame_count : 0.0f);
            }
        }
        else
        {
            ESP_LOGW(TAG, "⚠️  UART reception queue timeout");
        }
    }
}
