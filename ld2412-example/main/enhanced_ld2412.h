/**
 * @file enhanced_ld2412.h
 * @brief Enhanced LD2412 driver with improved error handling and diagnostics
 * 
 * This header provides additional functionality for the LD2412 sensor including
 * connectivity monitoring, error tracking, and enhanced debugging capabilities.
 */

#pragma once

#include "ld2412.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ENHANCED FUNCTIONALITY DECLARATIONS ---------------------------------------*/

/**
 * @brief Set firmware info received flag
 * 
 * @param received True if firmware info was successfully received
 */
void enhanced_ld2412_set_firmware_received(bool received);

/**
 * @brief Get firmware info received status
 * 
 * @return true if firmware info was received, false otherwise
 */
bool enhanced_ld2412_get_firmware_received(void);

/**
 * @brief Reset firmware info received flag
 */
void enhanced_ld2412_reset_firmware_flag(void);

/**
 * @brief Enhanced command ACK frame parser with additional logging
 * 
 * @param frame_data The input frame data
 */
void enhanced_ld2412_parse_command_ack_frame(const uint8_t* frame_data);

/**
 * @brief Enhanced target data frame parser with validation
 * 
 * @param frame_data The input frame data
 * @param movement_distance Pointer to store moving target distance
 * @param static_distance Pointer to store stationary target distance
 * @return Target state (0-2 valid, 100 for error)
 */
uint8_t enhanced_ld2412_parse_target_data_frame(const uint8_t* frame_data, int16_t* movement_distance, int16_t* static_distance);

/**
 * @brief Check if sensor is responsive by sending a test command
 * 
 * @return ESP_OK if sensor responds, ESP_FAIL otherwise
 */
esp_err_t enhanced_ld2412_check_connectivity(void);

/**
 * @brief Get detailed sensor status string
 * 
 * @param buffer Buffer to write status string to
 * @param buffer_size Size of the buffer
 * @return ESP_OK on success
 */
esp_err_t enhanced_ld2412_get_status_string(char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif
