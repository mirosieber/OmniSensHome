/**
 * @file enhanced_uart_config.h
 * @brief Enhanced UART configuration with improved parsing integration
 */

#pragma once

#include "uart_config.h"
#include "enhanced_ld2412.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enhanced UART reception task with improved error handling
 * 
 * @param pvParameters Task parameters
 */
void enhanced_uart_reception_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
