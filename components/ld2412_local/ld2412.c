#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include <esp_timer.h>

#include "uart_config.h"
#include "ld2412.h"

static const char *TAG = "LD2412";

static bool current_engineering_mode = false;

void send_command(uint8_t *command_str, uint8_t *command_val, int command_val_len) {
  // Initialize frame length Frame header 4 bytes + intra frame length 2 bytes
  int frame_data_index = 0;          
  int intra_frame_data_length = 0;          
  
  // Frame header bytes
  hUart.uart_txBuffer[0] = 0xFD;
  hUart.uart_txBuffer[1] = 0xFC;
  hUart.uart_txBuffer[2] = 0xFB;
  hUart.uart_txBuffer[3] = 0xFA;
  frame_data_index += 4;

  // intra frame data length bytes
  hUart.uart_txBuffer[4] = 0;
  hUart.uart_txBuffer[5] = 0;
  frame_data_index += 2;
  // command word bytes
  hUart.uart_txBuffer[6] = command_str[0];
  hUart.uart_txBuffer[7] = command_str[1];
  frame_data_index += 2;
  intra_frame_data_length += 2;
  // command value bytes
  if (command_val_len > 0) {
    for (int i = 0; i < command_val_len; i++)
    {
      hUart.uart_txBuffer[frame_data_index+i] = command_val[i];
      frame_data_index++;
      intra_frame_data_length++;
    }
  }
  // intra frame data length
  hUart.uart_txBuffer[4] = (uint8_t)(intra_frame_data_length & 0xFF);
  hUart.uart_txBuffer[5] = (uint8_t)((intra_frame_data_length >> 8) & 0xFF);

  hUart.uart_txBuffer[frame_data_index] = 0x04;
  hUart.uart_txBuffer[frame_data_index+1] = 0x03;
  hUart.uart_txBuffer[frame_data_index+2] = 0x02;
  hUart.uart_txBuffer[frame_data_index+3] = 0x01;
  frame_data_index += 4;

  hUart.uart_txPacketSize = frame_data_index;

  // Debug: Log the command being sent
  ESP_LOGI(TAG, "Sending command: CMD=0x%02X%02X, LEN=%d", command_str[0], command_str[1], command_val_len);
  char hex_str[100] = "";
  for (int i = 0; i < frame_data_index && i < 20; i++) {
    char byte_str[6];
    sprintf(byte_str, "%02X ", hUart.uart_txBuffer[i]);
    strcat(hex_str, byte_str);
  }
  ESP_LOGI(TAG, "TX: %s", hex_str);

  // Clear any remaining buffer data to prevent interference
  for (int i = frame_data_index; i < 64; i++) {
    hUart.uart_txBuffer[i] = 0x00;
  }

  xQueueSendToBack(uartTx_queue, &hUart, portMAX_DELAY);
}

void control_config_mode(bool enable) {
  // Command word (2 bytes) 0x00FF or 0x00FE
  uint8_t cmd[2] = {enable ? 0xFF : 0xFE, 0x00};
  uint8_t cmd_val[2] = {};
  int cmd_val_len = enable ? 2 : 0;
  if (enable) {
    // Command value (2 bytes) 0x0001
    cmd_val[0] = 0x01;
    cmd_val[1] = 0x0;
  }
  send_command(cmd, cmd_val, cmd_val_len);
}

uint8_t ld2412_parse_target_data_frame(const uint8_t* frame_data, int16_t* movement_distance, int16_t* static_distance) {
  if (frame_data[6] == 0x02 && frame_data[7] == 0xAA && frame_data[15] == 0x55) { // && frame_data[16] == 0x00) { // DL 24.07.08 sometimes value of 0xF1, 0x9, and 0x1 comes into 17th Check byte, instead of 0x00
    // Normal mode target data 
    *movement_distance = frame_data[9] + (frame_data[10] << 8);
    // *movement_energy = frame_data[11];
    *static_distance = frame_data[12] + (frame_data[13] << 8);
    // *static_energy = frame_data[14];
    // ESP_LOGI(TAG, "Normal mode: target=%d, moving=%d, stationary=%d", frame_data[8], *movement_distance, *static_distance);
    return frame_data[8]; // Normal mode
  } else if (frame_data[6] == 0x01 && frame_data[7] == 0xAA) {
    // Engineering mode target data - remove strict byte [45-46] check as sensor sends different values
    int16_t data_length = frame_data[4] + (frame_data[5] << 8);
    // ESP_LOGI(TAG, "Engineering mode data length: %d", data_length);
    
    // Parse engineering mode data - typical LD2412 engineering mode format
    // Bytes 8: target state, 9-10: moving distance, 12-13: stationary distance  
    *movement_distance = frame_data[9] + (frame_data[10] << 8);
    *static_distance = frame_data[12] + (frame_data[13] << 8);
    
    // ESP_LOGI(TAG, "Engineering mode: target=%d, moving=%d, stationary=%d", frame_data[8], *movement_distance, *static_distance);
    return frame_data[8]; // Engineering mode
  } else {
    // ESP_LOGW(TAG, "Unknown frame format - bytes [6-7]: 0x%02X 0x%02X", frame_data[6], frame_data[7]);
    return 100; // Return 100 if correct target frame not found
  }
}
void ld2412_parse_command_ack_frame(const uint8_t* frame_data) {
  // Debug: Log all command responses to see what we're receiving
  if (frame_data[7] == 0x01) {
    ESP_LOGI(TAG, "Command ACK received: [6-7]=0x%02X%02X, [8-9]=0x%02X%02X", 
             frame_data[6], frame_data[7], frame_data[8], frame_data[9]);
  }

  // Enable engineering mode 
  // Command word: 2 bytes 0x0162
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  if (frame_data[6] == 0x62 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "Successfully Enabled engineering mode");
  } 
  // Close engineering mode 
  // Command word: 2 bytes 0x0163
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  if (frame_data[6] == 0x63 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "Successfully Closed engineering mode");
  }

  // Read Firmware Version 
  // Command word: 2 bytes 0x01A0
  // Return value: 2-bytes ACK status (0 successful, 1 failed) + 2-bytes firmware type (0x2412)+2-bytes major
  //                version number+4-bytes minor version number
  if (frame_data[6] == 0xA0 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "LD2412 firmware type: %02X.%02X", frame_data[11], frame_data[10]);
    ESP_LOGI(TAG, "LD2412 firmware version: V%02X.%02X.%02X.%02X.%02X.%02X", frame_data[13], frame_data[12], frame_data[17], frame_data[16], frame_data[15], frame_data[14]);
  } 
  // Set serial port baud rate 
  // Command word: 2 bytes 0x01A1
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  if (frame_data[6] == 0xA1 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "Successfully Set serial baud rate, please restart the module");
  }

  // Reboot Module
  // Command word: 2 bytes 0x01A3
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  if (frame_data[6] == 0xA3 && frame_data[7] == 0x01) {
    if (frame_data[8] == 0x0 && frame_data[9] == 0x0) {
      ESP_LOGI(TAG, "Module reboot command acknowledged - module will restart");
    } else {
      ESP_LOGW(TAG, "Module reboot command failed (Status: 0x%02X%02X)", frame_data[9], frame_data[8]);
    }
  }

  // Disable/Enable Bluetooth (Official command from documentation)
  // Command word: 2 bytes 0x01A4 
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  // Note: Setting takes effect after module restart
  if (frame_data[6] == 0xA4 && frame_data[7] == 0x01) {
    if (frame_data[8] == 0x0 && frame_data[9] == 0x0) {
      ESP_LOGI(TAG, "Successfully set Bluetooth configuration (restart required)");
    } else {
      ESP_LOGW(TAG, "Failed to set Bluetooth configuration (Status: 0x%02X%02X)", frame_data[9], frame_data[8]);
    }
  }

  // Legacy Bluetooth disable command (keeping for compatibility)
  // Command word: 2 bytes 0x01A5
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  if (frame_data[6] == 0xA5 && frame_data[7] == 0x01) {
    if (frame_data[8] == 0x0 && frame_data[9] == 0x0) {
      ESP_LOGI(TAG, "Successfully disabled Bluetooth");
    } else {
      ESP_LOGW(TAG, "Bluetooth disable not supported by this firmware version (Command: 0xA5, Status: 0x%02X%02X)", frame_data[9], frame_data[8]);
    }
  }
}

/**
 * @brief Enable/Close engineering mode command
 *        Current implementation toggles engineering mode 
 * @param void
 *
 */
void control_engineering_mode(void) {

  current_engineering_mode = !current_engineering_mode;

  // Command word (2 bytes) 0x0062 or 0x0063
  // Command value None
  uint8_t cmd[2] = {current_engineering_mode ? 0x62 : 0x63, 0x00};
  uint8_t cmd_val[2] = {};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  send_command(cmd, cmd_val, 0);
  vTaskDelay(200/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);
}

/**
 * @brief Send command for read firmware version from Radar
 *
 * @param void
 *
 */
void read_firmware_version(void) {
  // Command word (2 bytes) 0x00A0
  // Command value None
  uint8_t cmd[2] = {0xA0, 0x00};
  uint8_t cmd_val[2] = {};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  // ESP_LOGI(TAG, "Time sent firmware read command: %lld us", esp_timer_get_time());
  send_command(cmd, cmd_val, 0);
  vTaskDelay(180/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);
}

/**
 * @brief Set the serial port baud rate
 *
 * @param void
 *
 */
void set_baud_rate(uint8_t *command_val) {
  // Command word (2 bytes) 0x00A1
  uint8_t cmd[2] = {0xA1, 0x00};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "Time sent set baud rate command: %lld us", esp_timer_get_time());
  send_command(cmd, command_val, 2);
  vTaskDelay(180/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);

  restart_module();
}

void restart_module(void) {
  // Command word (2 bytes) 0x00A3
  // Command value None
  uint8_t cmd[2] = {0xA3, 0x00};
  uint8_t cmd_val[2] = {};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  send_command(cmd, cmd_val, 0);
  vTaskDelay(180/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);
}

/**
 * @brief Disable Bluetooth functionality of the LD2412 module
 *
 * @param void
 *
 */
void disable_bluetooth(void) {
  // Command word (2 bytes) 0x00A4  
  // Command value (2 bytes) 0x0000 to disable Bluetooth, 0x0100 to enable Bluetooth
  // Note: Module needs restart for this setting to take effect
  uint8_t cmd[2] = {0xA4, 0x00};
  uint8_t cmd_val[2] = {0x00, 0x00}; // 0x0000 = Turn off Bluetooth (LSB first)

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "Sending disable Bluetooth command (0x00A4 with value 0x0000)");
  send_command(cmd, cmd_val, 2);
  vTaskDelay(800/portTICK_PERIOD_MS);  // Wait longer for Bluetooth command response
  
  control_config_mode(false);
  vTaskDelay(200/portTICK_PERIOD_MS);
  
  ESP_LOGI(TAG, "Bluetooth disable command sent. Waiting before reboot...");
  vTaskDelay(1000/portTICK_PERIOD_MS);  // Additional delay between Bluetooth and reboot commands
  
  ESP_LOGI(TAG, "Starting module reboot to apply Bluetooth setting...");
  
  // Automatically reboot the module to apply Bluetooth setting
  // Command word: 0x00A3, Command value: none
  uint8_t reboot_cmd[2] = {0xA3, 0x00};
  uint8_t reboot_cmd_val[2] = {0x00, 0x00}; // Initialize properly for reboot command

  control_config_mode(true);
  vTaskDelay(200/portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "Sending module reboot command (0x00A3)");
  send_command(reboot_cmd, reboot_cmd_val, 0); // No command value for reboot
  vTaskDelay(500/portTICK_PERIOD_MS);  // Wait longer for reboot ACK before module reboots
  
  control_config_mode(false);
  vTaskDelay(200/portTICK_PERIOD_MS);
  
  ESP_LOGI(TAG, "Module reboot command sent. Module will restart automatically.");
  vTaskDelay(3000/portTICK_PERIOD_MS);  // Give module much more time to reboot completely
}

/**
 * @brief Enable Bluetooth functionality of the LD2412 module
 *
 * @param void
 *
 */
void enable_bluetooth(void) {
  // Command word (2 bytes) 0x00A4  
  // Command value (2 bytes) 0x0000 to disable Bluetooth, 0x0100 to enable Bluetooth
  // Note: Module needs restart for this setting to take effect
  uint8_t cmd[2] = {0xA4, 0x00};
  uint8_t cmd_val[2] = {0x01, 0x00}; // 0x0100 = Turn on Bluetooth (LSB first: 0x01, 0x00)

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "Sending enable Bluetooth command (0x00A4 with value 0x0100)");
  send_command(cmd, cmd_val, 2);
  vTaskDelay(800/portTICK_PERIOD_MS);  // Wait longer for Bluetooth command response
  
  control_config_mode(false);
  vTaskDelay(200/portTICK_PERIOD_MS);
  
  ESP_LOGI(TAG, "Bluetooth enable command sent. Waiting before reboot...");
  vTaskDelay(1000/portTICK_PERIOD_MS);  // Additional delay between Bluetooth and reboot commands
  
  ESP_LOGI(TAG, "Starting module reboot to apply Bluetooth setting...");
  
  // Automatically reboot the module to apply Bluetooth setting
  // Command word: 0x00A3, Command value: none
  uint8_t reboot_cmd[2] = {0xA3, 0x00};
  uint8_t reboot_cmd_val[2] = {0x00, 0x00}; // Initialize properly for reboot command

  control_config_mode(true);
  vTaskDelay(200/portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "Sending module reboot command (0x00A3)");
  send_command(reboot_cmd, reboot_cmd_val, 0); // No command value for reboot
  vTaskDelay(500/portTICK_PERIOD_MS);  // Wait longer for reboot ACK before module reboots
  
  control_config_mode(false);
  vTaskDelay(200/portTICK_PERIOD_MS);
  
  ESP_LOGI(TAG, "Module reboot command sent. Module will restart automatically.");
  vTaskDelay(3000/portTICK_PERIOD_MS);  // Give module much more time to reboot completely
}
