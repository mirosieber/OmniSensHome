#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "esp_sleep.h"
#include "esp_idf_version.h"

#include "button_config.h"

static const char *TAG = "BUTTON_CONFIG";

// External function declarations from main application
extern void button_single_click_cb(void *arg, void *usr_data);
extern void button_double_click_cb(void *arg, void *usr_data);
extern void button_long_press_cb(void *arg, void *usr_data);

const char *button_event_table[] = {
    "BUTTON_PRESS_DOWN",
    "BUTTON_PRESS_UP", 
    "BUTTON_PRESS_REPEAT",
    "BUTTON_PRESS_REPEAT_DONE",
    "BUTTON_SINGLE_CLICK",
    "BUTTON_DOUBLE_CLICK",
    "BUTTON_MULTIPLE_CLICK",
    "BUTTON_LONG_PRESS_START",
    "BUTTON_LONG_PRESS_HOLD",
    "BUTTON_LONG_PRESS_UP",
};

static void button_event_cb(void *button_handle, void *usr_data)
{
    button_event_t event = (button_event_t)(uintptr_t)usr_data;
    ESP_LOGI(TAG, "Button event: %s", button_event_table[event]);
}

// Internal wrapper functions to handle button events
static void internal_single_click_cb(void *button_handle, void *usr_data)
{
    button_event_t event = (button_event_t)(uintptr_t)usr_data;
    ESP_LOGI(TAG, "Button event: %s", button_event_table[event]);
    button_single_click_cb(button_handle, usr_data);
}

static void internal_double_click_cb(void *button_handle, void *usr_data)
{
    button_event_t event = (button_event_t)(uintptr_t)usr_data;
    ESP_LOGI(TAG, "Button event: %s", button_event_table[event]);
    button_double_click_cb(button_handle, usr_data);
}

static void internal_long_press_cb(void *button_handle, void *usr_data)
{
    button_event_t event = (button_event_t)(uintptr_t)usr_data;
    ESP_LOGI(TAG, "Button event: %s", button_event_table[event]);
    button_long_press_cb(button_handle, usr_data);
}

void button_init(uint32_t button_num)
{
  ESP_LOGI(TAG, "Initializing button on GPIO %lu", button_num);
  
  button_config_t btn_cfg = {
    .long_press_time = 2000,   // 2 seconds for long press
    .short_press_time = 50,    // 50 ms minimum press
  };
  
  button_gpio_config_t gpio_cfg = {
    .gpio_num = button_num,
    .active_level = BUTTON_ACTIVE_LEVEL,
    .enable_power_save = false,
    .disable_pull = false,
  };
  
  button_handle_t btn;
  esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
  if(ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to create button device: %s", esp_err_to_name(ret));
    return;
  }
  
  // Register button event callbacks
  esp_err_t err = ESP_OK;
  
  // Basic events for debugging
  err |= iot_button_register_cb(btn, BUTTON_PRESS_DOWN, NULL, button_event_cb, (void *)(uintptr_t)BUTTON_PRESS_DOWN);
  err |= iot_button_register_cb(btn, BUTTON_PRESS_UP, NULL, button_event_cb, (void *)(uintptr_t)BUTTON_PRESS_UP);
  
  // Main application events
  err |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, internal_single_click_cb, (void *)(uintptr_t)BUTTON_SINGLE_CLICK);
  err |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, NULL, internal_double_click_cb, (void *)(uintptr_t)BUTTON_DOUBLE_CLICK);
  err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, NULL, internal_long_press_cb, (void *)(uintptr_t)BUTTON_LONG_PRESS_UP);
  
  if(err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to register button callbacks: %s", esp_err_to_name(err));
  }
  else
  {
    ESP_LOGI(TAG, "Button initialized successfully with all callbacks");
  }
}

// void power_save_init(void)
// {
//     esp_pm_config_t pm_config = {
//         .max_freq_mhz = CONFIG_EXAMPLE_MAX_CPU_FREQ_MHZ,
//         .min_freq_mhz = CONFIG_EXAMPLE_MIN_CPU_FREQ_MHZ,
// #if CONFIG_FREERTOS_USE_TICKLESS_IDLE
//         .light_sleep_enable = true
// #endif
//     };
//     ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
// }