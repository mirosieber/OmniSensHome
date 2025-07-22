#pragma once

#include "esp_err.h"
#include <stdbool.h>

#define MAX_SENSORS 10 // Sensors can vary, still fixed size
#define MAX_SWITCHES 4
#define MAX_RELAYS 4

typedef struct {
  char type[32];
  bool enabled;
  char i2c_address[8];
  int interrupt;
  int ws, sck, sd;
  int rx, tx, out;
  int baudrate;
  char name[64];
} sensor_config_t;

typedef struct {
  bool enabled;
  int pin;
  char name[32];
} switch_config_t;

typedef struct {
  bool enabled;
  int pin;
  char name[32];
} relay_config_t;

typedef struct {
  char type[32];
  char power_supply[16];
  char description[128];
  char hw_version[16];
  char fw_version[16];
  char serial[32];
  char manufacturer[32];
  char model[32];
} device_info_t;

typedef struct {
  bool enabled;
  char ssid[64];
  char password[64];
} wifi_config_t;

typedef struct {
  bool enabled;
  int red_pin;
  int green_pin;
  int blue_pin;
} rgb_led_config_t;

typedef struct {
  int sda;
  int scl;
} i2c_config_t;

typedef struct {
  bool enabled;
  char type[32];
  int lrc;
  int bclk;
  int din;
} speaker_config_t;

typedef struct {
  bool enabled;
  int ws;
  int sck;
  int sd;
} microphone_config_t;

typedef struct {
  bool enabled;
  char type[32];
} display_config_t;

typedef struct {
  device_info_t device;
  wifi_config_t wifi;
  rgb_led_config_t rgb_led;
  i2c_config_t i2c;
  speaker_config_t speaker;
  microphone_config_t microphone;
  display_config_t display;

  int factory_reset_pin;

  sensor_config_t sensors[MAX_SENSORS];
  int num_sensors;

  switch_config_t switches[MAX_SWITCHES];
  int num_switches;

  relay_config_t relays[MAX_RELAYS];
  int num_relays;

} app_config_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t config_load(app_config_t *config);

#ifdef __cplusplus
}
#endif
