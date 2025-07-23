// config.c

#include "configLoader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "cJSON.h"

static const char *TAG = "config";

static const char* get_string(cJSON *obj, const char *key, const char *default_val) {
    cJSON *item = cJSON_GetObjectItem(obj, key);
    return (item && cJSON_IsString(item)) ? item->valuestring : default_val;
}

static bool get_bool(cJSON *obj, const char *key, bool default_val) {
    cJSON *item = cJSON_GetObjectItem(obj, key);
    return (item && cJSON_IsBool(item)) ? cJSON_IsTrue(item) : default_val;
}

static int get_int(cJSON *obj, const char *key, int default_val) {
    cJSON *item = cJSON_GetObjectItem(obj, key);
    return (item && cJSON_IsNumber(item)) ? item->valueint : default_val;
}

static int get_hex_address(cJSON *obj, const char *key, int default_val) {
    cJSON *item = cJSON_GetObjectItem(obj, key);
    if (item && cJSON_IsString(item)) {
        char *hex_str = item->valuestring;
        if (hex_str && strlen(hex_str) > 0) {
            // Handle both "0x48" and "48" formats
            if (strncmp(hex_str, "0x", 2) == 0 || strncmp(hex_str, "0X", 2) == 0) {
                return (int)strtol(hex_str, NULL, 16);
            } else {
                return (int)strtol(hex_str, NULL, 16);
            }
        }
    }
    return default_val;
}

esp_err_t config_load(app_config_t *config) {
    memset(config, 0, sizeof(app_config_t));

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    FILE *f = fopen("/spiffs/config.json", "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open config file");
        return ESP_FAIL;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);

    char *data = malloc(size + 1);
    fread(data, 1, size, f);
    data[size] = '\0';
    fclose(f);

    cJSON *root = cJSON_Parse(data);
    free(data);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse error");
        return ESP_FAIL;
    }

    // Device
    cJSON *dev = cJSON_GetObjectItem(root, "Device");
    strcpy(config->device.type, get_string(dev, "Type", ""));
    strcpy(config->device.power_supply, get_string(dev, "PowerSupply", ""));
    strcpy(config->device.description, get_string(dev, "Description", ""));
    strcpy(config->device.hw_version, get_string(dev, "HardwareVersion", ""));
    strcpy(config->device.fw_version, get_string(dev, "FirmwareVersion", ""));
    strcpy(config->device.serial, get_string(dev, "SerialNumber", ""));
    strcpy(config->device.manufacturer, get_string(dev, "Manufacturer", ""));
    strcpy(config->device.model, get_string(dev, "Model", ""));

    // Factory reset
    config->factory_reset_pin = get_int(cJSON_GetObjectItem(root, "FactoryResetButton"), "Pin", -1);

    // WiFi
    cJSON *wifi = cJSON_GetObjectItem(root, "WiFi");
    config->wifi.enabled = get_bool(wifi, "Enabled", false);
    strcpy(config->wifi.ssid, get_string(wifi, "SSID", ""));
    strcpy(config->wifi.password, get_string(wifi, "Password", ""));

    // RGB LED
    cJSON *led = cJSON_GetObjectItem(root, "RGBLed");
    config->rgb_led.enabled = get_bool(led, "Enabled", false);
    config->rgb_led.red_pin = get_int(led, "Red", -1);
    config->rgb_led.green_pin = get_int(led, "Green", -1);
    config->rgb_led.blue_pin = get_int(led, "Blue", -1);

    // I2C
    cJSON *i2c = cJSON_GetObjectItem(root, "I2c");
    config->i2c.sda = get_int(i2c, "SDA", -1);
    config->i2c.scl = get_int(i2c, "SCL", -1);

    // Speaker
    cJSON *spk = cJSON_GetObjectItem(root, "Speaker");
    config->speaker.enabled = get_bool(spk, "Enabled", false);
    strcpy(config->speaker.type, get_string(spk, "Type", ""));
    config->speaker.lrc = get_int(spk, "LRC", -1);
    config->speaker.bclk = get_int(spk, "BCLK", -1);
    config->speaker.din = get_int(spk, "DIN", -1);

    // Microphone
    cJSON *mic = cJSON_GetObjectItem(root, "Microphone");
    config->microphone.enabled = get_bool(mic, "Enabled", false);
    config->microphone.ws = get_int(mic, "WS", -1);
    config->microphone.sck = get_int(mic, "SCK", -1);
    config->microphone.sd = get_int(mic, "SD", -1);

    // Display
    cJSON *disp = cJSON_GetObjectItem(root, "Display");
    config->display.enabled = get_bool(disp, "Enabled", false);
    strcpy(config->display.type, get_string(disp, "Type", ""));

    // Sensors
    cJSON *sensors = cJSON_GetObjectItem(root, "Sensors");
    if (cJSON_IsArray(sensors)) {
        int count = cJSON_GetArraySize(sensors);
        config->num_sensors = (count < MAX_SENSORS) ? count : MAX_SENSORS;
        for (int i = 0; i < config->num_sensors; ++i) {
            cJSON *s = cJSON_GetArrayItem(sensors, i);
            sensor_config_t *sc = &config->sensors[i];
            strcpy(sc->type, get_string(s, "Type", ""));
            sc->enabled = get_bool(s, "Enabled", false);
            sc->i2c_address = get_hex_address(s, "I2cAddress", 0);
            sc->interrupt = get_int(s, "Interrupt", -1);
            sc->ws = get_int(s, "WS", -1);
            sc->sck = get_int(s, "SCK", -1);
            sc->sd = get_int(s, "SD", -1);
            sc->rx = get_int(s, "RX", -1);
            sc->tx = get_int(s, "TX", -1);
            sc->out = get_int(s, "Out", -1);
            sc->baudrate = get_int(s, "Baudrate", 0);
            strcpy(sc->name, get_string(s, "Name", ""));
        }
    }

    // Switches
    cJSON *switches = cJSON_GetObjectItem(root, "Switches");
    if (cJSON_IsArray(switches)) {
        int count = cJSON_GetArraySize(switches);
        config->num_switches = (count < MAX_SWITCHES) ? count : MAX_SWITCHES;
        for (int i = 0; i < config->num_switches; ++i) {
            cJSON *sw = cJSON_GetArrayItem(switches, i);
            switch_config_t *swc = &config->switches[i];
            swc->enabled = get_bool(sw, "Enabled", false);
            swc->pin = get_int(sw, "Pin", -1);
            strcpy(swc->name, get_string(sw, "Name", ""));
        }
    }

    // Relays
    cJSON *relays = cJSON_GetObjectItem(root, "Relays");
    if (cJSON_IsArray(relays)) {
        int count = cJSON_GetArraySize(relays);
        config->num_relays = (count < MAX_RELAYS) ? count : MAX_RELAYS;
        for (int i = 0; i < config->num_relays; ++i) {
            cJSON *r = cJSON_GetArrayItem(relays, i);
            relay_config_t *rc = &config->relays[i];
            rc->enabled = get_bool(r, "Enabled", false);
            rc->pin = get_int(r, "Pin", -1);
            strcpy(rc->name, get_string(r, "Name", ""));
        }
    }

    cJSON_Delete(root);


    ESP_LOGI(TAG, "Configuration loaded successfully");
    
    return ESP_OK;
}
