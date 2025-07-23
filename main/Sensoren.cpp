#include "Sensoren.h"

void initSensors(app_config_t *config) {
  // Initialize temperature sensor
  MTS4Z.begin(config->i2c.sda, config->i2c.scl);
  MTS4Z.setMode(MEASURE_CONTINUOUS, false);
  MTS4Z.setConfig(MPS_2Hz, AVG_16, false);
  // Initialize Lux sensor
  opt3004.begin(config->sensors[0].i2c_address);
}
