#include "esp32-hal.h"
#include "Wire.h"
#include "MTS4x.h"


MTS4X::MTS4X(){

}

bool MTS4X::begin(int32_t sda, int32_t scl){
  if (Wire.begin(sda, scl) != true) {
    return false;
  }
  return true;
}

bool MTS4X::begin(int32_t sda, int32_t scl, MeasurementMode mode){
  if (Wire.begin(sda, scl) != true) {
    return false;
  }
  Wire.setClock(400000); // Kritisch für schnelle Messungen
  setMode(mode, false);
  return true;
}

bool MTS4X::startSingleMessurement(){
  return setMode(MEASURE_SINGLE, false);
}

bool MTS4X::setMode(MeasurementMode mode, bool heater){
  uint8_t command = 0;
  // set bits 7:6 – Mode
  command |= (mode & 0x03) << 6;
  // bits 5:4 = reserved, 00
  if(heater){
    // bits 3:0 – heater on with 0b1010
    command |= 0x0A;
  }
  // send command
  Wire.beginTransmission(MTS4X_ADDRESS);
  Wire.write(MTS4X_TEMP_CMD);
  Wire.write(command);
  return Wire.endTransmission();
}

bool MTS4X::setConfig(TempCfgMPS mps, TempCfgAVG avg, bool sleep){
  uint8_t command = 0;
  command |= mps;    // Bits 7–5
  command |= avg;    // Bits 4–3
  command |= sleep ? 1 : 0;  // Bit 0
  // send command
  Wire.beginTransmission(MTS4X_ADDRESS);
  Wire.write(MTS4X_TEMP_CFG);
  Wire.write(command);
  return Wire.endTransmission();
}

float MTS4X::readTemperature(bool waitOnNewVal){
  if (waitOnNewVal) {
    /*
    uint16_t cnt = 0;
    while (inProgress()) {
      delay(1);
      cnt++;
      if(cnt > 500){
        Serial.println("overflow");
        return -55.0f;
      }
    }
    */
    delay(20); //Fixe wartezeit, da polling nicht geht
  }
  
  Wire.beginTransmission(MTS4X_ADDRESS);
  Wire.write(MTS4X_TEMP_LSB);
  Wire.endTransmission();

  if (Wire.requestFrom(MTS4X_ADDRESS, (uint8_t)2) != 2) {
    return -55.0f;
  }

  if (Wire.available() < 2) {
    return -55.0f;
  }

  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();

  uint16_t rawTemp = ((uint16_t)msb << 8) | lsb;
  
  return MTS4X_RAW_TO_CELSIUS(rawTemp);
}

bool MTS4X::inProgress(){
  Wire.beginTransmission(MTS4X_ADDRESS);
  Wire.write(MTS4X_STATUS);
  Wire.endTransmission();
  if (Wire.requestFrom(MTS4X_ADDRESS, (uint8_t)1) != 1) {
    Serial.println("Progress fail");
    return true;
  }

  uint8_t status = Wire.read();
  return (status & 0x20) != 0;
}
