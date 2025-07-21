/*

Arduino/ESP32 library for Texas Instruments OPT300x Digital Ambient Light Sensor
Famyily Tested on OPT 3001 and 3004

---

The MIT License (MIT)

Copyright (c) 2015 ClosedCube Limited

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/
#include <Wire.h>

#include "OPT300x.h"

OPT300x::OPT300x() {}

OPT300x_ErrorCode OPT300x::begin(uint8_t address) {
  OPT300x_ErrorCode error = NO_ERROR;
  _address = address;

  return NO_ERROR;
}

uint16_t OPT300x::readManufacturerID() {
  uint16_t result = 0;
  OPT300x_ErrorCode error = writeData(MANUFACTURER_ID);
  if (error == NO_ERROR)
    error = readData(&result);
  return result;
}

uint16_t OPT300x::readDeviceID() {
  uint16_t result = 0;
  OPT300x_ErrorCode error = writeData(DEVICE_ID);
  if (error == NO_ERROR)
    error = readData(&result);
  return result;
}

OPT300x_Config OPT300x::readConfig() {
  OPT300x_Config config;
  OPT300x_ErrorCode error = writeData(CONFIG);
  if (error == NO_ERROR)
    error = readData(&config.rawData);
  return config;
}

OPT300x_ErrorCode OPT300x::writeConfig(OPT300x_Config config) {
  Wire.beginTransmission(_address);
  Wire.write(CONFIG);
  Wire.write(config.rawData >> 8);
  Wire.write(config.rawData & 0x00FF);
  return (OPT300x_ErrorCode)(-10 * Wire.endTransmission());
}

OPT300x_S OPT300x::readResult() { return readRegister(RESULT); }

OPT300x_S OPT300x::readHighLimit() { return readRegister(HIGH_LIMIT); }

OPT300x_S OPT300x::readLowLimit() { return readRegister(LOW_LIMIT); }

OPT300x_S OPT300x::readRegister(OPT300x_Commands command) {
  OPT300x_ErrorCode error = writeData(command);
  if (error == NO_ERROR) {
    OPT300x_S result;
    result.lux = 0;
    result.error = NO_ERROR;

    OPT300x_ER er;
    error = readData(&er.rawData);
    if (error == NO_ERROR) {
      result.raw = er;
      result.lux = 0.01 * pow(2, er.Exponent) * er.Result;
    } else {
      result.error = error;
    }

    return result;
  } else {
    return returnError(error);
  }
}

OPT300x_ErrorCode OPT300x::writeData(OPT300x_Commands command) {
  Wire.beginTransmission(_address);
  Wire.write(command);
  return (OPT300x_ErrorCode)(-10 * Wire.endTransmission(true));
}

OPT300x_ErrorCode OPT300x::readData(uint16_t *data) {
  uint8_t buf[2];

  Wire.requestFrom(_address, (uint8_t)2);

  int counter = 0;
  while (Wire.available() < 2) {
    counter++;
    delay(10);
    if (counter > 250)
      return TIMEOUT_ERROR;
  }

  Wire.readBytes(buf, 2);
  *data = (buf[0] << 8) | buf[1];

  return NO_ERROR;
}

OPT300x_S OPT300x::returnError(OPT300x_ErrorCode error) {
  OPT300x_S result;
  result.lux = 0;
  result.error = error;
  return result;
}
