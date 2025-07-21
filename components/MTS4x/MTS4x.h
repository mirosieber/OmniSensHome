//by MS

#ifndef __MTS4X_H__
#define __MTS4X_H__

#include <Arduino.h>
#include <Wire.h>

//i2c Address
#define MTS4X_ADDRESS 0x41

//Temperature output and conversion formula
#define MTS4X_TEMP_LSB         0x00
#define MTS4X_TEMP_MSB         0x01
#define MTS4X_RAW_TO_CELSIUS(S_T) (((S_T) / 256.0f) + 25.0f)

// list of command registers
#define MTS4X_CRC_TEMP         0x02
#define MTS4X_STATUS           0x03
#define MTS4X_TEMP_CMD         0x04
#define MTS4X_TEMP_CFG         0x05
#define MTS4X_ALERT_MODE       0x06
#define MTS4X_TH_LSB           0x07
#define MTS4X_TH_MSB           0x08
#define MTS4X_TL_LSB           0x09
#define MTS4X_TL_MSB           0x0A
#define MTS4X_CRC_SCRATCH      0x0B

// Benutzerdefinierte Register
#define MTS4X_USER_DEFINE_0    0x0C
#define MTS4X_USER_DEFINE_1    0x0D
#define MTS4X_USER_DEFINE_2    0x0E
#define MTS4X_USER_DEFINE_3    0x0F
#define MTS4X_USER_DEFINE_4    0x10
#define MTS4X_USER_DEFINE_5    0x11
#define MTS4X_USER_DEFINE_6    0x12
#define MTS4X_USER_DEFINE_7    0x13
#define MTS4X_USER_DEFINE_8    0x14
#define MTS4X_USER_DEFINE_9    0x15

#define MTS4X_CRC_SCRATCH_EXT  0x16
#define MTS4X_E2PROM_CMD       0x17
#define MTS4X_DEVICE_ID_LSB    0x18  // oder Romcode1
#define MTS4X_DEVICE_ID_MSB    0x19  // oder Romcode2

// Romcode
#define MTS4X_ROMCODE3         0x1A
#define MTS4X_ROMCODE4         0x1B
#define MTS4X_ROMCODE5         0x1C
#define MTS4X_ROMCODE6         0x1D
#define MTS4X_ROMCODE7         0x1E
#define MTS4X_CRC_ROMCODE      0x1F

typedef enum {
    MEASURE_CONTINUOUS = 0b00,        
    MEASURE_STOP =       0b01, 
    MEASURE_SINGLE =     0b11
} MeasurementMode;

// Messfrequenz (MPS)
typedef enum {
    MPS_8Hz  = 0b000 << 5,
    MPS_4Hz  = 0b001 << 5,
    MPS_2Hz  = 0b010 << 5,
    MPS_1Hz  = 0b011 << 5,
    MPS_0_5Hz = 0b100 << 5,
    MPS_0_25Hz = 0b101 << 5,
    MPS_0_125Hz = 0b110 << 5,
    MPS_0_0625Hz = 0b111 << 5
} TempCfgMPS;

// Mittelwertbildung (AVG)
typedef enum {
    AVG_1   = 0b00 << 3,
    AVG_8   = 0b01 << 3,
    AVG_16  = 0b10 << 3,
    AVG_32  = 0b11 << 3
} TempCfgAVG;


class MTS4X{
  public:
    MTS4X();
    bool begin(int32_t sda, int32_t scl);
    bool begin(int32_t sda, int32_t scl, MeasurementMode mode);
    bool startSingleMessurement();
    bool setMode(MeasurementMode mode, bool heater);
    //sleep geht nur im single modus
    bool setConfig(TempCfgMPS mps, TempCfgAVG avg, bool sleep);
    float readTemperature(bool waitOnNewVal);
  private:
    bool inProgress();
};


#endif // __MTS4X_H__