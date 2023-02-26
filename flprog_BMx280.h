#pragma once
#include "Arduino.h"
#include "flprogUtilites.h"
#include "flprogI2C.h"

#define FLPROG_BMX280_NORMAL_MODE 0x03
#define FLPROG_BMX280_FORCED_MODE 0x02

#define FLPROG_BMX280_STANDBY_500US 0x00
#define FLPROG_BMX280_STANDBY_10MS 0x06
#define FLPROG_BMX280_STANDBY_20MS 0x07
#define FLPROG_BMX280_STANDBY_6250US 0x01
#define FLPROG_BMX280_STANDBY_125MS 0x02
#define FLPROG_BMX280_STANDBY_250MS 0x03
#define FLPROG_BMX280_STANDBY_500MS 0x04
#define FLPROG_BMX280_STANDBY_1000MS 0x05

#define FLPROG_BMX280_MODULE_DISABLE 0x00
#define FLPROG_BMX280_OVERSAMPLING_1 0x01
#define FLPROG_BMX280_OVERSAMPLING_2 0x02
#define FLPROG_BMX280_OVERSAMPLING_4 0x03
#define FLPROG_BMX280_OVERSAMPLING_8 0x04
#define FLPROG_BMX280_OVERSAMPLING_16 0x05

#define FLPROG_BMX280_FILTER_DISABLE 0x00
#define FLPROG_BMX280_FILTER_COEF_2 0x01
#define FLPROG_BMX280_FILTER_COEF_4 0x02
#define FLPROG_BMX280_FILTER_COEF_8 0x03
#define FLPROG_BMX280_FILTER_COEF_16 0x04

#define FLPROG_BMX280_INIT_DEVICE_STEP1 10
#define FLPROG_BMX280_READ_PRESSURE_STEP 11
#define FLPROG_BMX280_READ_HUMIDITI_STEP 12

#define FLPROG_BMX280_READ_TEMP_RAW_ERROR 100
#define FLPROG_BMX280_READ_PERSS_RAW_ERROR 101
#define FLPROG_BMX280_CONVERT_PERSS_RAW_ERROR 102
#define FLPROG_BMX280_READ_HUM_RAW_ERROR 103

class FLProgBMx280 : public FLProgI2cStepWorkSensor
{
public:
    FLProgBMx280(FLProgI2C *device, uint8_t i2cAddress = 0x76);
    void pool();
    float getTemperature() { return temperature; };
    float getHumidity() { return humidity; };
    float getPressure() { return pressure; };
    void setMode(uint8_t mode);
    void setFilter(uint8_t mode);
    void setStandbyTime(uint8_t mode);
    void setHumOversampling(uint8_t mode);
    void setTempOversampling(uint8_t mode);
    void setPressOversampling(uint8_t mode);
    bool isBME280() { return deviceIsBME280; };

protected:
    virtual void readSensor();
    void initDevice();
    void initDeviceStep1();
    void createError();
    void readTemperature();
    void readPressure();
    void readHumidity();
    int32_t readTempInt();
    uint32_t readRegister24(uint8_t reg);
    float temperature;
    float humidity;
    float pressure;
    bool isInit = false;
    bool deviceIsBME280 = false;
    int32_t temp_raw;
    uint8_t operating_mode = FLPROG_BMX280_NORMAL_MODE;
    uint8_t standby_time = FLPROG_BMX280_STANDBY_250MS;
    uint8_t filter_coef = FLPROG_BMX280_FILTER_COEF_16;
    uint8_t temp_oversampl = FLPROG_BMX280_OVERSAMPLING_4;
    uint8_t hum_oversampl = FLPROG_BMX280_OVERSAMPLING_1;
    uint8_t press_oversampl = FLPROG_BMX280_OVERSAMPLING_2;
    struct
    {
        uint16_t _T1;
        int16_t _T2;
        int16_t _T3;
        uint16_t _P1;
        int16_t _P2;
        int16_t _P3;
        int16_t _P4;
        int16_t _P5;
        int16_t _P6;
        int16_t _P7;
        int16_t _P8;
        int16_t _P9;
        uint8_t _H1;
        int16_t _H2;
        uint8_t _H3;
        int16_t _H4;
        int16_t _H5;
        int8_t _H6;
    } CalibrationData;
};