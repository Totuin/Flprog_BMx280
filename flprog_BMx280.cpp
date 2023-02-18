#include "flprog_BMx280.h"

FLProgBMx280::FLProgBMx280(FLProgI2C *device, uint8_t i2cAddress)
{
    i2cDevice = device;
    addres = i2cAddress;
}

void FLProgBMx280::createError()
{
    isInit = false;
    gotoStepWithDelay(FLPROG_SENSOR_WAITING_READ_STEP, 500);
}

void FLProgBMx280::readSensor()
{
    if (!isInit)
    {
        initDevice();
    }
    if (codeError)
    {
        createError();
        return;
    }
    readTemperature();
}

void FLProgBMx280::initDevice()
{
    uint8_t data[2] = {0x0E, 0xB6};
    codeError = i2cDevice->fullWrite(addres, data, 2);
    if (codeError)
    {
        createError();
        return;
    }
    gotoStepWithDelay(FLPROG_BMX280_INIT_DEVICE_STEP1, 10);
}

void FLProgBMx280::initDeviceStep1()
{

    uint8_t data[25];

    codeError = i2cDevice->fullWrite(addres, 0xD0);
    if (codeError)
    {
        createError();
        return;
    }
    codeError = i2cDevice->fullRead(addres, data, 1);
    if (codeError)
    {
        createError();
        return;
    }

    if (data[0] != 0x60 && data[0] != 0x58)
    {
        codeError == FLPROG_SENSOR_DEVICE_NOT_FOUND_ERROR;
        createError();
        return;
    }

    codeError = i2cDevice->fullWrite(addres, 0x88);
    if (codeError)
    {
        createError();
        return;
    }

    codeError = i2cDevice->fullRead(addres, data, 25);
    if (codeError)
    {
        createError();
        return;
    }
    CalibrationData._T1 = (data[0] | (data[1] << 8));
    CalibrationData._T2 = (data[2] | (data[3] << 8));
    CalibrationData._T3 = (data[4] | (data[5] << 8));
    CalibrationData._P1 = (data[6] | (data[7] << 8));
    CalibrationData._P2 = (data[8] | (data[9] << 8));
    CalibrationData._P3 = (data[10] | (data[11] << 8));
    CalibrationData._P4 = (data[12] | (data[13] << 8));
    CalibrationData._P5 = (data[14] | (data[15] << 8));
    CalibrationData._P6 = (data[16] | (data[17] << 8));
    CalibrationData._P7 = (data[18] | (data[19] << 8));
    CalibrationData._P8 = (data[20] | (data[21] << 8));
    CalibrationData._P9 = (data[22] | (data[23] << 8));
    CalibrationData._H1 = data[24];
    codeError = i2cDevice->fullWrite(addres, 0xE1);
    if (codeError)
    {
        createError();
        return;
    }
    codeError = i2cDevice->fullRead(addres, data, 8);
    if (codeError)
    {
        createError();
        return;
    }
    CalibrationData._H2 = (data[0] | (data[1] << 8));
    CalibrationData._H3 = data[2];
    CalibrationData._H4 = (data[3] << 4);
    uint8_t interVal = data[4];
    CalibrationData._H4 |= (interVal & 0xF);
    CalibrationData._H5 = (((interVal & 0xF0) >> 4) | (data[5] << 4));
    CalibrationData._H6 = data[6];
    codeError = i2cDevice->fullWrite(addres, hum_oversampl);
    if (codeError)
    {
        createError();
        return;
    }
    codeError = i2cDevice->fullWrite(addres, 0xF2);
    if (codeError)
    {
        createError();
        return;
    }
    codeError = i2cDevice->fullRead(addres, data, 1);
    if (codeError)
    {
        createError();
        return;
    }
    data[1] = data[0];
    data[0] = 0xF2;
    codeError = i2cDevice->fullWrite(addres, data, 2);
    {
        createError();
        return;
    }
    data[0] = 0xF4;
    data[1] = (temp_oversampl << 5) | (press_oversampl << 2) | operating_mode;
    codeError = i2cDevice->fullWrite(addres, data, 2);
    {
        createError();
        return;
    }
    data[0] = 0xF5;
    data[1] = (standby_time << 5) | (filter_coef << 2);
    codeError = i2cDevice->fullWrite(addres, data, 2);
    {
        createError();
        return;
    }
    isInit = true;
    step = FLPROG_SENSOR_WAITING_READ_STEP;
    isNeededRead = true;
}

void FLProgBMx280::pool()
{
    checkReadPeriod();
    checkDelay();
    checkNeededRead();
    if (step == FLPROG_BMX280_INIT_DEVICE_STEP1)
    {
        initDeviceStep1();
    }
    if (step == FLPROG_BMX280_READ_PRESSURE_STEP)
    {
        readPressure();
    }
    if (step == FLPROG_BMX280_READ_HUMIDITI_STEP)
    {
        readHumidity();
    }
}

uint32_t FLProgBMx280::readRegister24(uint8_t reg)
{
    uint8_t data[3];
    codeError = i2cDevice->fullWrite(addres, reg);
    if (codeError)
    {
        return 0;
    }

    codeError = i2cDevice->fullRead(addres, data, 3);
    if (codeError)
    {
        return 0;
    }
    return = (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2]);
}

int32_t FLProgBMx280::readTempInt()
{
    int32_t temp_raw = readRegister24(0xFA);
    if (codeError)
    {
        return 0;
    }
    if (temp_raw == 0x800000)
        return 0; // If the temperature module has been disabled return '0'

    temp_raw >>= 4; // Start temperature reading in integers
    int32_t value_1 = ((((temp_raw >> 3) - ((int32_t)CalibrationData._T1 << 1))) *
                       ((int32_t)CalibrationData._T2)) >>
                      11;
    int32_t value_2 = (((((temp_raw >> 4) - ((int32_t)CalibrationData._T1)) *
                         ((temp_raw >> 4) - ((int32_t)CalibrationData._T1))) >>
                        12) *
                       ((int32_t)CalibrationData._T3)) >>
                      14;

    return ((int32_t)value_1 + value_2);
}

void FLProgBMx280::readTemperature()
{
    temp_raw = readTempInt();
    if (codeError)
    {
        createError();
        return;
    }
    float T = (temp_raw * 5 + 128) >> 8;
    temperature = T / 100.0;
    gotoStepWithDelay(FLPROG_BMX280_READ_PRESSURE_STEP, 1);
}

void FLProgBMx280::readPressure()
{
    uint32_t press_raw = readRegister24(0xF7);
    if (codeError)
    {
        createError();
        return;
    }
    if (press_raw == 0x800000)
    {
        return:
    }
    press_raw >>= 4;
    int64_t value_1 = ((int64_t)readTempInt()) - 128000;
    int64_t value_2 = value_1 * value_1 * (int64_t)CalibrationData._P6;
    value_2 = value_2 + ((value_1 * (int64_t)CalibrationData._P5) << 17);
    value_2 = value_2 + (((int64_t)CalibrationData._P4) << 35);
    value_1 = ((value_1 * value_1 * (int64_t)CalibrationData._P3) >> 8) + ((value_1 * (int64_t)CalibrationData._P2) << 12);
    value_1 = (((((int64_t)1) << 47) + value_1)) * ((int64_t)CalibrationData._P1) >> 33;
    if (!value_1)
    {
        return;
    }
    int64_t p = 1048576 - press_raw;
    p = (((p << 31) - value_2) * 3125) / value_1;
    value_1 = (((int64_t)CalibrationData._P9) * (p >> 13) * (p >> 13)) >> 25;
    value_2 = (((int64_t)CalibrationData._P8) * p) >> 19;
    p = ((p + value_1 + value_2) >> 8) + (((int64_t)CalibrationData._P7) << 4);
    pressure = (float)p / 256;
    gotoStepWithDelay(FLPROG_BMX280_READ_HUMIDITI_STEP, 1);
}

void FLProgBMx280::readHumidity()
{
    codeError = i2cDevice->fullWrite(addres, 0xFD);
    if (codeError)
    {
        createError();
        return;
    }
    uint8_t data[2];
    codeError = i2cDevice->fullRead(addres, data, 2);
    if (codeError)
    {
        createError();
        return 0;
    }
    int32_t hum_raw = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    if (hum_raw == 0x8000)
    {
        return;
    }
    int32_t value = (temp_raw - ((int32_t)76800));
    value = (((((hum_raw << 14) - (((int32_t)CalibrationData._H4) << 20) -
                (((int32_t)CalibrationData._H5) * value)) +
               ((int32_t)16384)) >>
              15) *
             (((((((value * ((int32_t)CalibrationData._H6)) >> 10) * (((value *
                                                                        ((int32_t)CalibrationData._H3)) >>
                                                                       11) +
                                                                      ((int32_t)32768))) >>
                 10) +
                ((int32_t)2097152)) *
                   ((int32_t)CalibrationData._H2) +
               8192) >>
              14));
    value = (value - (((((value >> 15) * (value >> 15)) >> 7) * ((int32_t)CalibrationData._H1)) >> 4));
    value = (value < 0) ? 0 : value;
    value = (value > 419430400) ? 419430400 : value;
    float h = (value >> 12);
    humidity = h / 1024.0;
    step = FLPROG_SENSOR_WAITING_READ_STEP;
}

void FLProgBMx280::setMode(uint8_t mode)
{
    operating_mode = mode;
    isInit = false;
}

void FLProgBMx280::setFilter(uint8_t mode)
{
    filter_coef = mode;
    isInit = false;
}

void FLProgBMx280::setStandbyTime(uint8_t mode)
{
    standby_time = mode;
    isInit = false;
}

void FLProgBMx280::setHumOversampling(uint8_t mode)
{
    hum_oversampl = mode;
    isInit = false;
}

void FLProgBMx280::setTempOversampling(uint8_t mode)
{
    temp_oversampl = mode;
    isInit = false;
}

void FLProgBMx280::setPressOversampling(uint8_t mode)
{
    press_oversampl = mode;
    isInit = false;
}
