#ifndef INA219_H
#define INA219_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "math.h"

class INA219 {
public:
    INA219(i2c_inst_t *i2c_instance, uint8_t i2c_addr);
    void I2C_START(uint32_t sda_pin, uint32_t scl_pin, uint32_t speed_khz);
    void init();
    void calibrate(float shunt_resistor_value, float max_expected_amps);
    float read_voltage();
    float read_shunt_voltage();
    float read_current();
    float read_power();

private:
    uint16_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint16_t value);

    //static constexpr uint8_t INA219_I2C_ADDR = 0x40;
    static constexpr uint8_t INA219_REG_CONFIG = 0x00;
    static constexpr uint8_t INA219_REG_SHUNTVOLTAGE = 0x01;
    static constexpr uint8_t INA219_REG_BUSVOLTAGE = 0x02;
    static constexpr uint8_t INA219_REG_POWER = 0x03;
    static constexpr uint8_t INA219_REG_CURRENT = 0x04;
    static constexpr uint8_t INA219_REG_CALIBRATION = 0x05;

    uint8_t _i2c_addr;
    i2c_inst_t *_i2c_instance;
    float _current_LSB;
};

#endif // INA219_H
