#include "INA219.h"

INA219::INA219(i2c_inst_t *i2c_instance, uint8_t i2c_addr) : _i2c_instance(i2c_instance), _i2c_addr(i2c_addr), _current_LSB(0.0) {}

void INA219::I2C_START(uint32_t sda_pin, uint32_t scl_pin, uint32_t speed_khz) {
    i2c_init(_i2c_instance, speed_khz * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

uint16_t INA219::read_register(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(_i2c_instance, _i2c_addr, &reg, 1, true);
    i2c_read_blocking(_i2c_instance, _i2c_addr, buf, 2, false);

    return (buf[0] << 8) | buf[1];
}

void INA219::write_register(uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;
    i2c_write_blocking(_i2c_instance, _i2c_addr, buf, 3, false);
}

void INA219::init() {
    uint16_t config = 0x399F;
    write_register(INA219_REG_CONFIG, config);
    sleep_ms(10);
}

void INA219::calibrate(float shunt_resistor_value, float max_expected_amps) {
    _current_LSB = max_expected_amps / 32768.0;
    uint16_t cal_reg_value = (uint16_t)(0.04096 / (_current_LSB * shunt_resistor_value));
    write_register(INA219_REG_CALIBRATION, cal_reg_value);
}

float INA219::read_voltage() {
    uint16_t value = read_register(INA219_REG_BUSVOLTAGE);
    return (value >> 3) * 0.004;
}

float INA219::read_shunt_voltage() {
    int16_t value = (int16_t)read_register(INA219_REG_SHUNTVOLTAGE);
    return value * 0.01;
}

float INA219::read_current() {
    int16_t value = (int16_t)read_register(INA219_REG_CURRENT);
    return value * _current_LSB;
}

float INA219::read_power() {
    uint16_t value = read_register(INA219_REG_POWER);
    return value * 0.02;
}
//
// Created by elect on 3/17/2023.
//
