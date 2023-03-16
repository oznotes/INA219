#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "math.h"

#define INA219_I2C_ADDR 0x40
#define INA219_REG_CONFIG 0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

float global_current_LSB;


void i2c_init() {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(8, GPIO_FUNC_I2C);
    gpio_set_function(9, GPIO_FUNC_I2C);
    gpio_pull_up(8);
    gpio_pull_up(9);
}

uint16_t read_register(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(i2c0, INA219_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, INA219_I2C_ADDR, buf, 2, false);

    return (buf[0] << 8) | buf[1];
}

void write_register(uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;

    i2c_write_blocking(i2c0, INA219_I2C_ADDR, buf, 3, false);
}

void ina219_init() {
    uint16_t config = 0x399F;
    write_register(INA219_REG_CONFIG, config);
    sleep_ms(10); // Add this line

}

void ina219_calibrate(float shunt_resistor_value, float max_expected_amps) {
    float current_LSB = max_expected_amps / 32768.0;
    uint16_t cal_reg_value = (uint16_t)(0.04096 / (current_LSB * shunt_resistor_value));
    write_register(INA219_REG_CALIBRATION, cal_reg_value);
    global_current_LSB = current_LSB;
}


float ina219_read_voltage() {
    uint16_t value = read_register(INA219_REG_BUSVOLTAGE);
    return (value >> 3) * 0.004;
}

float ina219_read_shunt_voltage() {
    int16_t value = (int16_t) read_register(INA219_REG_SHUNTVOLTAGE);
    return value * 0.01;
}

float ina219_read_current() {
    int16_t value = (int16_t) read_register(INA219_REG_CURRENT);
    return value * global_current_LSB;
}

float ina219_read_power() {
    uint16_t value = read_register(INA219_REG_POWER);
    return value * 0.02; // Assuming the default calibration value of 4096
}

int main() {
    stdio_init_all();
    i2c_init();
    ina219_init();
    ina219_calibrate(0.1, 3.2); // Calibrate for 0.1 Ohm shunt resistor and 3.2A max expected current

    while (1) {
        float voltage = ina219_read_voltage();
        //float shunt_voltage = ina219_read_shunt_voltage();
        float shunt_voltage = fabs(ina219_read_shunt_voltage());

        //float current = ina219_read_current();
        float current = fabs(ina219_read_current());

        float power = ina219_read_power();

        printf("Voltage: %.4f V\n", voltage);
        printf("Shunt Voltage: %.4f mV\n", shunt_voltage);
        printf("Current: %.4f A\n", current);
        printf("Power: %.4f W\n", power);
        printf("--------------------\n");

        sleep_ms(1000);
    }
}
