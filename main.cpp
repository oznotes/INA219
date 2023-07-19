#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "INA219_driver/INA219.h"

void core1_entry() {
    // i2c bus, i2c address for sensor
    INA219 ina219(i2c0, 0x40);
    //SDA pin 8, SCL pin 9, 400 kHz speed
    ina219.I2C_START(8, 9, 400);
    // Calibrate for 0.1 Ohm shunt resistor and 3.2A max expected current
    ina219.calibrate(0.1, 3.2);

    while (1) {
        float voltage = ina219.read_voltage();
        float shunt_voltage = fabs(ina219.read_shunt_voltage());
        float current = fabs(ina219.read_current());
        float power = ina219.read_power();

        printf("Voltage: %.4f V\n", voltage);
        printf("Shunt Voltage: %.4f mV\n", shunt_voltage);
        printf("Current: %.4f A\n", current);
        printf("Power: %.4f W\n", power);
        printf("--------------------\n");

        sleep_ms(1000);
    }
}

int main() {
    stdio_init_all();
    multicore_launch_core1(core1_entry);

    while (1) {
        tight_loop_contents();
    }

    return 0;
}
