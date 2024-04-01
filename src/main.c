// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"

#include "bt_hid.h"

#include "dht20-pico/DHT20.h"

DHT20 sens;
DHT20 *sens_ptr = &sens;

void sensor_init() {
	i2c_init(i2c0, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

int main() {

	stdio_init_all();

	sensor_init();

	printf("Initialize DHT20.\n");
    int sensor_ret = DHT20_init(sens_ptr);
    if (sensor_ret != DHT20_OK)
    {
        printf("Failed to initialize the sensor.\n");
        printf("Sensor return value %d\n", sensor_ret);
        return sensor_ret;
    }
    printf("Initialized DHT20.\n");

	int ret = 0;
    uint32_t count = 1;
    printf("Starting the temperature, humidity fetch loop.\n");
    while (true)
    {
        ret = getMeasurement(sens_ptr);
        if (ret != DHT20_OK)
        {
            printf("Measurement %d failed with error value %d\n", count, ret);
            printf("Trying again after 10s...\n");
        }
        else
        {
            printf("Measurements: \n");
            printf("--- Temperature: %5.2f C°", getTemperature(sens_ptr));
            printf("--- Humidity: %5.2f \%RH\n", getHumidity(sens_ptr));
        }
        count++;
        sleep_ms(1000);
    }

}