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


#define MOTOR_PWM_PIN 18
#define MOTOR_EN_1 16
#define MOTOR_EN_2 17

#define SERVO_PWM_PIN 0
#define SERVO_VOLTAGE_PIN 6 // output PWM to step ~12V down to ~6V

#define MAX_JOY_Y 128.0
#define JOY_Y_CENTER 128.0
#define JOY_Y_BUFFER 20 // necessary deviation from center to activate motor

#define MAX_JOY_X 128.0
#define JOY_X_CENTER 128.0


uint servo_slice;
uint servo_voltage_slice;
uint motor_slice;

uint motor_limmitter = 80; // capped at <>% of full power (100% duty)
uint servo_limmitter = 15; // +- degrees that the servo is capped at; MAX 90

double servo_center = .075; // set servo center to 7.5%

uint motor_dir_status = 0; // 0: stopped; 1: forward; 2: backward

DHT20 sens;
DHT20 *sens_ptr = &sens;

void sensor_init() {
	i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void servo_init() {
	// servo pwm setup 
	gpio_set_function(SERVO_PWM_PIN, GPIO_FUNC_PWM);
	servo_slice = pwm_gpio_to_slice_num(SERVO_PWM_PIN);
	// clk div down to 50 Hz (period is 25000 counts) (clock is 125x10^6/100)
	pwm_set_wrap(servo_slice, 25000); 
	pwm_set_clkdiv(servo_slice, 100);
	// set to center at 7.5% duty
	pwm_set_chan_level(servo_slice, 0, 1875);
	pwm_set_enabled(servo_slice, true);
}

void motor_init() {
	// motor pwm setup
	gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
	motor_slice = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
	// clk div down to 500Hz (period is 31250 counts) (clock is 125x10^6/4)
	pwm_set_wrap(motor_slice, 31249); 
	pwm_set_clkdiv(motor_slice, 8);
	// set to still
	pwm_set_chan_level(motor_slice, 0, 0);
	pwm_set_enabled(motor_slice, true);
	// motor direction setup and set to stop
	gpio_init(MOTOR_EN_1);
	gpio_init(MOTOR_EN_2);
	gpio_set_dir(MOTOR_EN_1, GPIO_OUT);
	gpio_set_dir(MOTOR_EN_2, GPIO_OUT);
	gpio_put(MOTOR_EN_1, 0);
	gpio_put(MOTOR_EN_2, 0);
}

uint joystickY2MotorPwm(uint8_t joy_in, uint dir_stat) {
	// if it should be stopped
	if((dir_stat != 0) && (joy_in <= JOY_Y_CENTER + JOY_Y_BUFFER) && (joy_in >= JOY_Y_CENTER - JOY_Y_BUFFER)) {
		gpio_put(MOTOR_EN_1, 0);
		gpio_put(MOTOR_EN_2, 0);
		dir_stat = 0;

	// if it should be forward
	} else if((dir_stat != 1) && (joy_in < JOY_Y_CENTER - JOY_Y_BUFFER)) {
		gpio_put(MOTOR_EN_1, 0);
		gpio_put(MOTOR_EN_2, 1);
		dir_stat = 1;
		
	// if it should be backward
	} else if((dir_stat != 2) && (joy_in > JOY_Y_CENTER + JOY_Y_BUFFER)) {
		gpio_put(MOTOR_EN_1, 1);
		gpio_put(MOTOR_EN_2, 0);
		dir_stat = 2;
	}

	// convert to PWM duty and set
	uint16_t val = (int) 31250 * (abs(joy_in-JOY_Y_CENTER) / MAX_JOY_Y) * motor_limitter / 100;
	pwm_set_gpio_level(MOTOR_PWM_PIN, val);
	
	//printf("Motor PWM: (%d) %d (%.2f%%) EN1: %d EN2: %d STATUS: %d\n", joy_in, val, 100.0*val/31250, gpio_get_out_level(MOTOR_EN_1), gpio_get_out_level(MOTOR_EN_2), dir_stat);

	return dir_stat;
}

void joystickX2ServoPwm(uint8_t joy_in) {
	uint16_t val = (int) 25000 * (servo_center + .05 * (servo_limitter / 90) * (joy_in-JOY_X_CENTER) / MAX_JOY_Y);
	pwm_set_gpio_level(SERVO_PWM_PIN, val);
	//printf("Servo PWM: (%d) %d (%.2f%%)\n", joy_in, val, 100.0*val/25000);
}

int main() {
	stdio_init_all();

	motor_init();
	servo_init();
	sensor_init();

	sleep_ms(5000);
	
	printf("Initialize DHT20.\n");
    int sensor_ret = DHT20_init(sens_ptr);
    if (sensor_ret != DHT20_OK)
    {
        printf("Failed to initialize the sensor.\n");
        printf("Sensor return value %d\n", sensor_ret);
    }
    printf("Initialized DHT20.\n");
	

	sleep_ms(1000);

	multicore_launch_core1(bt_main);
	// Wait for init (should do a handshake with the fifo here?)
	sleep_ms(1000);

	struct bt_hid_state state;

	for ( ;; ) {
		sleep_ms(1000);
		bt_hid_get_latest(&state);
		printf("buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d\n",
				state.buttons, state.lx, state.ly, state.rx, state.ry,
				state.l2, state.r2, state.hat);
		motor_dir_status = joystickY2MotorPwm(state.ly, motor_dir_status);
		joystickX2ServoPwm(state.rx);

		if(state.buttons == 0x2000) {
			int ret = getMeasurement(sens_ptr);
			if (ret != DHT20_OK)
			{
				printf("Measurement failed with error value %d\n", ret);
				printf("Trying again after 10s...\n");
			}
			else
			{
				printf("Measurements: \n");
				printf("--- Temperature: %5.2f C°", getTemperature(sens_ptr));
				printf("--- Humidity: %5.2f \%RH\n", getHumidity(sens_ptr));
			}
		} /*else if (state.buttons == FORWARD ON D PAD && motor_limitter <=98) { // increase max power
			motor_limitter += 2;
		} else if (state.buttons == BACKWARD ON D PAD && motor_limitter >=2) { // decrease max power
			motor_limitter -= 2;
		} else if (state.buttons == LEFT ON D PAD && servo_limitter >= 1) { // incrase turning radius
			servo_limitter += 1;
		} else if (state.buttons == LEFT ON D PAD && servo_limitter >= 1) { // decrease turning radius
			servo_limitter -= 1;
		} else if (state.buttons == L2 && servo_center >= .027) { // trim left
			servo_limitter -= .002;
		} else if (state.buttons == R2 && servo_center <= .123) { // trim right
			servo_limitter += .002;
		} else if (state.buttons == OPTIONS) { // print out current settings
			printf("----------CURRENT SETTINGS----------\n");
			printf("Power: %f%%\nSteering Angle: %.2fdeg\nSteering Trim: %+.2f\n", motor_limitter, servo_limitter, (.075-servo_center)/.05*90)
		}
		*/
	}
}