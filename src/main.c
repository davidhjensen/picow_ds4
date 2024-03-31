// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "bt_hid.h"

#define MOTOR_PWM_PIN 18
#define MOTOR_LIMITTER 80.0 // capped at <>% of full power (100% duty)
#define MOTOR_EN_1 16
#define MOTOR_EN_2 17

#define MAX_JOY_Y 128.0
#define JOY_Y_CENTER 128.0
#define JOY_Y_BUFFER 20 // necessary deviation from center to activate motor

uint motor_slice;

uint motor_dir_status = 0; // 0: stopped; 1: forward; 2: backward

void motor_init() {
	// motor pwm setup
	gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
	motor_slice = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
	// clk div down to 1kHz (period is 31250 counts) (clock is 125x10^6/4)
	pwm_set_wrap(motor_slice, 31250); 
	pwm_set_clkdiv(motor_slice, 4);
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
	uint16_t val = (int) 31250 * (abs(joy_in-JOY_Y_CENTER) / MAX_JOY_Y) * MOTOR_LIMITTER / 100;
	pwm_set_gpio_level(MOTOR_PWM_PIN, val);
	
	printf("Motor PWM: (%d) %d (%.2f%%) EN1: %d EN2: %d STATUS: %d\n", joy_in, val, 100.0*val/31250, gpio_get_out_level(MOTOR_EN_1), gpio_get_out_level(MOTOR_EN_2), dir_stat);

	return dir_stat;
}

int main() {
	stdio_init_all();

	motor_init();

	sleep_ms(1000);
	printf("Hello\n");

	while(1) {
		sleep_ms(1000);
		printf("%c", 12);
		motor_dir_status = joystickY2MotorPwm(100, motor_dir_status);
		sleep_ms(1000);
		printf("%c", 12);
		motor_dir_status = joystickY2MotorPwm(-100, motor_dir_status);
	}
}