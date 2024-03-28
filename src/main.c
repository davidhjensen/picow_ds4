// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "bt_hid.h"

#define MOTOR_PWM_PIN 0
#define MOTOR_LIMITTER 80.0 // capped at <>% of full power (100% duty)

#define SERVO_PWM_PIN 2
#define SERVO_LIMITTER 15.0 // +- degrees that the servo is capped at; MAX 90

#define MAX_JOY_Y 128.0
#define MAX_JOY_X 128.0

uint servo_slice;
uint motor_slice;


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
	// clk div down to 1kHz (period is 31250 counts) (clock is 125x10^6/4)
	pwm_set_wrap(motor_slice, 31250); 
	pwm_set_clkdiv(motor_slice, 4);
	// set to still
	pwm_set_chan_level(motor_slice, 0, 31250/5);
	pwm_set_enabled(motor_slice, true);
}

void joystickY2MotorPwm(int8_t joy_in) {
	uint16_t val = (int) 31250 * (fabs(joy_in) / MAX_JOY_Y) * MOTOR_LIMITTER / 100;
	pwm_set_gpio_level(MOTOR_PWM_PIN, val);
	printf("Motor PWM: %d (%lf%%)\n", val, val/31250);
}

void joystickX2ServoPwm(int8_t joy_in) {
	uint16_t val = (int) 25000 * (.075 + .05 * (SERVO_LIMITTER/90) * (fabs(joy_in) / MAX_JOY_Y));
	pwm_set_gpio_level(MOTOR_PWM_PIN, val);
	printf("Servo PWM: %d (%lf%%)\n", val, val/31250);
}

int main() {
	stdio_init_all();

	motor_init();
	servo_init();

	while(1) {
		sleep_ms(1000);
		joystickX2ServoPwm(120);
		joystickY2MotorPwm(120);
		sleep_ms(1000);
		joystickX2ServoPwm(-50);
		joystickY2MotorPwm(-50);
	}
}

