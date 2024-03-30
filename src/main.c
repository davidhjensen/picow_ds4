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
#define SERVO_VOLTAGE_PIN 6 // output PWM to step ~12V down to ~6V

#define MAX_JOY_Y 128.0
#define JOY_Y_CENTER 128.0

#define MAX_JOY_X 128.0
#define JOY_X_CENTER 128.0


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

void servo_voltage_reg() {
	// servo pwm setup 
	gpio_set_function(SERVO_VOLTAGE_PIN, GPIO_FUNC_PWM);
	servo_voltage_slice = pwm_gpio_to_slice_num(SERVO_VOLTAGE_PIN);
	// clk div down to 20kHz
	pwm_set_wrap(servo_voltage_slice, 6250); 
	// set to 5.088V = 12V * (2650 / 6250)
	pwm_set_chan_level(servo_voltage_slice, 0, 2650);
	pwm_set_enabled(servo_voltage_slice, true);
}

void joystickY2MotorPwm(int8_t joy_in) {
	uint16_t val = (int) 31250 * (joy_in-JOY_Y_CENTER) / MAX_JOY_Y) * MOTOR_LIMITTER / 100;
	//pwm_set_gpio_level(MOTOR_PWM_PIN, val);
	printf("Motor PWM: %d (%.2f%%)\n", val, 100.0*val/31250);
}

void joystickX2ServoPwm(int8_t joy_in) {
	uint16_t val = (int) 25000 * (.075 + .05 * (SERVO_LIMITTER/90) * (joy_in-JOY_X_CENTER) / MAX_JOY_Y);
	//pwm_set_gpio_level(MOTOR_PWM_PIN, val);
	printf("Servo PWM: %d (%.2f%%)\n", val, 100.0*val/25000);
}

int main() {
	stdio_init_all();

	motor_init();
	servo_voltage_reg();
	servo_init();

	sleep_ms(1000);
	printf("Hello\n");

	multicore_launch_core1(bt_main);
	// Wait for init (should do a handshake with the fifo here?)
	sleep_ms(1000);

	struct bt_hid_state state;

	for ( ;; ) {
		sleep_ms(500);
		bt_hid_get_latest(&state);
		char buffer[100];
		//printf(buffer, "buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d\n",
		//		state.buttons, state.lx, state.ly, state.rx, state.ry,
		//		state.l2, state.r2, state.hat);
		joystickY2MotorPwm(state.rx);
		joystickX2ServoPwm(state.ly);
	}
	/*
	while(1) {
		sleep_ms(1000);
		printf("%c", 12);
		joystickX2ServoPwm(120);
		joystickY2MotorPwm(120);
		sleep_ms(1000);
		printf("%c", 12);
		joystickX2ServoPwm(-50);
		joystickY2MotorPwm(-50);
	}
	*/
}