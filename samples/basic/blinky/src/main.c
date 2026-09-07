/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "alif_uart_check.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	alif_uart_check_init();
	alif_uart_check_print();
	alif_uart_baud_sweep();
	alif_uart_pin_la_burst();
	alif_uart_ext_loopback();
	alif_uart_throughput();
	alif_lpuart_all();
	/* USB-COM script: 115200 PASS only. 2.5M/3.75M never see ECHO.
	 * High baud is P8_1↔P8_0. Leave uart_echo_test.py unused.
	 * alif_uart_echo_wire();
	 */

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		alif_uart_check_line();
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
