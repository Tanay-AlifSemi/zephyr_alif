/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ALIF_UART_CHECK_H_
#define ALIF_UART_CHECK_H_

#ifdef CONFIG_SOC_SERIES_B1
void alif_uart_check_init(void);
void alif_uart_check_print(void);
void alif_uart_check_line(void);
void alif_uart_baud_sweep(void);
void alif_uart_pin_la_burst(void);
void alif_uart_ext_loopback(void);
void alif_uart_throughput(void);
void alif_lpuart_all(void);
void alif_uart_echo_wire(void);
#else
static inline void alif_uart_check_init(void) {}
static inline void alif_uart_check_print(void) {}
static inline void alif_uart_check_line(void) {}
static inline void alif_uart_baud_sweep(void) {}
static inline void alif_uart_pin_la_burst(void) {}
static inline void alif_uart_ext_loopback(void) {}
static inline void alif_uart_throughput(void) {}
static inline void alif_lpuart_all(void) {}
static inline void alif_uart_echo_wire(void) {}
#endif

#endif /* ALIF_UART_CHECK_H_ */
