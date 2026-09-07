/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Spark turbo UART check for blinky (B1 HE). Uses uart_ns16550 only:
 *   uart_configure / uart_config_get
 *   uart_poll_in / uart_poll_out
 *   uart_line_ctrl_set(UART_LINE_CTRL_LOOPBACK)  — MCR LOOP in the driver
 *
 * Console stays UART2 @ 115200. DUT is UART0 (P8_1 TX, P8_0 RX).
 *
 * Driver baud (clocks, no clock-frequency):
 *   DL  = SCLK / (16 * baud)
 *   DLF = round(SCLK / baud) & 0xF
 *   160 MHz / 40 MHz PCLK → 115200 is DL=21 DLF=11
 *   240 MHz / 60 MHz PCLK → 115200 is DL=32 DLF=9
 */

#include "alif_uart_check.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#if !DT_HAS_CHOSEN(zephyr_console)
#error "B1 blinky UART check needs zephyr,console"
#endif

#define CONSOLE_NODE DT_CHOSEN(zephyr_console)
#define UART_BAUD    DT_PROP_OR(CONSOLE_NODE, current_speed, 115200)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
#define DUT_NODE DT_NODELABEL(uart0)
#define DUT_OK   1
#else
#define DUT_OK   0
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart), okay)
#define LPU_NODE DT_NODELABEL(lpuart)
#define LPU_OK   1
#else
#define LPU_OK   0
#endif

#define SWEEP_PAYLOAD   64
#define PIN_BURST_BYTES 32
#define PIN_BURST_REPS  3
#define BENCH_BYTES     1024U
#define BENCH_ITERS     10U

struct uart_check {
	uint32_t sclk;
	uint32_t baud_req;
	uint32_t exp_dl;
	uint32_t exp_dlf;
	uint32_t baud_actual;
	uint32_t drv_baud;
};

static struct uart_check chk;

enum sweep_status {
	SWEEP_PASS = 0,
	SWEEP_FAIL,
	SWEEP_SKIP,
};

struct sweep_row {
	uint32_t baud;
	uint32_t dl;
	uint32_t dlf;
	uint32_t actual;
	int32_t ppm;
	enum sweep_status status;
	int err;
};

static void formula_for_baud(uint32_t sclk, uint32_t baud, uint32_t *dl,
			     uint32_t *dlf, uint32_t *actual, int32_t *ppm)
{
	*dl = (baud != 0U && sclk != 0U) ? (sclk / (baud * 16U)) : 0U;
	*dlf = (baud != 0U && sclk != 0U) ?
	       ((sclk + (baud >> 1)) / baud) & 0xFU : 0U;

	if (*dl == 0U || (16U * (*dl) + *dlf) == 0U) {
		*actual = 0U;
		*ppm = 0;
		return;
	}

	*actual = sclk / (16U * (*dl) + *dlf);
	*ppm = ((int32_t)*actual - (int32_t)baud) * 1000000 / (int32_t)baud;
}

static struct uart_config cfg_8n1(uint32_t baud)
{
	struct uart_config cfg = {
		.baudrate = baud,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	return cfg;
}

void alif_uart_check_init(void)
{
	const struct device *clkdev = DEVICE_DT_GET(DT_CLOCKS_CTLR(CONSOLE_NODE));
	clock_control_subsys_t subsys =
		(clock_control_subsys_t)DT_CLOCKS_CELL(CONSOLE_NODE, clkid);
	const struct device *console = DEVICE_DT_GET(CONSOLE_NODE);
	struct uart_config cfg;
	int32_t ppm;

	chk.baud_req = UART_BAUD;
	chk.sclk = 0;
	chk.drv_baud = 0;

	if (device_is_ready(clkdev) &&
	    clock_control_get_rate(clkdev, subsys, &chk.sclk) != 0) {
		chk.sclk = 0;
	}

	formula_for_baud(chk.sclk, chk.baud_req, &chk.exp_dl, &chk.exp_dlf,
			 &chk.baud_actual, &ppm);

	if (device_is_ready(console) && uart_config_get(console, &cfg) == 0) {
		chk.drv_baud = cfg.baudrate;
	}
}

void alif_uart_check_print(void)
{
	int32_t err_ppm = 0;

	if (chk.baud_req != 0U && chk.baud_actual != 0U) {
		err_ppm = ((int32_t)chk.baud_actual - (int32_t)chk.baud_req) *
			  1000000 / (int32_t)chk.baud_req;
	}

	printk("\n======== UART spark turbo check (ns16550 driver) ========\n");
	printk("HE / SysTick : %u Hz%s\n",
	       CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
	       IS_ENABLED(CONFIG_ALIF_SPARK_TURBO_MODE) ?
		       "  [turbo 240]" : "  [normal 160]");
	printk("UART         : %s (console) + uart0 DUT\n",
	       DEVICE_DT_NAME(CONSOLE_NODE));
	printk("SCLK (PCLK)  : %u Hz\n", chk.sclk);
	printk("requested    : %u baud\n", chk.baud_req);
	printk("uart_config  : %u baud\n", chk.drv_baud);
	printk("driver DL/DLF: %u / %u  (same formula as uart_ns16550)\n",
	       chk.exp_dl, chk.exp_dlf);
	printk("actual baud  : %u  (error %d ppm)\n", chk.baud_actual, err_ppm);
	if (chk.drv_baud == chk.baud_req) {
		printk("result       : uart_config MATCH  (host must stay 115200)\n");
	} else {
		printk("result       : uart_config MISMATCH — check uart_configure\n");
	}
	printk("==========================================================\n\n");
}

void alif_uart_check_line(void)
{
	printk("UART %s  SCLK=%u  DL=%u DLF=%u  baud=%u  %s\n",
	       IS_ENABLED(CONFIG_ALIF_SPARK_TURBO_MODE) ? "turbo240" : "normal160",
	       chk.sclk, chk.exp_dl, chk.exp_dlf, chk.baud_actual,
	       (chk.drv_baud == chk.baud_req) ? "MATCH" : "MISMATCH");
}

static void print_sweep_table(const struct sweep_row *rows, size_t n,
			      uint32_t max_hw, uint32_t max_pass,
			      const char *note)
{
	printk("\r\n");
	printk("+----------+-----+-----+----------+--------+------------------+\r\n");
	printk("|     baud |    DL | DLF |   actual |    ppm | result           |\r\n");
	printk("+----------+-----+-----+----------+--------+------------------+\r\n");

	for (size_t i = 0; i < n; i++) {
		const struct sweep_row *r = &rows[i];

		if (r->status == SWEEP_SKIP) {
			printk("| %8u |     - |   - |        - |      - | SKIP (DL=0)     |\r\n",
			       r->baud);
		} else if (r->status == SWEEP_PASS) {
			printk("| %8u | %5u | %3u | %8u | %6d | PASS             |\r\n",
			       r->baud, r->dl, r->dlf, r->actual, r->ppm);
		} else {
			printk("| %8u | %5u | %3u | %8u | %6d | FAIL (%d)         |\r\n",
			       r->baud, r->dl, r->dlf, r->actual, r->ppm, r->err);
		}
	}

	printk("+----------+-----+-----+----------+--------+------------------+\r\n");
	printk("MAX loopback PASS : %u baud\r\n", max_pass);
	printk("SCLK/16 theor max : %u baud\r\n", max_hw);
	if (note != NULL && note[0] != '\0') {
		printk("%s\r\n", note);
	}
	printk("================================================\r\n\r\n");
}

#if DUT_OK || LPU_OK
static void dut_rx_drain(const struct device *dev)
{
	unsigned char c;
	int guard = 64;

	while (guard-- > 0 && uart_poll_in(dev, &c) == 0) {
	}
}

static bool poll_in_wait(const struct device *dev, uint8_t *c)
{
	uint32_t start = k_cycle_get_32();
	uint32_t limit = sys_clock_hw_cycles_per_sec() / 50U;

	while (uart_poll_in(dev, c) != 0) {
		if ((k_cycle_get_32() - start) > limit) {
			return false;
		}
	}
	return true;
}

static int dut_set_baud(const struct device *dev, uint32_t baud)
{
	struct uart_config cfg = cfg_8n1(baud);

	return uart_configure(dev, &cfg);
}

/* TX one byte, wait for its echo (MCR loop or P8 jumper). */
static int dut_poll_xfer(const struct device *dev, const uint8_t *tx,
			 uint8_t *rx, size_t n)
{
	dut_rx_drain(dev);

	for (size_t i = 0; i < n; i++) {
		uart_poll_out(dev, tx[i]);
		if (!poll_in_wait(dev, &rx[i])) {
			return -2;
		}
	}

	return memcmp(tx, rx, n) == 0 ? 0 : -3;
}

static K_SEM_DEFINE(dut_irq_sem, 0, 1);

static struct {
	const uint8_t *tx;
	uint8_t *rx;
	size_t n;
	volatile size_t tx_i;
	volatile size_t rx_i;
	volatile uint32_t rx_empty;
} irq_xf;

static void dut_irq_cb(const struct device *dev, void *user)
{
	ARG_UNUSED(user);

	if (uart_irq_update(dev) <= 0) {
		return;
	}

	if (uart_irq_tx_ready(dev) && irq_xf.tx_i < irq_xf.n) {
		int got;

		got = uart_fifo_fill(dev, &irq_xf.tx[irq_xf.tx_i],
				     (int)(irq_xf.n - irq_xf.tx_i));
		if (got > 0) {
			irq_xf.tx_i += (size_t)got;
		}
		if (irq_xf.tx_i >= irq_xf.n) {
			uart_irq_tx_disable(dev);
		}
	}

	if (uart_irq_rx_ready(dev) && irq_xf.rx_i < irq_xf.n) {
		int got;

		got = uart_fifo_read(dev, &irq_xf.rx[irq_xf.rx_i],
				     (int)(irq_xf.n - irq_xf.rx_i));
		if (got > 0) {
			irq_xf.rx_i += (size_t)got;
			irq_xf.rx_empty = 0;
		} else {
			irq_xf.rx_empty++;
			/* Analog/noise on RX (e.g. P2_0 LPUART_RX_A): IIR
			 * reports RXRDY/CTI but LSR.DR is 0, so RBR is never
			 * read and the IRQ never clears. Drop IER so the
			 * thread can time out instead of spinning in ISR.
			 */
			if (irq_xf.rx_empty >= 8U) {
				uart_irq_rx_disable(dev);
				uart_irq_tx_disable(dev);
			}
		}
		if (irq_xf.rx_i >= irq_xf.n) {
			uart_irq_rx_disable(dev);
			k_sem_give(&dut_irq_sem);
		}
	}
}

/* Same path as alif/tests/drivers/uart: IRQ + fifo_fill / fifo_read. */
static int dut_irq_xfer(const struct device *dev, const uint8_t *tx,
			uint8_t *rx, size_t n, uint32_t baud)
{
	int64_t to_ms;
	int ret;

	dut_rx_drain(dev);
	irq_xf.tx = tx;
	irq_xf.rx = rx;
	irq_xf.n = n;
	irq_xf.tx_i = 0;
	irq_xf.rx_i = 0;
	irq_xf.rx_empty = 0;
	k_sem_reset(&dut_irq_sem);

	ret = uart_irq_callback_user_data_set(dev, dut_irq_cb, NULL);
	if (ret != 0) {
		return ret;
	}

	uart_irq_rx_enable(dev);
	uart_irq_tx_enable(dev);

	to_ms = ((int64_t)n * 30000) / (int64_t)baud + 200;
	if (to_ms < 200) {
		to_ms = 200;
	}

	if (k_sem_take(&dut_irq_sem, K_MSEC(to_ms)) != 0) {
		uart_irq_tx_disable(dev);
		uart_irq_rx_disable(dev);
		return -2;
	}

	uart_irq_tx_disable(dev);
	uart_irq_rx_disable(dev);

	return memcmp(tx, rx, n) == 0 ? 0 : -3;
}

static int dut_pattern_xfer(const struct device *dev)
{
	uint8_t tx[SWEEP_PAYLOAD];
	uint8_t rx[SWEEP_PAYLOAD];

	for (int i = 0; i < SWEEP_PAYLOAD; i++) {
		tx[i] = (uint8_t)(0xA5U ^ (uint8_t)i);
	}

	return dut_poll_xfer(dev, tx, rx, SWEEP_PAYLOAD);
}

static void fill_sweep_row(struct sweep_row *r, uint32_t baud, uint32_t sclk)
{
	memset(r, 0, sizeof(*r));
	r->baud = baud;
	formula_for_baud(sclk, baud, &r->dl, &r->dlf, &r->actual, &r->ppm);
	if (r->dl == 0U) {
		r->status = SWEEP_SKIP;
	}
}

static const uint32_t sweep_bauds[] = {
	9600, 115200, 230400, 460800, 921600,
	1500000, 2000000, 2500000,
	3000000, 3750000, 4000000,
};

/* LPUART SCLK = HE (160/240 MHz) → max SCLK/16 = 10 / 15 Mbps. */
static const uint32_t lpu_sweep_bauds[] = {
	9600, 115200, 230400, 460800, 921600,
	1500000, 2000000, 2500000,
	3000000, 3750000, 4000000,
	5000000, 10000000, 15000000,
};

static const uint32_t lpu_la_bauds[] = {
	115200, 2500000, 3750000, 10000000, 15000000,
};
static const uint32_t lpu_tp_bauds[] = {
	115200, 2500000, 3750000, 10000000, 15000000,
};

#define LPU_LA_BYTES 4
#define LPU_LA_REPS  1

#if LPU_OK
static uint32_t lpuart_sclk(void)
{
	const struct device *clkdev =
		DEVICE_DT_GET(DT_CLOCKS_CTLR(LPU_NODE));
	clock_control_subsys_t subsys =
		(clock_control_subsys_t)DT_CLOCKS_CELL(LPU_NODE, clkid);
	uint32_t rate = 0;

	if (device_is_ready(clkdev)) {
		(void)clock_control_on(clkdev, subsys);
		(void)clock_control_get_rate(clkdev, subsys, &rate);
	}
	return rate;
}
#endif
#endif

void alif_uart_baud_sweep(void)
{
#if !DUT_OK
	printk("UART0 not enabled — skip MCR loopback sweep\r\n");
#else
	struct sweep_row rows[ARRAY_SIZE(sweep_bauds)];
	uint32_t max_hw = chk.sclk / 16U;
	uint32_t max_pass = 0;
	const struct device *dut = DEVICE_DT_GET(DUT_NODE);

	if (!device_is_ready(dut)) {
		printk("UART0 not ready — skip MCR loopback sweep\r\n");
		return;
	}

	printk("======== UART0 baud sweep (driver MCR loopback) ========\r\n");
	printk("uart_configure + uart_line_ctrl_set(LOOPBACK). "
	       "Console stays UART2.\r\n");

	for (size_t i = 0; i < ARRAY_SIZE(sweep_bauds); i++) {
		struct sweep_row *r = &rows[i];
		int ret;

		fill_sweep_row(r, sweep_bauds[i], chk.sclk);
		if (r->status == SWEEP_SKIP) {
			continue;
		}

		ret = dut_set_baud(dut, r->baud);
		if (ret != 0) {
			r->err = ret;
			r->status = SWEEP_FAIL;
			continue;
		}

		ret = uart_line_ctrl_set(dut, UART_LINE_CTRL_LOOPBACK, 1);
		if (ret != 0) {
			r->err = ret;
			r->status = SWEEP_FAIL;
			continue;
		}

		r->err = dut_pattern_xfer(dut);
		r->status = (r->err == 0) ? SWEEP_PASS : SWEEP_FAIL;
		if (r->status == SWEEP_PASS) {
			max_pass = r->baud;
		}
	}

	(void)uart_line_ctrl_set(dut, UART_LINE_CTRL_LOOPBACK, 0);
	(void)dut_set_baud(dut, 115200);

	print_sweep_table(rows, ARRAY_SIZE(sweep_bauds), max_hw, max_pass,
			  "Note: MCR loopback via uart_ns16550 line_ctrl. Pin not proven here.");
#endif
}

void alif_uart_pin_la_burst(void)
{
#if !DUT_OK
	printk("UART0 not enabled — skip pin TX burst\r\n");
#else
	static const uint32_t bauds[] = { 115200, 2500000, 3750000 };
	const struct device *dut = DEVICE_DT_GET(DUT_NODE);
	bool sent[ARRAY_SIZE(bauds)] = { false };

	if (!device_is_ready(dut)) {
		printk("UART0 not ready — skip pin TX burst\r\n");
		return;
	}

	printk("======== UART0 TX-only burst (logic analyzer) ========\r\n");
	printk("uart_configure + uart_poll_out(0x55). Console UART2 P5_3.\r\n");
	printk("Probe LA on UART0 TX = P8_1\r\n");
	printk("Pattern: 0x55 x %u x %u reps\r\n", PIN_BURST_BYTES, PIN_BURST_REPS);
	printk("Arm LA on P8_1 falling edge. Bursts in 3 s...\r\n");
	printk("  115200  -> bit ~8.68 us   (both modes)\r\n");
	printk("  2500000 -> bit 400 ns     (fair max)\r\n");
	printk("  3750000 -> bit 267 ns     (turbo only)\r\n");
	k_msleep(3000);

	(void)uart_line_ctrl_set(dut, UART_LINE_CTRL_LOOPBACK, 0);

	for (size_t i = 0; i < ARRAY_SIZE(bauds); i++) {
		uint32_t dl, dlf, actual;
		int32_t ppm;

		formula_for_baud(chk.sclk, bauds[i], &dl, &dlf, &actual, &ppm);
		if (dl == 0U) {
			printk(" SKIP baud: %u (DL=0)\r\n", bauds[i]);
			continue;
		}

		if (dut_set_baud(dut, bauds[i]) != 0) {
			printk("  uart_configure(%u) failed\r\n", bauds[i]);
			continue;
		}

		printk("  sending %u baud on P8_1 ...\r\n", bauds[i]);
		for (int r = 0; r < PIN_BURST_REPS; r++) {
			for (int b = 0; b < PIN_BURST_BYTES; b++) {
				uart_poll_out(dut, 0x55);
			}
			k_busy_wait(200);
		}
		sent[i] = true;
		k_msleep(50);
	}

	(void)dut_set_baud(dut, 115200);

	printk("LA bursts done on P8_1:\r\n");
	for (size_t i = 0; i < ARRAY_SIZE(bauds); i++) {
		printk("  %7u baud : %s\r\n", bauds[i],
		       sent[i] ? "sent on P8_1" : "SKIP (DL=0)");
	}
	printk("Measure bit width on P8_1. Do not use UART decode.\r\n");
	printk("=====================================================\r\n\r\n");
#endif
}

void alif_uart_ext_loopback(void)
{
#if !DUT_OK
	printk("UART0 not enabled — skip external loopback\r\n");
#else
	struct sweep_row rows[ARRAY_SIZE(sweep_bauds)];
	uint32_t max_hw = chk.sclk / 16U;
	uint32_t max_pass = 0;
	const struct device *dut = DEVICE_DT_GET(DUT_NODE);

	if (!device_is_ready(dut)) {
		printk("UART0 not ready — skip external loopback\r\n");
		return;
	}

	printk("======== UART0 external loopback (wire, ns16550) ========\r\n");
	printk("Jumper: P8_1 (UART0 TX) --- P8_0 (UART0 RX)\r\n");
	printk("uart_configure + uart_poll_out/in. LOOPBACK off.\r\n");
	printk("Waiting 5 s for the jumper...\r\n");
	k_msleep(5000);

	(void)uart_line_ctrl_set(dut, UART_LINE_CTRL_LOOPBACK, 0);

	for (size_t i = 0; i < ARRAY_SIZE(sweep_bauds); i++) {
		struct sweep_row *r = &rows[i];
		int ret;

		fill_sweep_row(r, sweep_bauds[i], chk.sclk);
		if (r->status == SWEEP_SKIP) {
			continue;
		}

		ret = dut_set_baud(dut, r->baud);
		if (ret != 0) {
			r->err = ret;
			r->status = SWEEP_FAIL;
			continue;
		}

		r->err = dut_pattern_xfer(dut);
		r->status = (r->err == 0) ? SWEEP_PASS : SWEEP_FAIL;
		if (r->status == SWEEP_PASS) {
			max_pass = r->baud;
		}
	}

	(void)dut_set_baud(dut, 115200);

	printk("External loopback (P8_1->P8_0), %u-byte pattern\r\n",
	       SWEEP_PAYLOAD);
	printk("FAIL (-2) = no RX (jumper missing). FAIL (-3) = data mismatch.\r\n");
	print_sweep_table(rows, ARRAY_SIZE(sweep_bauds), max_hw, max_pass,
			  "Note: TX left P8_1 and came back on P8_0 (uart_ns16550).");
#endif
}

void alif_uart_throughput(void)
{
#if !DUT_OK
	printk("UART0 not enabled — skip throughput\r\n");
#else
	static const uint32_t bauds[] = { 115200, 2500000, 3750000 };
	static uint8_t tx[BENCH_BYTES];
	static uint8_t rx[BENCH_BYTES];
	const struct device *dut = DEVICE_DT_GET(DUT_NODE);
	uint32_t hz = sys_clock_hw_cycles_per_sec();

	if (!device_is_ready(dut)) {
		printk("UART0 not ready — skip throughput\r\n");
		return;
	}

	for (uint32_t i = 0; i < BENCH_BYTES; i++) {
		tx[i] = (uint8_t)i;
	}

	printk("======== UART0 throughput (P8_1->P8_0, ns16550 IRQ) ========\r\n");
	printk("uart_fifo_fill / uart_fifo_read in ISR. "
	       "k_cycle_get_32() around one xfer.\r\n");
	printk("%u B x %u (1 warmup dropped). 8N1 = 10 bits/byte.\r\n",
	       BENCH_BYTES, BENCH_ITERS);
	printk("Fair: 115200 and 2.5M. 3.75M turbo only.\r\n");

	(void)uart_line_ctrl_set(dut, UART_LINE_CTRL_LOOPBACK, 0);

	printk("\r\n");
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");
	printk("|     baud |   actual | result |     cycles |       us |   kB/s |   Mbit/s |\r\n");
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");

	for (size_t i = 0; i < ARRAY_SIZE(bauds); i++) {
		uint32_t baud = bauds[i];
		uint32_t dl, dlf, actual;
		int32_t ppm;
		uint64_t cyc_sum = 0;
		uint32_t scored = 0;
		int err = 0;

		formula_for_baud(chk.sclk, baud, &dl, &dlf, &actual, &ppm);
		if (dl == 0U) {
			printk("| %8u |        - | SKIP   |          - |        - |      - |        - |\r\n",
			       baud);
			continue;
		}

		if (dut_set_baud(dut, baud) != 0) {
			printk("| %8u | %8u | FAIL   |          - |        - |      - |        - |\r\n",
			       baud, actual);
			continue;
		}

		memset(rx, 0, BENCH_BYTES);
		err = dut_irq_xfer(dut, tx, rx, BENCH_BYTES, baud);
		if (err != 0) {
			printk("| %8u | %8u | FAIL(%d)|          - |        - |      - |        - |\r\n",
			       baud, actual, err);
			continue;
		}

		for (uint32_t n = 0; n < BENCH_ITERS; n++) {
			uint32_t t0, t1;

			memset(rx, 0, BENCH_BYTES);
			t0 = k_cycle_get_32();
			err = dut_irq_xfer(dut, tx, rx, BENCH_BYTES, baud);
			t1 = k_cycle_get_32();
			if (err != 0) {
				break;
			}
			cyc_sum += (uint32_t)(t1 - t0);
			scored++;
		}

		if (err != 0 || scored == 0U) {
			printk("| %8u | %8u | FAIL(%d)|          - |        - |      - |        - |\r\n",
			       baud, actual, err);
			continue;
		}

		{
			uint32_t bytes = BENCH_BYTES * scored;
			uint32_t us = (hz != 0U) ?
				(uint32_t)(cyc_sum * 1000000ULL / hz) : 0U;
			uint32_t wire_us = (actual != 0U) ?
				(uint32_t)((uint64_t)bytes * 10ULL * 1000000ULL /
					   actual) : 0U;
			uint32_t kBps = (us != 0U) ?
				(uint32_t)(bytes * 1000ULL / us) : 0U;
			uint32_t kbit = (us != 0U) ?
				(uint32_t)(bytes * 8000ULL / us) : 0U;

			printk("| %8u | %8u | PASS   | %10u | %8u | %6u | %4u.%u%u |\r\n",
			       baud, actual, (uint32_t)cyc_sum, us, kBps,
			       kbit / 1000U, (kbit / 100U) % 10U, (kbit / 10U) % 10U);
			printk("    scored %u x %u B  wire_us=%u  (8N1, one way)\r\n",
			       scored, BENCH_BYTES, wire_us);
		}
	}

	(void)dut_set_baud(dut, 115200);
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");
	printk("IRQ + FIFO path (same APIs as alif/tests/drivers/uart).\r\n");
	printk("================================================\r\n\r\n");
#endif
}

void alif_lpuart_all(void)
{
#if !LPU_OK
	printk("LPUART not enabled — skip (add &lpuart { status = \"okay\"; })\r\n");
#else
	const struct device *dut = DEVICE_DT_GET(LPU_NODE);
	uint32_t sclk = lpuart_sclk();
	uint32_t max_hw = sclk / 16U;
	uint32_t hz = sys_clock_hw_cycles_per_sec();
	struct sweep_row rows[ARRAY_SIZE(lpu_sweep_bauds)];
	uint32_t max_pass = 0;
	static uint8_t tx[BENCH_BYTES];
	static uint8_t rx[BENCH_BYTES];

	if (!device_is_ready(dut)) {
		printk("LPUART not ready — skip\r\n");
		return;
	}

	printk("======== LPUART check (ns16550, SCLK=HE) ========\r\n");
	printk("HE / SysTick : %u Hz%s\r\n",
	       CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
	       IS_ENABLED(CONFIG_ALIF_SPARK_TURBO_MODE) ?
		       "  [turbo 240]" : "  [normal 160]");
	printk("LPUART       : %s @ 0x43008000\r\n", DEVICE_DT_NAME(LPU_NODE));
	printk("SCLK (HE)    : %u Hz  (not PCLK)\r\n", sclk);
	printk("theor max    : %u baud (SCLK/16)\r\n", max_hw);
	printk("Pins         : P7_1 TX_B, P7_0 RX_B  (same group B)\r\n");
	printk("Console stays UART2 P5_3 @ 115200.\r\n");
	printk("================================================\r\n\r\n");

	printk("======== LPUART baud sweep (driver MCR loopback) ========\r\n");
	for (size_t i = 0; i < ARRAY_SIZE(lpu_sweep_bauds); i++) {
		struct sweep_row *r = &rows[i];
		int ret;

		fill_sweep_row(r, lpu_sweep_bauds[i], sclk);
		if (r->status == SWEEP_SKIP) {
			continue;
		}
		ret = dut_set_baud(dut, r->baud);
		if (ret != 0) {
			r->err = ret;
			r->status = SWEEP_FAIL;
			continue;
		}
		ret = uart_line_ctrl_set(dut, UART_LINE_CTRL_LOOPBACK, 1);
		if (ret != 0) {
			r->err = ret;
			r->status = SWEEP_FAIL;
			continue;
		}
		r->err = dut_pattern_xfer(dut);
		r->status = (r->err == 0) ? SWEEP_PASS : SWEEP_FAIL;
		if (r->status == SWEEP_PASS) {
			max_pass = r->baud;
		}
	}
	(void)dut_set_baud(dut, 115200);
	print_sweep_table(rows, ARRAY_SIZE(lpu_sweep_bauds), max_hw, max_pass,
			  "LPUART MCR loopback. SCLK=HE.");

	printk("======== LPUART TX-only burst (logic analyzer) ========\r\n");
	printk("Probe LA on P7_1. 0x55 x %u x %u. LOOP off.\r\n",
	       LPU_LA_BYTES, LPU_LA_REPS);
	printk("Leave P7_1--P7_0 jumper OFF until the wire section.\r\n");
	printk("  115200   -> bit ~8.68 us\r\n");
	printk("  2500000  -> bit 400 ns\r\n");
	printk("  10000000 -> bit 100 ns\r\n");
	printk("  15000000 -> bit 67 ns (turbo 240)\r\n");
	(void)uart_line_ctrl_set(dut, UART_LINE_CTRL_LOOPBACK, 0);
	uart_irq_rx_disable(dut);
	uart_irq_tx_disable(dut);
	for (size_t i = 0; i < ARRAY_SIZE(lpu_la_bauds); i++) {
		uint32_t dl, dlf, actual;
		int32_t ppm;
		unsigned char dump;

		formula_for_baud(sclk, lpu_la_bauds[i], &dl, &dlf, &actual, &ppm);
		if (dl == 0U) {
			printk(" SKIP baud: %u (DL=0)\r\n", lpu_la_bauds[i]);
			continue;
		}
		if (dut_set_baud(dut, lpu_la_bauds[i]) != 0) {
			printk("  uart_configure(%u) failed\r\n", lpu_la_bauds[i]);
			continue;
		}
		dut_rx_drain(dut);
		printk("  sending %u baud on P7_1 (%u B) ...\r\n",
		       lpu_la_bauds[i], LPU_LA_BYTES);
		for (int r = 0; r < LPU_LA_REPS; r++) {
			for (int b = 0; b < LPU_LA_BYTES; b++) {
				uart_poll_out(dut, 0x55);
				/* Jumper on: echo fills RX. Drain so overrun
				 * cannot stall THRE (that was the TAD9 hang).
				 */
				k_busy_wait(50);
				while (uart_poll_in(dut, &dump) == 0) {
				}
			}
		}
		printk("  P7_1 %u done\r\n", lpu_la_bauds[i]);
	}
	(void)dut_set_baud(dut, 115200);
	printk("LA bursts done on P7_1. Measure bit width, no UART decode.\r\n");
	printk("=====================================================\r\n\r\n");

	/* --- wire P7_1 TX_B -> P7_0 RX_B (IRQ + timeout) --- */
	printk("======== LPUART external loopback (wire, IRQ) ========\r\n");
	printk("NOW jumper P7_1 (TX) --- P7_0 (RX). Waiting 5 s...\r\n");
	printk("FAIL(-2) = no RX (jumper off). That is not a hang.\r\n");
	k_msleep(5000);
	(void)uart_line_ctrl_set(dut, UART_LINE_CTRL_LOOPBACK, 0);
	max_pass = 0;
	memset(rows, 0, sizeof(rows));
	for (size_t i = 0; i < ARRAY_SIZE(lpu_sweep_bauds); i++) {
		struct sweep_row *r = &rows[i];
		int ret;
		uint8_t ptx[SWEEP_PAYLOAD];
		uint8_t prx[SWEEP_PAYLOAD];

		fill_sweep_row(r, lpu_sweep_bauds[i], sclk);
		if (r->status == SWEEP_SKIP) {
			continue;
		}
		ret = dut_set_baud(dut, r->baud);
		if (ret != 0) {
			r->err = ret;
			r->status = SWEEP_FAIL;
			continue;
		}
		for (int j = 0; j < SWEEP_PAYLOAD; j++) {
			ptx[j] = (uint8_t)(0xA5U ^ (uint8_t)j);
		}
		r->err = dut_irq_xfer(dut, ptx, prx, SWEEP_PAYLOAD, r->baud);
		r->status = (r->err == 0) ? SWEEP_PASS : SWEEP_FAIL;
		if (r->status == SWEEP_PASS) {
			max_pass = r->baud;
		}
	}
	(void)dut_set_baud(dut, 115200);
	print_sweep_table(rows, ARRAY_SIZE(lpu_sweep_bauds), max_hw, max_pass,
			  "LPUART TX left P7_1 and came back on P7_0.");

	/* --- IRQ throughput --- */
	for (uint32_t i = 0; i < BENCH_BYTES; i++) {
		tx[i] = (uint8_t)i;
	}
	printk("======== LPUART throughput (P7_1->P7_0, ns16550 IRQ) ========\r\n");
	printk("SCLK=HE. Fair vs UART0: 115200 and 2.5M. "
	       "LPUART: 10M both, 15M turbo.\r\n");
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");
	printk("|     baud |   actual | result |     cycles |       us |   kB/s |   Mbit/s |\r\n");
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");
	for (size_t i = 0; i < ARRAY_SIZE(lpu_tp_bauds); i++) {
		uint32_t baud = lpu_tp_bauds[i];
		uint32_t dl, dlf, actual;
		int32_t ppm;
		uint64_t cyc_sum = 0;
		uint32_t scored = 0;
		int err = 0;

		formula_for_baud(sclk, baud, &dl, &dlf, &actual, &ppm);
		if (dl == 0U) {
			printk("| %8u |        - | SKIP   |          - |        - |      - |        - |\r\n",
			       baud);
			continue;
		}
		if (dut_set_baud(dut, baud) != 0) {
			printk("| %8u | %8u | FAIL   |          - |        - |      - |        - |\r\n",
			       baud, actual);
			continue;
		}
		memset(rx, 0, BENCH_BYTES);
		err = dut_irq_xfer(dut, tx, rx, BENCH_BYTES, baud);
		if (err != 0) {
			printk("| %8u | %8u | FAIL(%d)|          - |        - |      - |        - |\r\n",
			       baud, actual, err);
			continue;
		}
		for (uint32_t n = 0; n < BENCH_ITERS; n++) {
			uint32_t t0, t1;

			memset(rx, 0, BENCH_BYTES);
			t0 = k_cycle_get_32();
			err = dut_irq_xfer(dut, tx, rx, BENCH_BYTES, baud);
			t1 = k_cycle_get_32();
			if (err != 0) {
				break;
			}
			cyc_sum += (uint32_t)(t1 - t0);
			scored++;
		}
		if (err != 0 || scored == 0U) {
			printk("| %8u | %8u | FAIL(%d)|          - |        - |      - |        - |\r\n",
			       baud, actual, err);
			continue;
		}
		{
			uint32_t bytes = BENCH_BYTES * scored;
			uint32_t us = (hz != 0U) ?
				(uint32_t)(cyc_sum * 1000000ULL / hz) : 0U;
			uint32_t wire_us = (actual != 0U) ?
				(uint32_t)((uint64_t)bytes * 10ULL * 1000000ULL /
					   actual) : 0U;
			uint32_t kBps = (us != 0U) ?
				(uint32_t)(bytes * 1000ULL / us) : 0U;
			uint32_t kbit = (us != 0U) ?
				(uint32_t)(bytes * 8000ULL / us) : 0U;

			printk("| %8u | %8u | PASS   | %10u | %8u | %6u | %4u.%u%u |\r\n",
			       baud, actual, (uint32_t)cyc_sum, us, kBps,
			       kbit / 1000U, (kbit / 100U) % 10U, (kbit / 10U) % 10U);
			printk("    scored %u x %u B  wire_us=%u  (8N1, one way)\r\n",
			       scored, BENCH_BYTES, wire_us);
		}
	}
	(void)dut_set_baud(dut, 115200);
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");
	printk("================================================\r\n\r\n");
#endif
}

void alif_uart_echo_wire(void)
{
	const struct device *uart = DEVICE_DT_GET(CONSOLE_NODE);
	uint32_t baud = CONFIG_ALIF_UART_ECHO_BAUD;
	uint32_t dl = (chk.sclk != 0U) ? (chk.sclk / (baud * 16U)) : 0U;
	struct uart_config cfg = cfg_8n1(baud);
	int ret;

	if (!device_is_ready(uart)) {
		printk("echo: UART2 not ready\r\n");
		return;
	}

	if (dl == 0U) {
		printk("echo: %u baud SKIP (DL=0). Stay 115200.\r\n", baud);
		baud = 115200;
		cfg.baudrate = baud;
	}

	printk("======== UART2 wire echo (uart_poll_*) ========\r\n");
	printk("Close the 115200 terminal, then uart_echo_test.py --baud %u\r\n",
	       baud);
	printk("============================================\r\n");
	k_msleep(2000);

	ret = uart_configure(uart, &cfg);
	if (ret != 0) {
		printk("echo: uart_configure(%u) failed %d\r\n", baud, ret);
		return;
	}

	uart_irq_rx_disable(uart);
	uart_irq_tx_disable(uart);

	for (int i = 0; i < 3; i++) {
		const char *s = "ECHO\r\n";

		while (*s) {
			uart_poll_out(uart, *s++);
		}
		k_msleep(50);
	}

	for (;;) {
		unsigned char c;

		if (uart_poll_in(uart, &c) == 0) {
			uart_poll_out(uart, c);
		}
	}
}
