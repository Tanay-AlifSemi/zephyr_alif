/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Spark turbo UART check for blinky (B1 HE console = UART2).
 *
 * Expected 115200 (driver formula, SCLK = SYST_PCLK = core/4):
 *   160 MHz / 40 MHz PCLK → DL=21  DLF=11
 *   240 MHz / 60 MHz PCLK → DL=32  DLF=9
 *
 * Host terminal must stay 115200 in both modes. If turbo is 1.5x too
 * fast (bit time ~5.76 us), DL/DLF were not retuned.
 */

#include "alif_uart_check.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>

#if !DT_HAS_CHOSEN(zephyr_console)
#error "B1 blinky UART check needs zephyr,console"
#endif

#define CONSOLE_NODE DT_CHOSEN(zephyr_console)
#define UART_BASE    DT_REG_ADDR(CONSOLE_NODE)
#define UART_SHIFT   DT_PROP(CONSOLE_NODE, reg_shift)
#define UART_BAUD    DT_PROP_OR(CONSOLE_NODE, current_speed, 115200)

#define REG_THR  0x00
#define REG_RDR  0x00
#define REG_BRDL 0x00
#define REG_BRDH 0x01
#define REG_LCR  0x03
#define REG_MCR  0x04
#define REG_LSR  0x05
#define REG_DLF  0xC0
#define LCR_DLAB 0x80
#define MCR_LOOP 0x10
#define LSR_RXRDY 0x01
#define LSR_THRE  0x20
#define LSR_TEMT  0x40

#define SWEEP_PAYLOAD 64
#define UART_IRQ      DT_IRQN(CONSOLE_NODE)

struct uart_check {
	uint32_t sclk;
	uint32_t baud_req;
	uint32_t hw_dl;
	uint32_t hw_dlf;
	uint32_t exp_dl;
	uint32_t exp_dlf;
	uint32_t baud_actual;
};

static struct uart_check chk;

static inline uint32_t uart_reg(uint32_t offset)
{
	return UART_BASE + (offset << UART_SHIFT);
}

static void read_divisor_hw(uint32_t *dl, uint32_t *dlf)
{
	unsigned int key;
	uint8_t lcr;

	*dlf = sys_read8(UART_BASE + REG_DLF);

	key = irq_lock();
	lcr = sys_read8(uart_reg(REG_LCR));
	sys_write8(lcr | LCR_DLAB, uart_reg(REG_LCR));
	*dl = sys_read8(uart_reg(REG_BRDL)) |
	      ((uint32_t)sys_read8(uart_reg(REG_BRDH)) << 8);
	sys_write8(lcr, uart_reg(REG_LCR));
	irq_unlock(key);
}

void alif_uart_check_init(void)
{
	const struct device *clkdev = DEVICE_DT_GET(DT_CLOCKS_CTLR(CONSOLE_NODE));
	clock_control_subsys_t subsys =
		(clock_control_subsys_t)DT_CLOCKS_CELL(CONSOLE_NODE, clkid);

	chk.baud_req = UART_BAUD;
	chk.sclk = 0;

	if (device_is_ready(clkdev) &&
	    clock_control_get_rate(clkdev, subsys, &chk.sclk) != 0) {
		chk.sclk = 0;
	}

	if (chk.sclk != 0U && chk.baud_req != 0U) {
		chk.exp_dl = chk.sclk / (chk.baud_req * 16U);
		chk.exp_dlf = ((chk.sclk + (chk.baud_req >> 1)) / chk.baud_req) & 0xFU;
	}

	read_divisor_hw(&chk.hw_dl, &chk.hw_dlf);

	if (chk.sclk != 0U && (16U * chk.hw_dl + chk.hw_dlf) != 0U) {
		chk.baud_actual = chk.sclk / (16U * chk.hw_dl + chk.hw_dlf);
	}
}

void alif_uart_check_print(void)
{
	int32_t err_ppm = 0;

	if (chk.baud_req != 0U && chk.baud_actual != 0U) {
		err_ppm = ((int32_t)chk.baud_actual - (int32_t)chk.baud_req) *
			  1000000 / (int32_t)chk.baud_req;
	}

	printk("\n======== UART spark turbo check (console) ========\n");
	printk("HE / SysTick : %u Hz%s\n",
	       CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
	       IS_ENABLED(CONFIG_ALIF_SPARK_TURBO_MODE) ?
		       "  [turbo 240]" : "  [normal 160]");
	printk("UART         : %s @ 0x%08x\n",
	       DEVICE_DT_NAME(CONSOLE_NODE), (uint32_t)UART_BASE);
	printk("SCLK (PCLK)  : %u Hz\n", chk.sclk);
	printk("requested    : %u baud\n", chk.baud_req);
	printk("expected     : DL=%u  DLF=%u\n", chk.exp_dl, chk.exp_dlf);
	printk("hardware     : DL=%u  DLF=%u\n", chk.hw_dl, chk.hw_dlf);
	printk("actual baud  : %u  (error %d ppm)\n", chk.baud_actual, err_ppm);
	if (chk.hw_dl == chk.exp_dl && chk.hw_dlf == chk.exp_dlf) {
		printk("result       : DL/DLF MATCH  (host must stay 115200)\n");
	} else {
		printk("result       : DL/DLF MISMATCH — baud will look 1.5x if stale\n");
	}
	printk("==================================================\n\n");
}

void alif_uart_check_line(void)
{
	printk("UART %s  SCLK=%u  DL=%u DLF=%u  baud=%u  %s\n",
	       IS_ENABLED(CONFIG_ALIF_SPARK_TURBO_MODE) ? "turbo240" : "normal160",
	       chk.sclk, chk.hw_dl, chk.hw_dlf, chk.baud_actual,
	       (chk.hw_dl == chk.exp_dl && chk.hw_dlf == chk.exp_dlf) ?
		       "MATCH" : "MISMATCH");
}

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

static void write_divisor(uint32_t dl, uint32_t dlf)
{
	unsigned int key;
	uint8_t lcr;

	sys_write8((uint8_t)dlf, UART_BASE + REG_DLF);

	key = irq_lock();
	lcr = sys_read8(uart_reg(REG_LCR));
	sys_write8(lcr | LCR_DLAB, uart_reg(REG_LCR));
	sys_write8((uint8_t)(dl & 0xffU), uart_reg(REG_BRDL));
	sys_write8((uint8_t)((dl >> 8) & 0xffU), uart_reg(REG_BRDH));
	sys_write8(lcr & (uint8_t)~LCR_DLAB, uart_reg(REG_LCR));
	irq_unlock(key);
}

static bool lsr_wait(uint8_t mask)
{
	uint32_t start = k_cycle_get_32();
	/* 20 ms per byte is enough down to 9600 8N1 */
	uint32_t limit = sys_clock_hw_cycles_per_sec() / 50U;

	while ((sys_read8(uart_reg(REG_LSR)) & mask) == 0U) {
		if ((k_cycle_get_32() - start) > limit) {
			return false;
		}
	}
	return true;
}

static void uart_rx_drain(void)
{
	int guard = 64;

	while (guard-- > 0 && (sys_read8(uart_reg(REG_LSR)) & LSR_RXRDY) != 0U) {
		(void)sys_read8(uart_reg(REG_RDR));
	}
}

/* Console printk must finish before we retune the same UART. */
static void uart_wait_idle(void)
{
	(void)lsr_wait(LSR_TEMT);
	uart_rx_drain();
}

static int loopback_xfer(void)
{
	uint8_t tx[SWEEP_PAYLOAD];
	uint8_t rx[SWEEP_PAYLOAD];
	uint8_t mcr;

	for (int i = 0; i < SWEEP_PAYLOAD; i++) {
		tx[i] = (uint8_t)(0xA5U ^ (uint8_t)i);
	}

	uart_wait_idle();
	mcr = sys_read8(uart_reg(REG_MCR));
	sys_write8(mcr | MCR_LOOP, uart_reg(REG_MCR));
	uart_rx_drain();

	for (int i = 0; i < SWEEP_PAYLOAD; i++) {
		if (!lsr_wait(LSR_THRE)) {
			sys_write8(mcr, uart_reg(REG_MCR));
			return -1;
		}
		sys_write8(tx[i], uart_reg(REG_THR));
		if (!lsr_wait(LSR_RXRDY)) {
			sys_write8(mcr, uart_reg(REG_MCR));
			return -2;
		}
		rx[i] = sys_read8(uart_reg(REG_RDR));
	}

	(void)lsr_wait(LSR_TEMT);
	sys_write8(mcr, uart_reg(REG_MCR));
	uart_rx_drain();

	return memcmp(tx, rx, SWEEP_PAYLOAD) == 0 ? 0 : -3;
}

static void print_sweep_table(const struct sweep_row *rows, size_t n,
			      uint32_t max_hw, uint32_t max_pass,
			      const char *note)
{
	printk("\r\n");
	printk("+----------+-----+-----+----------+--------+------------------+\r\n");
	printk("|     baud |  DL | DLF |   actual |    ppm | result           |\r\n");
	printk("+----------+-----+-----+----------+--------+------------------+\r\n");

	for (size_t i = 0; i < n; i++) {
		const struct sweep_row *r = &rows[i];

		if (r->status == SWEEP_SKIP) {
			printk("| %8u |   - |   - |        - |      - | SKIP (DL=0)     |\r\n",
			       r->baud);
		} else if (r->status == SWEEP_PASS) {
			printk("| %8u | %3u | %3u | %8u | %6d | PASS             |\r\n",
			       r->baud, r->dl, r->dlf, r->actual, r->ppm);
		} else {
			printk("| %8u | %3u | %3u | %8u | %6d | FAIL (%d)         |\r\n",
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

void alif_uart_baud_sweep(void)
{
	static const uint32_t bauds[] = {
		9600, 115200, 230400, 460800, 921600,
		1500000, 2000000, 2500000,
		3000000, 3750000, 4000000,
	};
	struct sweep_row rows[ARRAY_SIZE(bauds)];
	uint32_t max_hw = chk.sclk / 16U;
	uint32_t max_pass = 0;
	uint8_t mcr_save;

	printk("======== UART baud sweep (MCR loopback) ========\r\n");
	printk("Collecting %u rates, console silent until table...\r\n",
	       (uint32_t)ARRAY_SIZE(bauds));
	uart_wait_idle();

	mcr_save = sys_read8(uart_reg(REG_MCR));
	irq_disable(UART_IRQ);

	for (size_t i = 0; i < ARRAY_SIZE(bauds); i++) {
		struct sweep_row *r = &rows[i];

		memset(r, 0, sizeof(*r));
		r->baud = bauds[i];
		r->dl = chk.sclk / (r->baud * 16U);
		r->dlf = ((chk.sclk + (r->baud >> 1)) / r->baud) & 0xFU;

		if (r->dl == 0U) {
			r->status = SWEEP_SKIP;
			continue;
		}

		r->actual = chk.sclk / (16U * r->dl + r->dlf);
		r->ppm = ((int32_t)r->actual - (int32_t)r->baud) * 1000000 /
			 (int32_t)r->baud;

		write_divisor(r->dl, r->dlf);
		r->err = loopback_xfer();
		r->status = (r->err == 0) ? SWEEP_PASS : SWEEP_FAIL;
		if (r->status == SWEEP_PASS) {
			max_pass = r->baud;
		}
	}

	write_divisor(chk.hw_dl, chk.hw_dlf);
	sys_write8(mcr_save, uart_reg(REG_MCR));
	uart_wait_idle();
	irq_enable(UART_IRQ);

	print_sweep_table(rows, ARRAY_SIZE(bauds), max_hw, max_pass,
			  "Note: MCR loopback is on-chip. Pin/LA not proven here.");
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
#define DUT_NODE  DT_NODELABEL(uart0)
#define DUT_BASE  DT_REG_ADDR(DUT_NODE)
#define DUT_SHIFT DT_PROP(DUT_NODE, reg_shift)
#define DUT_OK    1
#else
#define DUT_OK    0
#endif

#define PIN_BURST_BYTES 32
#define PIN_BURST_REPS  3

#if DUT_OK
static inline uint32_t dut_reg(uint32_t offset)
{
	return DUT_BASE + (offset << DUT_SHIFT);
}

static bool dut_lsr_wait(uint8_t mask)
{
	uint32_t start = k_cycle_get_32();
	uint32_t limit = sys_clock_hw_cycles_per_sec() / 50U;

	while ((sys_read8(dut_reg(REG_LSR)) & mask) == 0U) {
		if ((k_cycle_get_32() - start) > limit) {
			return false;
		}
	}
	return true;
}

static void dut_write_divisor(uint32_t dl, uint32_t dlf)
{
	unsigned int key;
	uint8_t lcr;

	sys_write8((uint8_t)dlf, DUT_BASE + REG_DLF);

	key = irq_lock();
	lcr = sys_read8(dut_reg(REG_LCR));
	sys_write8(lcr | LCR_DLAB, dut_reg(REG_LCR));
	sys_write8((uint8_t)(dl & 0xffU), dut_reg(REG_BRDL));
	sys_write8((uint8_t)((dl >> 8) & 0xffU), dut_reg(REG_BRDH));
	sys_write8(lcr & (uint8_t)~LCR_DLAB, dut_reg(REG_LCR));
	irq_unlock(key);
}

static void dut_tx_55_burst(void)
{
	uint8_t mcr = sys_read8(dut_reg(REG_MCR));

	sys_write8(mcr & (uint8_t)~MCR_LOOP, dut_reg(REG_MCR));

	for (int r = 0; r < PIN_BURST_REPS; r++) {
		for (int i = 0; i < PIN_BURST_BYTES; i++) {
			if (!dut_lsr_wait(LSR_THRE)) {
				return;
			}
			sys_write8(0x55, dut_reg(REG_THR));
		}
		(void)dut_lsr_wait(LSR_TEMT);
		k_busy_wait(200);
		//k_msleep(1000);
	}
}
#endif /* DUT_OK */

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
	printk("Console is UART2 P5_3 @ 115200  (terminal — leave it)\r\n");
	printk("Probe LA on UART0 TX = P8_1     (this pin only)\r\n");
	printk("Pattern: 0x55 x %u x %u reps\r\n", PIN_BURST_BYTES, PIN_BURST_REPS);
	printk("Arm LA on P8_1 falling edge. Bursts in 3 s...\r\n");
	printk("  115200  -> bit ~8.68 us   (both modes)\r\n");
	printk("  2500000 -> bit 400 ns     (fair max)\r\n");
	printk("  3750000 -> bit 267 ns     (turbo only)\r\n");
	k_msleep(5000);

	for (size_t i = 0; i < ARRAY_SIZE(bauds); i++) {
		uint32_t baud = bauds[i];
		uint32_t dl = chk.sclk / (baud * 16U);
		uint32_t dlf = ((chk.sclk + (baud >> 1)) / baud) & 0xFU;

		if (dl == 0U) {
			printk(" SKIP baud: %u dl: %u dlf: %u  \r\n", baud, dl, dlf);
			//continue;
		}

		printk("  sending %u baud on P8_1 ...\r\n", baud);
		dut_write_divisor(dl, dlf);
		dut_tx_55_burst();
		sent[i] = true;
		k_msleep(50);
		//k_msleep(5000);
	}

	/* Park UART0 at 115200 so a later probe is quiet */
	dut_write_divisor(chk.exp_dl, chk.exp_dlf);

	printk("LA bursts done on P8_1:\r\n");
	for (size_t i = 0; i < ARRAY_SIZE(bauds); i++) {
		printk("  %7u baud : %s\r\n", bauds[i],
		       sent[i] ? "sent on P8_1" : "SKIP (DL=0)");
	}
	printk("Measure bit width on P8_1. Do not use UART decode.\r\n");
	printk("=====================================================\r\n\r\n");
#endif
}

#if DUT_OK
static void dut_rx_drain(void)
{
	int guard = 64;

	while (guard-- > 0 && (sys_read8(dut_reg(REG_LSR)) & LSR_RXRDY) != 0U) {
		(void)sys_read8(dut_reg(REG_RDR));
	}
}

/* Pin loopback: MCR LOOP off, data must go P8_1 -> wire -> P8_0 */
static int dut_ext_xfer(void)
{
	uint8_t tx[SWEEP_PAYLOAD];
	uint8_t rx[SWEEP_PAYLOAD];
	uint8_t mcr = sys_read8(dut_reg(REG_MCR));

	for (int i = 0; i < SWEEP_PAYLOAD; i++) {
		tx[i] = (uint8_t)(0xA5U ^ (uint8_t)i);
	}

	sys_write8(mcr & (uint8_t)~MCR_LOOP, dut_reg(REG_MCR));
	dut_rx_drain();

	for (int i = 0; i < SWEEP_PAYLOAD; i++) {
		if (!dut_lsr_wait(LSR_THRE)) {
			return -1;
		}
		sys_write8(tx[i], dut_reg(REG_THR));
		if (!dut_lsr_wait(LSR_RXRDY)) {
			return -2;
		}
		rx[i] = sys_read8(dut_reg(REG_RDR));
	}

	(void)dut_lsr_wait(LSR_TEMT);
	return memcmp(tx, rx, SWEEP_PAYLOAD) == 0 ? 0 : -3;
}

#define BENCH_BYTES 1024U
#define BENCH_ITERS 10U

static int dut_ext_xfer_n(uint8_t *tx, uint8_t *rx, size_t n)
{
	uint8_t mcr = sys_read8(dut_reg(REG_MCR));

	sys_write8(mcr & (uint8_t)~MCR_LOOP, dut_reg(REG_MCR));
	dut_rx_drain();

	for (size_t i = 0; i < n; i++) {
		if (!dut_lsr_wait(LSR_THRE)) {
			return -1;
		}
		sys_write8(tx[i], dut_reg(REG_THR));
		if (!dut_lsr_wait(LSR_RXRDY)) {
			return -2;
		}
		rx[i] = sys_read8(dut_reg(REG_RDR));
	}

	(void)dut_lsr_wait(LSR_TEMT);
	return memcmp(tx, rx, n) == 0 ? 0 : -3;
}
#endif

void alif_uart_ext_loopback(void)
{
#if !DUT_OK
	printk("UART0 not enabled — skip external loopback\r\n");
#else
	static const uint32_t bauds[] = {
		9600, 115200, 230400, 460800, 921600,
		1500000, 2000000, 2500000,
		3000000, 3750000, 4000000,
	};
	struct sweep_row rows[ARRAY_SIZE(bauds)];
	uint32_t max_hw = chk.sclk / 16U;
	uint32_t max_pass = 0;
	const struct device *dut = DEVICE_DT_GET(DUT_NODE);

	if (!device_is_ready(dut)) {
		printk("UART0 not ready — skip external loopback\r\n");
		return;
	}

	printk("======== UART0 external loopback (wire) ========\r\n");
	printk("Jumper: P8_1 (UART0 TX) --- P8_0 (UART0 RX)\r\n");
	printk("Console stays UART2 P5_3 @ 115200. Do not jumper P5_3.\r\n");
	printk("Waiting 5 s for the jumper...\r\n");
	k_msleep(5000);

	for (size_t i = 0; i < ARRAY_SIZE(bauds); i++) {
		struct sweep_row *r = &rows[i];

		memset(r, 0, sizeof(*r));
		r->baud = bauds[i];
		r->dl = chk.sclk / (r->baud * 16U);
		r->dlf = ((chk.sclk + (r->baud >> 1)) / r->baud) & 0xFU;

		if (r->dl == 0U) {
			r->status = SWEEP_SKIP;
			continue;
		}

		r->actual = chk.sclk / (16U * r->dl + r->dlf);
		r->ppm = ((int32_t)r->actual - (int32_t)r->baud) * 1000000 /
			 (int32_t)r->baud;

		dut_write_divisor(r->dl, r->dlf);
		r->err = dut_ext_xfer();
		r->status = (r->err == 0) ? SWEEP_PASS : SWEEP_FAIL;
		if (r->status == SWEEP_PASS) {
			max_pass = r->baud;
		}
	}

	dut_write_divisor(chk.exp_dl, chk.exp_dlf);

	printk("External loopback (P8_1->P8_0), %u-byte pattern\r\n",
	       SWEEP_PAYLOAD);
	printk("FAIL (-2) = no RX (jumper missing). FAIL (-3) = data mismatch.\r\n");
	print_sweep_table(rows, ARRAY_SIZE(bauds), max_hw, max_pass,
			  "Note: TX left P8_1 and came back on P8_0.");
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

	printk("======== UART0 throughput (P8_1->P8_0) ========\r\n");
	printk("Same jumper as external loopback. 8N1 = 10 bits/byte.\r\n");
	printk("k_cycle_get_32() around TX+RX poll only. "
	       "%u B x %u (1 warmup dropped).\r\n",
	       BENCH_BYTES, BENCH_ITERS);
	printk("Fair rates: 115200 and 2.5M both modes. 3.75M turbo only.\r\n");

	printk("\r\n");
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");
	printk("|     baud |   actual | result |     cycles |       us |   kB/s |   Mbit/s |\r\n");
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");

	for (size_t i = 0; i < ARRAY_SIZE(bauds); i++) {
		uint32_t baud = bauds[i];
		uint32_t dl = chk.sclk / (baud * 16U);
		uint32_t dlf = ((chk.sclk + (baud >> 1)) / baud) & 0xFU;
		uint32_t actual;
		uint64_t cyc_sum = 0;
		uint32_t scored = 0;
		int err = 0;

		if (dl == 0U) {
			printk("| %8u |        - | SKIP   |          - |        - |      - |        - |\r\n",
			       baud);
			continue;
		}

		actual = chk.sclk / (16U * dl + dlf);
		dut_write_divisor(dl, dlf);

		/* warmup — not timed */
		memset(rx, 0, BENCH_BYTES);
		err = dut_ext_xfer_n(tx, rx, BENCH_BYTES);
		if (err != 0) {
			printk("| %8u | %8u | FAIL(%d)|          - |        - |      - |        - |\r\n",
			       baud, actual, err);
			continue;
		}

		for (uint32_t n = 0; n < BENCH_ITERS; n++) {
			uint32_t t0, t1;

			memset(rx, 0, BENCH_BYTES);
			t0 = k_cycle_get_32();
			err = dut_ext_xfer_n(tx, rx, BENCH_BYTES);
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

	dut_write_divisor(chk.exp_dl, chk.exp_dlf);
	printk("+----------+----------+--------+------------+----------+--------+----------+\r\n");
	printk("Mbit/s is payload (8 data bits). Wire bits = 10/byte so "
	       "ideal payload = baud*0.8.\r\n");
	printk("Same baud: expect ~same us, cycles ~1.5x on turbo (HE clock).\r\n");
	printk("================================================\r\n\r\n");
#endif
}

/*
 * UART2 (USB-COM) becomes a wire echo so a PC script can TX and check RX.
 * Console IRQ is turned off so printk does not steal RX bytes.
 */
void alif_uart_echo_wire(void)
{
	const struct device *uart = DEVICE_DT_GET(CONSOLE_NODE);
	uint32_t baud = CONFIG_ALIF_UART_ECHO_BAUD;
	uint32_t dl = (chk.sclk != 0U) ? (chk.sclk / (baud * 16U)) : 0U;
	struct uart_config cfg = {
		.baudrate = baud,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	int ret;

	if (!device_is_ready(uart)) {
		printk("echo: UART2 not ready\r\n");
		return;
	}

	if (dl == 0U) {
		printk("echo: %u baud SKIP (DL=0, above SCLK/16=%u). "
		       "Use 2500000 on normal, or turbo for 3000000+.\r\n",
		       baud, chk.sclk / 16U);
		baud = 115200;
		cfg.baudrate = baud;
	}

	printk("======== UART2 wire echo (PC TX/RX) ========\r\n");
	printk("Close the 115200 terminal, then:\r\n");
	printk("  python3 samples/basic/blinky/tools/uart_echo_test.py "
	       "--port /dev/ttyUSB2 --baud %u --dtr off --iterations 0\r\n",
	       baud);
	printk("Firmware will send ECHO then loop back every byte.\r\n");
	printk("============================================\r\n");
	k_msleep(2000);

	ret = uart_configure(uart, &cfg);
	if (ret != 0) {
		printk("echo: uart_configure(%u) failed %d — stay 115200\r\n",
		       baud, ret);
		return;
	}

	uart_irq_rx_disable(uart);
	uart_irq_tx_disable(uart);
	irq_disable(UART_IRQ);

	/* Banner then echo immediately — PC must ping-sync before scoring. */
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
