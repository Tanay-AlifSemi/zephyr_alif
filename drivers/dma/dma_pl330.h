/*
 * Copyright 2020 Broadcom
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DMA_PL330_H
#define DMA_PL330_H

#include <zephyr/drivers/dma.h>

#define DT_DRV_COMPAT arm_dma_pl330
/*
 * Max burst length and max burst size for 32bit system with
 * 128bit bus width for memory to memory data transfer
 *
 * Burst length is encoded in following format for pl330
 * b0000 = 1 data transfer
 * b0001 = 2 data transfers
 * b0010 = 3 data transfers
 * .
 * .
 * b1111 = 16 data transfers
 *
 * Burst size is encoded in following format for pl330
 * b000 = 1 byte
 * b001 = 2 bytes
 * b010 = 4 bytes
 * b011 = 8 bytes
 * b100 = 16 bytes
 * b101 = 32 bytes
 * b110 = 64 bytes
 * b111 = 128 bytes.
 */
#define MAX_BURST_LEN		0xf /* 16byte data */
#define MAX_BURST_SIZE_LOG2	4

/*
 * PL330 works only on 4GB boundary.
 * PL330 has 32bit registers for source and destination addresses
 */
#define PL330_MAX_OFFSET	0x100000000

/* PL330 supports max 16MB dma based on AXI bus size */
#define PL330_MAX_DMA_SIZE	0x1000000

/* Maximum possible values for PL330 ucode loop counters */
#define PL330_LOOP_COUNTER0_MAX	0x100
#define PL330_LOOP_COUNTER1_MAX	0x100

#define DMAC_PL330_INTEN	0x20
#define DMAC_PL330_INTMIS	0x28
#define DMAC_PL330_INTCLR	0x2C
#define DMAC_PL330_FSRD		0x30
#define DMAC_PL330_FSRC		0x34
#define DMAC_PL330_FTRD		0x38
#define DMAC_PL330_FTR0		0x40
#define DMAC_PL330_CS0		0x100
#define DMAC_PL330_DBGSTATUS	0xd00
#define DMAC_PL330_DBGCMD	0xd04
#define DMAC_PL330_DBGINST0	0xd08
#define DMAC_PL330_DBGINST1	0xd0c
#define DMAC_PL330_CR0		0xE00
#define DMAC_PL330_CRD		0xE14

/*
 * TIMEOUT value of 100000us is kept to cover all possible data
 * transfer sizes, with lesser time out value(10us) DMA channel
 * appears to be busy on FPGA/Emul environment. Ideally 100000us
 * timeout value should never hit.
 */
#define DMA_TIMEOUT_US		100000

#define CH_STATUS_MASK		0xf
#define DATA_MASK		0xf

#define DMA_INTSR1_SHIFT	24
#define DMA_INTSR0_SHIFT	16
#define DMA_INTSR0		0xa0
#define DMA_SECURE_SHIFT	17
#define DMA_CH_SHIFT		8
#define DMA_DBG_CHN			1

#define CONTROL_OFFSET		0x4
#define HIGHER_32_ADDR_MASK	0x0f
#define DST_ADDR_SHIFT		0x4

#define DMA_NUM_PERIPH_REQ_SHIFT	12
#define DMA_NUM_PERIPH_REQ_MASK		0x1F
#define DMA_AXI_DATA_WIDTH_MASK		0x7

#define DMA_MAX_EVENTS		32

#define MICROCODE_SIZE_MAX	0x400
#define GET_MAX_DMA_SIZE(byte_width, burst_len) \
		(PL330_LOOP_COUNTER0_MAX * PL330_LOOP_COUNTER1_MAX * \
		(byte_width) * ((burst_len) + 1))

#define CC_SRCINC_SHIFT		0
#define CC_DSTINC_SHIFT		14
#define CC_SRCPRI_SHIFT		8
#define CC_DSTPRI_SHIFT		22
#define CC_DSTNS_SHIFT		23
#define CC_SRCBRSTLEN_SHIFT	4
#define CC_DSTBRSTLEN_SHIFT	18
#define CC_BRSTLEN_MASK		0xF
#define CC_SRCBRSTSIZE_SHIFT	1
#define CC_DSTBRSTSIZE_SHIFT	15
#define CC_SRCCCTRL_SHIFT	11
#define CC_SRCCCTRL_MASK	0x7
#define CC_DSTCCTRL_SHIFT	25
#define CC_DRCCCTRL_MASK	0x7
#define CC_SWAP_SHIFT		28
#define SRC_PRI_NONSEC_VALUE	0x2
#define SRC_PRI_SEC_VALUE	0x0
#define CC_CCTRL_MODIFIABLE_VALUE	0x2

#define OP_DMA_MOV		0xbc
#define OP_DMA_LOOP_COUNT1	0x22
#define OP_DMA_LOOP		0x20
#define OP_DMA_LD(req)	(0x4 | req)
#define OP_DMA_LDP(req)	(0x25 | req)
#define OP_DMA_ST(req)	(0x8 | req)
#define OP_DMA_STP(req)	(0x29 | req)
#define OP_DMA_SEV		0x34
#define OP_DMA_FLUSHP	0x35
#define OP_DMA_WMB		0x13
#define OP_DMA_END		0x00
#define OP_DMA_KILL		0x01
#define OP_DMA_LP_BK_JMP1	0x38
#define OP_DMA_LP_BK_JMP2	0x3c
#define OP_DMA_WFP(req)		(0x30 | req)
#define SZ_CMD_DMAMOV		0x6

#define DMA_PERIPH_REQ_TYPE_SINGLE	0
#define DMA_PERIPH_REQ_TYPE_BURST	2
#define DMA_PERIPH_REQ_TYPE_PERIPH	1

#define DMA_LDST_REQ_TYPE_FORCE		0
#define DMA_LDST_REQ_TYPE_SINGLE	1
#define DMA_LDST_REQ_TYPE_BURST		3

enum dmamov_type {
	/* Source Address Register */
	SAR = 0,
	/* Channel Control Register */
	CCR,
	/* Destination Address Register */
	DAR,
};

/* Channel specific private data */
struct dma_pl330_ch_internal {
	uint64_t src_addr;
	uint64_t dst_addr;
	int src_burst_sz;
	uint32_t src_burst_len;
	int dst_burst_sz;
	uint32_t dst_burst_len;
	uint32_t trans_size;
	uint32_t dst_id;
	uint32_t src_id;
	uint32_t perip_type;
	uint32_t breq_only;
	uint32_t src_cache_ctrl;
	uint32_t dst_cache_ctrl;
	uint32_t dst_inc;
	uint32_t src_inc;
	int nonsec_mode;
};

struct dma_pl330_ch_config {
	/* Channel configuration details */
	uint64_t src_addr;
	enum dma_addr_adj src_addr_adj;
	uint64_t dst_addr;
	enum dma_addr_adj dst_addr_adj;
	uint16_t src_burst_sz;
	uint16_t dst_burst_sz;
	uint16_t src_blen;
	uint16_t dst_blen;
	enum dma_channel_direction direction;
	uint32_t trans_size;
	void *user_data;
	dma_callback_t dma_callback;
	mem_addr_t dma_exec_addr;
	atomic_t channel_is_active;
	uint8_t periph_slot;

	/* Channel specific private data */
	struct dma_pl330_ch_internal internal;
};

struct dma_pl330_config {
	mem_addr_t mcode_base;
	mem_addr_t reg_base;
#ifdef CONFIG_DMA_64BIT
	mem_addr_t control_reg_base;
#endif
	uint8_t max_dma_channels;

	uint8_t num_irqs;
	void (*irq_configure)(const struct device *dev);
};

struct dma_pl330_dev_data {
	struct dma_pl330_ch_config *channels;
	int event_irq[DMA_MAX_EVENTS];
	uint8_t num_periph_req;
	uint8_t axi_data_width;
};

#endif
