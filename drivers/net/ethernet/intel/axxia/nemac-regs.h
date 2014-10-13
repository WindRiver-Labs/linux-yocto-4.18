/*
 * Register definitions for Axxia NEMAC Gigabit Ethernet controller
 *
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _NEMAC_REGS_H
#define _NEMAC_REGS_H

/* NEMAC GLOBAL REGISTERS */

#define NEM_VERSION_R			0x0000
#define    MAJOR_REV(_x)		(((_x) >> 24) & 0xff)
#define    MINOR_REV(_x)		(((_x) >> 16) & 0xff)
#define NEM_SCRATCHPAD_R		0x0004
#define NEM_PRESENCE_R			0x0008
#define    PRESENT_GMAC0		(1 << 0)
#define    PRESENT_GMAC0_PFC		(1 << 16)
#define NEM_INT_STATUS_R		0x0010
#define NEM_INT_ENABLE_R		0x0014
#define NEM_INT_FORCE_R			0x0018
#define    INT_GROUP_GMAC0		(1 << 0)
#define    INT_GROUP_STATS		(1 << 14)
#define    INT_GROUP_PTI		(1 << 15)
#define NEM_SWRESET_R			0x0020
#define    SWRESET_GMAC0		(1 << 0)
#define    SWRESET_STATS		(1 << 14)
#define    SWRESET_PTI			(1 << 15)
#define NEM_ACTIVE_R			0x0024
#define    ACTIVE_GMAC0			(1 << 0)
#define NEM_LOS_CONTROL_R		0x0028
#define    FILTER_ENABLE		(1 << 0)
#define    DEBUG_SEL(_x)		(((_x) & 0xf) << 12)	 /* 0..15 */
#define    FILTER_TOV(_x)		(((_x) & 0xffff) << 16)	 /* 0..65535 */
#define NEM_LOS_DEBUG_R			0x002c
#define    LOS_RAW			(1 << 0)
#define    LOS_HIGH_WM(_x)		(((_x) & 0xffff) << 16)	 /* 0..65535 */

/* PTI REGISTERS */

#define NEM_PTI_INT_STATUS_R		0x0100
#define REG_PTI_INT_ENABLE_R		0x0104
#define REG_PTI_INT_FORCE_R		0x0108
#define NEM_PTI_TX_UNDERRUN_STATUS_R	0x010c
#define NEM_PTI_TX_OVERRUN_STATUS_R	0x0110
#define NEM_PTI_RX_OVERRUN_STATUS_R	0x0114
#define    PTI_GMAC0			(1 << 0)
#define NEM_PTI_CONFIG_R		0x0118
#define    GMAC_RX_TS_ENABLE(_x)	(((_x) & 0xfff) << 0)	 /* 0..4095 */
#define    XGMAC_RX_TS_ENABLE		(1 << 12)
#define    TX_TS_COR			(1 << 13)
#define    TS_OUT_FORMAT(_x)		(((_x) & 0x3) << 14)	 /* 0..3 */
#define    PTI_MAC_LOOPBACK		(1 << 16)
#define    TS_MODE			(1 << 17)
#define NEM_PTI_BURST_LENGTH_R		0x011c
#define    BURST_LENGTH(_x)		(((_x) & 0x1f) << 0)	 /* 0..31 */
#define NEM_PTI_GMAC_TX_HIGH_WM_R	0x0120
#define NEM_PTI_GMAC_TX_LOW_WM_R	0x0124
#define NEM_PTI_GMAC_RX_HIGH_WM_R	0x0128
#define NEM_PTI_GMAC_RX_LOW_WM_R	0x012c
#define NEM_PTI_TX_FIFO_STAT_R		0x0160
#define    TX_FIFO_UNDERFLOW		(1 << 0)
#define    TX_FIFO_OVERFLOW		(1 << 1)
#define    TX_FIFO_EMPTY		(1 << 2)
#define    TX_FIFO_AEMPTY		(1 << 3)
#define    TX_FIFO_AFULL		(1 << 4)
#define    TX_FIFO_FULL			(1 << 5)
#define    TX_FIFO_DEPTH_MASK		(0x1fff << 6)
#define NEM_PTI_TX_MAX_USED_R		0x0164
#define NEM_PTI_TX_TRUNCATED_R		0x0168
#define NEM_PTI_TX_OVERRUNS_R		0x016c
#define NEM_PTI_RX_FIFO_STAT_R		0x0170
#define    RX_FIFO_UNDERFLOW		(1 << 0)
#define    RX_FIFO_OVERFLOW		(1 << 1)
#define    RX_FIFO_EMPTY		(1 << 2)
#define    RX_FIFO_AEMPTY		(1 << 3)
#define    RX_FIFO_AFULL		(1 << 4)
#define    RX_FIFO_FULL			(1 << 5)
#define    RX_FIFO_DEPTH_MASK		(0x1fff << 6)
#define NEM_PTI_RX_MAX_USED_R		0x0174
#define NEM_PTI_RX_TRUNCATED_R		0x0178
#define NEM_PTI_RX_DROPPED_R		0x017c

/* GMAC REGISTERS */

#define NEM_GMAC_ENABLE_R		0x0300
#define    MAC_PAUSE_QUANTA(_x)		(((_x) & 0xffff) << 16)
#define    RGMII_SWAP_TX		(1 << 9)
#define    RGMII_SWAP_RX		(1 << 8)
#define    RGMII_MODE			(1 << 7)
#define    GPIO_CLK_MUX_SELECT		(1 << 6)
#define    GMAC_TX_INIT_PAUSE		(1 << 5)
#define    GMII_BYPASS_MODE		(1 << 4)
#define    GMAC_RX_EN			(1 << 3)
#define    PORT_ENABLE			(1 << 2)
#define    GMAC_RX_PAUSE_EN		(1 << 1)
#define    GMAC_TX_PAUSE_EN		(1 << 0)
#define NEM_GMAC_STA_ADDR_R		0x0304
#define NEM_GMAC_STA_ADDR_UPPER_R	0x0308
#define NEM_GMAC_TIMING_CTRL_R		0x030c
#define    LOOPBACK_IFG_MODE		(1 << 6)
#define    SEL_EXT_PAUSE_WAIT_IN	(1 << 5)
#define    INFINITE_RETRY		(1 << 4)
#define    ZERO_BACKOFF			(1 << 3)
#define    FAST_SLOT_TMR_MODE		(1 << 2)
#define    FIXED_MODE			(1 << 1)
#define    SHORT_IFG_MODE		(1 << 0)
#define NEM_GMAC_MAC_CTRL_R		0x0310
#define    WOL_ENABLE			(1 << 10)
#define    TX_PAD_ENABLE		(1 << 6)
#define    RX_CRC_STRIP_ENABLE		(1 << 5)
#define    RX_ABORT_DISABLE		(1 << 4)
#define    GMAC_TX_FLUSH		(1 << 2)
#define    AUTO_FLUSH_MODE		(1 << 1)
#define    GM_DEBUG_SELECT		(1 << 0)
#define NEM_GMAC_MAC_STATUS_R		0x0314
#define    GM_DEBUG(_x)			(((_x) & 0x3ff) << 3)	 /* 0..1023 */
#define    L_FIFO_OVERRUN		(1 << 2)
#define    L_ANY_FIFO_UNDERRUN		(1 << 1)
#define    TX_ENCAP_DONE		(1 << 0)
#define NEM_GMAC_PRBS_ERR_R		0x0318
#define    C6_PRBS_ERR(_x)		(((_x) & 0xff) << 0)	 /* 0..255 */
#define NEM_GMAC_SGMII_ENABLE_R		0x031c
#define    SGMII_C73_INT_DISABLE	(1 << 2)
#define    SGMII_C37_INT_DISABLE	(1 << 1)
#define    SGMII_PORT_EN		(1 << 0)
#define NEM_GMAC_PRBS_CTRL_R		0x0320
#define    CLK_625_PATTERN_55		(1 << 26)
#define    CLK_625_PATTERN_AA		(1 << 25)
#define    LOOP_BACK			(1 << 24)
#define    RESET_PRBS_COUNTER		(1 << 23)
#define    INV_COMPARE			(1 << 22)
#define    LOAD_SEED			(1 << 21)
#define    PRBS_SEED(_x)		(((_x) & 0x3ff) << 11)	 /* 0..1023 */
#define    PRBS_DATA(_x)		(((_x) & 0x3ff) << 1)	 /* 0..1023 */
#define    PRBS_ENABLE			(1 << 0)
#define NEM_GMAC_ANEG_CTRL_R		0x0324
#define    SGMII_CLOCK_RATE		(1 << 13)
#define    IGNORE_AN_SYNC_STATUS	(1 << 12)
#define    IGNORE_RUDI_INVALID		(1 << 11)
#define    FIBER_MODE			(1 << 10)
#define    SGMII_GMAC_RX_PAUSE_EN	(1 << 9)
#define    SGMII_GMAC_FLOW_CONTROL_INITIATE_EN			(1 << 8)
#define    AUTONEG_OFFLINE_ADVERTISE	(1 << 7)
#define    AUTONEG_SPEED_UP		(1 << 6)
#define    SPEED_MODE_FORCE(_x)		(((_x) & 0x3) << 4)	 /* 0..3 */
#define    GOOD_LINK_FORCE		(1 << 3)
#define    FULL_DUPLEX_FORCE		(1 << 2)
#define    MR_AUTONEG_ENABLE		(1 << 1)
#define    RESTART_AUTONEG		(1 << 0)
#define NEM_GMAC_ANEG_STATUS_R		0x0328
#define    WOL				(1 << 13)
#define    C10_GMII_SPEED_MODE(_x)	(((_x) & 0x3) << 11)	 /* 0..3 */
#define    C10_GMII_FULL_DUPLEX		(1 << 10)
#define    GMII_LINK_OK			(1 << 9)
#define    AUTONEG_TX_PAUSE_EN		(1 << 8)
#define    AUTONEG_RX_PAUSE_EN		(1 << 7)
#define    REMOTE_FAULT(_x)		(((_x) & 0x3) << 5)	 /* 0..3 */
#define    REMOTE_PAUSE(_x)		(((_x) & 0x3) << 3)	 /* 0..3 */
#define    REMOTE_HD_FD(_x)		(((_x) & 0x3) << 1)	 /* 0..3 */
#define    MR_AN_COMPLETE		(1 << 0)
#define NEM_GMAC_SGMII_STATUS_R		0x032c
#define    SGMII_FULL_DUPLEX		(1 << 13)
#define    SGMII_SPEED_MODE(_x)		(((_x) & 0x3) << 11)	 /* 0..3 */
#define    SGMII_LINK			(1 << 10)
#define    SYNC_STATUS			(1 << 9)
#define    RX_DV			(1 << 8)
#define    C11_PRBS_ERR			(1 << 7)
#define    CURRENT_AUTONEG_STATE(_x)	(((_x) & 0x7) << 4)	 /* 0..7 */
#define    C11_GMII_FULL_DUPLEX		(1 << 3)
#define    C11_GMII_SPEED_MODE(_x)	(((_x) & 0x3) << 1)	 /* 0..3 */
#define    GMII_LINK			(1 << 0)
#define NEM_GMAC_FILTER_CTRL_R		0x0330
#define    ENABLE_BLANKET_FILTERING	(1 << 7)
#define    ENABLE_UNICAST_SA_FILTERING	(1 << 6)
#define    ENABLE_RUNT_FILTERING	(1 << 5)
#define    ENABLE_PAUSE_FILTERING	(1 << 4)
#define    ENABLE_UNICAST_FILTERING	(1 << 3)
#define    ENABLE_MULTICAST_FILTERING	(1 << 2)
#define    ENABLE_BROADCAST_FILTERING	(1 << 1)
#define    PACKET_FILTERING_ENABLE	(1 << 0)
#define NEM_GMAC_MULTICAST_R(_x)	(0x0334 + (_x) * 4)
#define NEM_GMAC_UNICAST1_R		0x0344
#define NEM_GMAC_UNICAST2_R		0x0348
#define NEM_GMAC_UNICAST12_UPPER_R	0x034c
#define    UNICAST_ADDRESS_1_UPPER(_x)	(((_x) & 0xffff) << 0)
#define    UNICAST_ADDRESS_2_UPPER(_x)	(((_x) & 0xffff) << 16)
#define NEM_GMAC_UNICAST3_R		0x0350
#define NEM_GMAC_UNICAST3_UPPER_R	0x0354
#define    UNICAST_ADDRESS_3_UPPER(_x)	(((_x) & 0xffff) << 0)
#define NEM_GMAC_C37_ANEG_INT_MASK_R	0x0360
#define    WOL				(1 << 13)
#define    GMII_SPEED_MODE_MASK(_x)	(((_x) & 0x3) << 11)	 /* 0..3 */
#define    GMII_FULL_DUPLEX_MASK	(1 << 10)
#define    GMII_LINK_OK_MASK		(1 << 9)
#define    AUTONEG_TX_PAUSE_EN_MASK	(1 << 8)
#define    AUTONEG_RX_PAUSE_EN_MASK	(1 << 7)
#define    REMOTE_FAULT_MASK(_x)	(((_x) & 0x3) << 5)	 /* 0..3 */
#define    REMOTE_PAUSE_MASK(_x)	(((_x) & 0x3) << 3)	 /* 0..3 */
#define    REMOTE_HD_FD_MASK(_x)	(((_x) & 0x3) << 1)	 /* 0..3 */
#define    MR_AN_COMPLETE_MASK		(1 << 0)
#define NEM_GMAC_C73_ANEG_STATUS_R	0x0368
#define    LINK_STATUS_10GKR_CHANGE	(1 << 18)
#define    LINK_STATUS_10GKR		(1 << 17)
#define    LOSS_OF_SIGNAL_CHANGED	(1 << 16)
#define    AN_COMPLETE_CHANGED		(1 << 15)
#define    LINK_STATUS_10GKX4_CHANGE	(1 << 14)
#define    LINK_STATUS_1GKX_CHANGE	(1 << 13)
#define    AN_RESTART_FAULT		(1 << 12)
#define    AN_LINK_OK			(1 << 11)
#define    LOSS_OF_SIGNAL		(1 << 10)
#define    NONCE_FAILURE		(1 << 9)
#define    PARALLEL_DETECTION_FAULT	(1 << 8)
#define    PARALLEL_DETECTION_COMPLETE	(1 << 7)
#define    PAGE_RECEIVED		(1 << 6)
#define    AN_COMPLETE			(1 << 5)
#define    C26_REMOTE_FAULT		(1 << 4)
#define    AN_ABILITY			(1 << 3)
#define    LINK_STATUS_10GKX4		(1 << 2)
#define    LINK_STATUS_1GKX		(1 << 1)
#define    LP_AN_ABILITY		(1 << 0)
#define NEM_GMAC_C73_ANEG_INT_MASK_R	0x038c
#define    LINK_STATUS_10GKR_CHANGE_MASK	(1 << 18)
#define    LINK_STATUS_10GKR_MASK	(1 << 17)
#define    LOSS_OF_SIGNAL_CHANGED_MASK	(1 << 16)
#define    AN_COMPLETE_CHANGED_MASK	(1 << 15)
#define    LINK_STATUS_10GKX4_CHANGE_MASK	(1 << 14)
#define    LINK_STATUS_1GKX_CHANGE_MASK	(1 << 13)
#define    AN_RESTART_FAULT_MASK	(1 << 12)
#define    AN_LINK_OK_MASK		(1 << 11)
#define    LOSS_OF_SIGNAL_MASK		(1 << 10)
#define    NONCE_FAILURE_MASK		(1 << 9)
#define    PARALLEL_DETECTION_FAULT_MASK	(1 << 8)
#define    PARALLEL_DETECTION_COMPLETE_MASK	(1 << 7)
#define    PAGE_RECEIVED_MASK		(1 << 6)
#define    AN_COMPLETE_MASK		(1 << 5)
#define    C35_REMOTE_FAULT_MASK	(1 << 4)
#define    AN_ABILITY_MASK		(1 << 3)
#define    LINK_STATUS_10GKX4_MASK	(1 << 2)
#define    LINK_STATUS_1GKX_MASK	(1 << 1)
#define    LP_AN_ABILITY_MASK		(1 << 0)
#define NEM_GMAC_UCAST_MASK_LOWER_R	0x0394
#define NEM_GMAC_UCAST_MASK_UPPER_R	0x0398
#define NEM_GMAC_PFC_ENABLE_R		0x039c
#define    PLEVEL(_x)			(((_x) & 0xff) << 0)
#define NEM_GMAC_PFC_CONTROL_R		0x03a0
#define    ALLOW_UCAST			(1 << 8)
#define    SEND_PAUSE(_x)		(((_x) & 0xff) << 0)
#define NEM_GMAC_PFC_STATUS_R		0x03a4
#define    PAUSED(_x)			(((_x) & 0xff) << 0)
#define NEM_GMAC_IFG_MON_R		0x03b8
#define    EXCESS_IFG_FRAMES(_x)	(((_x) & 0xffff) << 16)
#define    MIN_IFG(_x)			(((_x) & 0x3f) << 10)
#define    MAX_IFG(_x)			(((_x) & 0x3ff) << 0)

/* STATS REGISTERS */

#define NEM_STATS_INT_STATUS_R		0x0d00
#define NEM_STATS_INT_ENABLE_R		0x0d04
#define NEM_STATS_INT_FORCE_R		0x0d08
#define    STATS_INT_SNAPSHOT_RDY	(1 << 0)
#define NEM_STATS_CONFIG_R		0x0d0c
#define    STATS_MAC_RAW		(1 << 15)
#define    COR				(1 << 14)
#define    MAX_LENGTH(_x)		(((_x) & 0x3fff) << 0)
#define NEM_STATS_SNAPSHOT_R		0x0d40
#define    MAC(_x)			(((_x) & 0xf) << 0)
#define NEM_STATS_BACKDOOR_CONTROL_R	0x0d44
#define    W_R				(1 << 11)
#define    CACHE			(1 << 10)
#define    TX_RX			(1 << 9)
#define    STATS_MAC(_x)		(((_x) & 0xf) << 5)
#define    STATS_COUNTER(_x)		(((_x) & 0x1f) << 0)
#define NEM_STATS_BACKDOOR_DATA_HI_R	0x0d48
#define NEM_STATS_BACKDOOR_DATA_LO_R	0x0d4c
#define NEM_STATS_TX_R(_x)		(0x0e00 + (_x) * 8)
#define NEM_STATS_RX_R(_x)		(0x0f00 + (_x) * 8)

/* DMA REGISTERS */

#define NEM_DMA_CTL		0x3000
#define   DMACTL_RST		(1 << 31)
#define   DMACTL_EN		(1 << 16)
#define   DMACTL_ALLOW_TX_PAUSE	(1 << 15)
#define   DMACTL_FORCE_RX_ORDER	(1 << 14)
#define   DMACTL_TX_TAIL_PTR_EN	(1 << 13)
#define   DMACTL_RX_TAIL_PTR_EN	(1 << 12)
#define   DMACTL_AXI_BURST(_n)	((_n) <<  8)
#define   DMACTL_PTI_BURST(_n)	((_n) <<  4)
#define   DMACTL_ARB(_x)	((_x) <<  0)
#define      ARB_RR		0		/* Round robin */
#define      ARB_PRIO_RX	1		/* Prioritize receive */
#define      ARB_PRIO_TX	2		/* Prioritize transmit */
#define NEM_DMA_ENABLE		0x3004
#define   RXDMA_EN		(1<<17)
#define   TXDMA_EN		(1<<16)
#define NEM_DMA_RXQ_ADDR	0x3008
#define NEM_DMA_TXQ_ADDR	0x3010
#define NEM_DMA_RXQ_SIZE	0x3018
#define NEM_DMA_TXQ_SIZE	0x301c
#define NEM_DMA_RXTAIL_PTR_ADDR	0x3020
#define NEM_DMA_TXTAIL_PTR_ADDR	0x3028
#define NEM_DMA_RXHEAD_PTR	0x3030
#define NEM_DMA_TXHEAD_PTR	0x3034
#define NEM_DMA_AXI_CTL		0x3038
#define   AXICTL_ARPROT(_x)	((_x) << 12)
#define   AXICTL_ARCACHE(_x)	((_x) <<  8)
#define   AXICTL_AWPROT(_x)	((_x) <<  4)
#define   AXICTL_AWCACHE(_x)	((_x) <<  0)
#define NEM_DMA_MISC_CTL	0x303c
#define   MISCCTL_RGMII_1000	(0 << 0)
#define   MISCCTL_RGMII_100	(1 << 0)
#define   MISCCTL_RGMII_10	(2 << 0)
#define NEM_DMA_RXTAIL_PTR	0x3080
#define NEM_DMA_TXTAIL_PTR	0x3084
#define NEM_DMA_STATUS		0x3088
#define NEM_DMA_INTL_STATUS	0x3100
#define   INTL_BAD_PARAM	(1 << 0)
#define NEM_DMA_INTL_MASK	0x3104
#define NEM_DMA_INTL_INV	0x3108
#define NEM_DMA_INTL_NOMASK	0x310c
#define NEM_DMA_INTE_STATUS	0x3110
#define   INTE_RX_DONE		(1 << 9)
#define   INTE_TX_DONE		(1 << 8)
#define   INTE_ERR_MASK		0x7f
#define   INTE_RXDMA_WERR	(1 << 6)
#define   INTE_RXDMA_FIFO_ERR	(1 << 5)
#define   INTE_RXDESC_RERR	(1 << 4)
#define   INTE_TXDMA_RERR	(1 << 3)
#define   INTE_TXDMA_WERR	(1 << 2)
#define   INTE_TXDMA_FIFO_ERR	(1 << 1)
#define   INTE_TXDESC_RERR	(1 << 0)
#define NEM_DMA_INTE_MASK	0x3114
#define NEM_DMA_INTE_INV	0x3118
#define NEM_DMA_INTE_NOMASK	0x311c
#define NEM_DMA_INTE_RAW	0x3120

enum nem_stats_tx_counter_def {
	TX_OCTETS	= 0x00, /* Total Octets: octets in all frames. */
	TX_BAD_OCTETS	= 0x01, /* Bad Octets: octets in all bad frames. */
	TX_FRM		= 0x02, /* Total Frames: number of frames. */
	TX_BAD_FRM	= 0x03, /* Bad Frames: number of bad frames. */
	TX_MCAST	= 0x04, /* Multicast Frames */
	TX_BCAST	= 0x05, /* Broadcast Frames */
	TX_PAUSE	= 0x06, /* Pause Frames */
	TX_FCS		= 0x07, /* FCS Errors */
	TX_DEFER	= 0x08, /* Deferred Frames */
	TX_UNUSED1	= 0x09, /* Unused */
	TX_1COL		= 0x0a, /* Single Collisions */
	TX_MCOL		= 0x0b, /* Multiple Collisions */
	TX_LCOL		= 0x0c, /* Late Collisions */
	TX_XSCOL	= 0x0d, /* Excessive Collisions */
	TX_FRAG		= 0x0e, /* Fragments */
	TX_USIZE	= 0x0f, /* Undersize Frames */
	TX_64		= 0x10, /* 64 Byte Frames: frames with length == 64. */
	TX_65		= 0x11, /* 65-127 Byte Frames */
	TX_128		= 0x12, /* 128-255 Byte Frames */
	TX_256		= 0x13, /* 256-511 Byte Frames */
	TX_512		= 0x14, /* 512-1023 Byte Frames */
	TX_1024		= 0x15, /* 1024-1518 Byte Frames */
	TX_1519		= 0x16, /* 1519-Max Byte Frames */
	TX_OSIZE	= 0x17, /* Oversize Frames */
	TX_JAB		= 0x18, /* Jabbers */
	TX_URUN		= 0x19, /* FIFO Underruns */
	TX_STAT_OVFL	= 0x1a, /* Stats Overflow */
};

enum nem_stats_rx_counter_def {
	RX_OCTETS	= 0x00, /* Total Octets: octets in all frames. */
	RX_BAD_OCTETS	= 0x01, /* Bad Octets: octets in all bad frames. */
	RX_FRM		= 0x02, /* Total Frames: number of frames. */
	RX_BAD_FRM	= 0x03, /* Bad Frames: number of bad frames. */
	RX_MCAST	= 0x04, /* Multicast Frames */
	RX_BCAST	= 0x05, /* Broadcast Frames */
	RX_CTL		= 0x06, /* Control Frames */
	RX_PAUSE	= 0x07, /* Pause Frames */
	RX_UNK_OP	= 0x08, /* Unknown Op Codes */
	RX_FCS		= 0x09, /* FCS Errors */
	RX_ALIGN	= 0x0a, /* Alignment Errors */
	RX_LENGTH	= 0x0b, /* Length Errors */
	RX_CODE		= 0x0c, /* Code Errors */
	RX_FRAG		= 0x0d, /* Fragments */
	RX_USIZE	= 0x0e, /* Undersize Frames */
	RX_64		= 0x0f, /* 64 Byte Frames */
	RX_65		= 0x10, /* 65-127 Byte Frames */
	RX_128		= 0x11, /* 128-255 Byte Frames */
	RX_256		= 0x12, /* 256-511 Byte Frames */
	RX_512		= 0x13, /* 512-1023 Byte Frames */
	RX_1024		= 0x14, /* 1024-1518 Byte Frames */
	RX_1519		= 0x15, /* 1519-Max Byte Frames */
	RX_OSIZE	= 0x16, /* Oversize Frames */
	RX_JAB		= 0x17, /* Jabbers */
	RX_DROP		= 0x18, /* Dropped Frames */
	RX_CARRIER	= 0x19, /* Carrier Errors */
	RX_ORUN		= 0x1a, /* FIFO Overruns */
	RX_STAT_OVFL	= 0x1b, /* Stats Overflow */
};

#define NEMAC_REGISTER_NAMES { \
	{ "NEM_VERSION_R", NEM_VERSION_R }, \
	{ "NEM_SCRATCHPAD_R", NEM_SCRATCHPAD_R }, \
	{ "NEM_PRESENCE_R", NEM_PRESENCE_R }, \
	{ "NEM_INT_STATUS_R", NEM_INT_STATUS_R }, \
	{ "NEM_INT_ENABLE_R", NEM_INT_ENABLE_R }, \
	{ "NEM_INT_FORCE_R", NEM_INT_FORCE_R }, \
	{ "NEM_SWRESET_R", NEM_SWRESET_R }, \
	{ "NEM_ACTIVE_R", NEM_ACTIVE_R }, \
	{ "NEM_LOS_CONTROL_R", NEM_LOS_CONTROL_R }, \
	{ "NEM_LOS_DEBUG_R", NEM_LOS_DEBUG_R }, \
	{ "NEM_PTI_INT_STATUS_R", NEM_PTI_INT_STATUS_R }, \
	{ "REG_PTI_INT_ENABLE_R", REG_PTI_INT_ENABLE_R }, \
	{ "REG_PTI_INT_FORCE_R", REG_PTI_INT_FORCE_R }, \
	{ "NEM_PTI_TX_UNDERRUN_STATUS_R", NEM_PTI_TX_UNDERRUN_STATUS_R }, \
	{ "NEM_PTI_TX_OVERRUN_STATUS_R", NEM_PTI_TX_OVERRUN_STATUS_R }, \
	{ "NEM_PTI_RX_OVERRUN_STATUS_R", NEM_PTI_RX_OVERRUN_STATUS_R }, \
	{ "NEM_PTI_CONFIG_R", NEM_PTI_CONFIG_R }, \
	{ "NEM_PTI_BURST_LENGTH_R", NEM_PTI_BURST_LENGTH_R }, \
	{ "NEM_PTI_GMAC_TX_HIGH_WM_R", NEM_PTI_GMAC_TX_HIGH_WM_R }, \
	{ "NEM_PTI_GMAC_TX_LOW_WM_R", NEM_PTI_GMAC_TX_LOW_WM_R }, \
	{ "NEM_PTI_GMAC_RX_HIGH_WM_R", NEM_PTI_GMAC_RX_HIGH_WM_R }, \
	{ "NEM_PTI_GMAC_RX_LOW_WM_R", NEM_PTI_GMAC_RX_LOW_WM_R }, \
	{ "NEM_PTI_TX_FIFO_STAT_R", NEM_PTI_TX_FIFO_STAT_R }, \
	{ "NEM_PTI_TX_MAX_USED_R", NEM_PTI_TX_MAX_USED_R }, \
	{ "NEM_PTI_TX_TRUNCATED_R", NEM_PTI_TX_TRUNCATED_R }, \
	{ "NEM_PTI_TX_OVERRUNS_R", NEM_PTI_TX_OVERRUNS_R }, \
	{ "NEM_PTI_RX_FIFO_STAT_R", NEM_PTI_RX_FIFO_STAT_R }, \
	{ "NEM_PTI_RX_MAX_USED_R", NEM_PTI_RX_MAX_USED_R }, \
	{ "NEM_PTI_RX_TRUNCATED_R", NEM_PTI_RX_TRUNCATED_R }, \
	{ "NEM_PTI_RX_DROPPED_R", NEM_PTI_RX_DROPPED_R }, \
	{ "NEM_GMAC_ENABLE_R", NEM_GMAC_ENABLE_R }, \
	{ "NEM_GMAC_STA_ADDR_R", NEM_GMAC_STA_ADDR_R }, \
	{ "NEM_GMAC_STA_ADDR_UPPER_R", NEM_GMAC_STA_ADDR_UPPER_R }, \
	{ "NEM_GMAC_TIMING_CTRL_R", NEM_GMAC_TIMING_CTRL_R }, \
	{ "NEM_GMAC_MAC_CTRL_R", NEM_GMAC_MAC_CTRL_R }, \
	{ "NEM_GMAC_MAC_STATUS_R", NEM_GMAC_MAC_STATUS_R }, \
	{ "NEM_GMAC_PRBS_ERR_R", NEM_GMAC_PRBS_ERR_R }, \
	{ "NEM_GMAC_SGMII_ENABLE_R", NEM_GMAC_SGMII_ENABLE_R }, \
	{ "NEM_GMAC_PRBS_CTRL_R", NEM_GMAC_PRBS_CTRL_R }, \
	{ "NEM_GMAC_ANEG_CTRL_R", NEM_GMAC_ANEG_CTRL_R }, \
	{ "NEM_GMAC_ANEG_STATUS_R", NEM_GMAC_ANEG_STATUS_R }, \
	{ "NEM_GMAC_SGMII_STATUS_R", NEM_GMAC_SGMII_STATUS_R }, \
	{ "NEM_GMAC_FILTER_CTRL_R", NEM_GMAC_FILTER_CTRL_R }, \
	{ "NEM_GMAC_MULTICAST_R(0)", NEM_GMAC_MULTICAST_R(0) }, \
	{ "NEM_GMAC_MULTICAST_R(1)", NEM_GMAC_MULTICAST_R(1) }, \
	{ "NEM_GMAC_MULTICAST_R(2)", NEM_GMAC_MULTICAST_R(2) }, \
	{ "NEM_GMAC_MULTICAST_R(3)", NEM_GMAC_MULTICAST_R(3) }, \
	{ "NEM_GMAC_UNICAST1_R", NEM_GMAC_UNICAST1_R }, \
	{ "NEM_GMAC_UNICAST2_R", NEM_GMAC_UNICAST2_R }, \
	{ "NEM_GMAC_UNICAST12_UPPER_R", NEM_GMAC_UNICAST12_UPPER_R }, \
	{ "NEM_GMAC_UNICAST3_R", NEM_GMAC_UNICAST3_R }, \
	{ "NEM_GMAC_UNICAST3_UPPER_R", NEM_GMAC_UNICAST3_UPPER_R }, \
	{ "NEM_GMAC_C37_ANEG_INT_MASK_R", NEM_GMAC_C37_ANEG_INT_MASK_R }, \
	{ "NEM_GMAC_C73_ANEG_STATUS_R", NEM_GMAC_C73_ANEG_STATUS_R }, \
	{ "NEM_GMAC_C73_ANEG_INT_MASK_R", NEM_GMAC_C73_ANEG_INT_MASK_R }, \
	{ "NEM_GMAC_UCAST_MASK_LOWER_R", NEM_GMAC_UCAST_MASK_LOWER_R }, \
	{ "NEM_GMAC_UCAST_MASK_UPPER_R", NEM_GMAC_UCAST_MASK_UPPER_R }, \
	{ "NEM_GMAC_PFC_ENABLE_R", NEM_GMAC_PFC_ENABLE_R }, \
	{ "NEM_GMAC_PFC_CONTROL_R", NEM_GMAC_PFC_CONTROL_R }, \
	{ "NEM_GMAC_PFC_STATUS_R", NEM_GMAC_PFC_STATUS_R }, \
	{ "NEM_GMAC_IFG_MON_R", NEM_GMAC_IFG_MON_R }, \
	{ "NEM_STATS_CONFIG_R", NEM_STATS_CONFIG_R }, \
	{ "NEM_STATS_SNAPSHOT_R", NEM_STATS_SNAPSHOT_R }, \
	{ "NEM_DMA_CTL", NEM_DMA_CTL }, \
	{ "NEM_DMA_ENABLE", NEM_DMA_ENABLE }, \
	{ "NEM_DMA_RXHEAD_PTR", NEM_DMA_RXHEAD_PTR }, \
	{ "NEM_DMA_TXHEAD_PTR", NEM_DMA_TXHEAD_PTR }, \
	{ "NEM_DMA_RXTAIL_PTR", NEM_DMA_RXTAIL_PTR }, \
	{ "NEM_DMA_TXTAIL_PTR", NEM_DMA_TXTAIL_PTR }, \
	{ "NEM_DMA_INTL_STATUS", NEM_DMA_INTL_STATUS }, \
	{ "NEM_DMA_INTL_MASK", NEM_DMA_INTL_MASK }, \
	{ "NEM_DMA_INTL_NOMASK", NEM_DMA_INTL_NOMASK }, \
	{ "NEM_DMA_INTE_STATUS", NEM_DMA_INTE_STATUS }, \
	{ "NEM_DMA_INTE_MASK", NEM_DMA_INTE_MASK }, \
	{ "NEM_DMA_INTE_NOMASK", NEM_DMA_INTE_NOMASK }, \
	{ "NEM_DMA_INTE_RAW", NEM_DMA_INTE_RAW }, }

#endif /* _NEMAC_REGS_H */
