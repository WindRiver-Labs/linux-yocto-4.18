/*
 *  Copyright (C) 2016 Intel Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

/*
  ==============================================================================
  ==============================================================================
  Private
  ==============================================================================
  ==============================================================================
*/

#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/lsi-ncr.h>
#include <linux/mutex.h>

static int is_5500;
static int is_5600;
static int is_6700;

static void __iomem *pcie_gpreg0;
static void __iomem *pcie_gpreg1;
static void __iomem *pcie_gpreg2;
static void __iomem *pcie_rc;

static int pcie_cc_gpreg_offset = 0x8000;
static int is_pei_control_available;
static int is_pei_control_v2;

DEFINE_MUTEX(axxia_pei_mux);

struct axxia_pei {
	unsigned int	control;
	int		control_set;
	unsigned int	initialized;
};

struct pei_coefficients {
	unsigned version;
	unsigned control;
	unsigned primary_input_clock;
	unsigned input_ref_clock_range;
	unsigned lane_0_eq_main;
	unsigned lane_0_eq_pre;
	unsigned lane_0_eq_post;
	unsigned lane_0_vboost;
	unsigned lane_1_eq_main;
	unsigned lane_1_eq_pre;
	unsigned lane_1_eq_post;
	unsigned lane_1_vboost;
};

static struct axxia_pei axxia_pei;
static struct pei_coefficients coefficients;

enum SataMode {
	SATA0,
	SATA1
};
enum SataSpeed {
	SATA_SPEED_1_5G,
	SATA_SPEED_3G,
	SATA_SPEED_6G
};
enum SrioMode {
	SRIO0,
	SRIO1
};
enum SrioSpeed {
	SRIO_SPEED_1_25G,
	SRIO_SPEED_2_5G,
	SRIO_SPEED_3_125G,
	SRIO_SPEED_5G,
	SRIO_SPEED_6_25G
};

enum PCIMode {
	PEI0,
	PEI1,
	PEI2
};
enum PLLMode {
	PLLA,
	PLLB
};
enum PowerState {
	P0,
	P1,
	P2
};

enum Dir {
	TX,
	RX
};

/* PEI0x8 000/010 which is valid? */
enum port_config0 {
	pc0_PEI0x8 = 0x0,
	pc0_PEI0x4 = 0x1,
	pc0_PEI0x8_alt = 0x2,
	pc0_PEI0x2_PEI2x2 = 0x3,
	pc0_SRIO1x2_SRIO0x2 = 0x4,
	pc0_PEI0x2_SRIO0x2 = 0x7,
};

enum port_config1 {
	pc1_PEI0x8 = 0x0,
	pc1_PEI1x4 = 0x1,
	pc1_PEI1x2_SATA0x1_SATA1x1 = 0x2,
	pc1_PEI1x2_PEI2x2 = 0x3,
};

enum pipe_port {
	pp_disable = 0x0,
	pp_0 = 0x1,
	pp_0_1 = 0x2,
	pp_0_1_2_3 = 0x3,
};

enum pipe_nphy {
	one_phy = 0x0,
	two_phy = 0x1,
	three_phy = 0x2,
	four_phy = 0x3,
};

static int pei_trace;
module_param(pei_trace, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(pei_trace, "enable pei_trace");

static u32
pei_readl(void *address)
{
	u32 value;

	value = readl(address);

	if (0 != pei_trace)
		printk("PEI: Read 0x%x from 0x%px\n", value, address);

	return value;
}

static void
pei_writel(u32 value, void *address)
{
	writel(value, address);

	if (0 != pei_trace)
		printk("PEI: Wrote 0x%x to 0x%p\n", value, address);

	return;
}

/******************************************************************************
 * PCIe/SRIO/SATA parameters
 * Supported configs:
 *	PEI0x8
 *	PEI0x4_PEI1x4
 *	PEI0x4_PEI1x2_SATA0x1_SATA1x1
 *	PEI0x2_PEI2x2_PEI1x2_SATA0x1_SATA1x1
 *	PEI0x2_SRIO0x2_PEI1x4
 *	PEI0x2_SRIO0x2_PEI1x2_SATA0x1_SATA1x1
 *	PEI0x2_SRIO0x2_PEI1x2_PEI2x2
 *	SRIO1x2_SRIO0x2_PEI1x4
 *	SRIO1x2_SRIO0x2_PEI1x2_SATA0x1_SATA1x1
 *	SRIO1x2_SRIO0x2_PEI1x2_PEI2x2
 *
 * Bits 25:22 :Supported configs:
 *	0x0 -	PEI0x8 (HSS10-ch0,1;HSS11-ch0,1;HSS12-ch0,1;HSS13-ch0,1)
 *	0x1 -	PEI0x4 (HSS10-ch0,1; HSS11-ch0,1)
 *		PEI1x4 (HSS12-ch0,1; HSS12-ch0,1)
 *	0x2 -	PEI0x4 (HSS10-ch0,1; HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		SATA0x1 (HSS13-ch0)
 *		SATA1x1 (HSS13-ch1)
 *	0x3 -	PEI0x2 (HSS10-ch0,1)
 *		PEI2x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		SATA0x1 (HSS13-ch0)
 *		SATA1x1 (HSS13-ch1)
 *	0x4 -	PEI0x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x4 (HSS12-ch0,1; HSS13-ch0,1)
 *	0x5 -	PEI0x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		SATA0x1 (HSS13-ch0)
 *		SATA1x1 (HSS13-ch1)
 *	0x6 -	PEI0x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		PEI2x2 (HSS13-ch0,1)
 *	0x7 -	SRIO1x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x4 (HSS12-ch0,1; HSS13-ch0,1)
 *	0x8 -	SRIO1x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		SATA0x1 (HSS13-ch0)
 *		SATA1x1 (HSS13-ch1)
 *	0x9 -	SRIO1x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		PEI2x2 (HSS13-ch0,1)
 * Bits 21:20: SATA1 speed selection
 *	0x0 - indicates 1.5 Gbps
 *	0x1 - indicates 3 Gbps
 *	0x2 - indicates 6 Gbps
 * Bits 19:18: SATA0 speed selection
 *	0x0 - indicates 1.5 Gbps
 *	0x1 - indicates 3 Gbps
 *	0x2 - indicates 6 Gbps
 * Bits 17:15 : SRIO1 speed selection
 *	0x0 -  indicates 1.25 Gbps
 *	0x1 -  indicates 2.5 Gbps
 *	0x2 -  indicates 3.125 Gbps
 *	0x3 -  indicates 5 Gbps
 *	0x4 -  indicates 6.25 Gbps
 * Bits 14:12 : SRIO0 speed selection
 *	0x0 -  indicates 1.25 Gbps
 *	0x1 -  indicates 2.5 Gbps
 *	0x2 -  indicates 3.125 Gbps
 *	0x3 -  indicates 5 Gbps
 *	0x4 -  indicates 6.25 Gbps
 * Bits 11:10 : If SRIO1 is configured,
 *	0x0 - indicates Host with ID 0,
 *	0x1 - indicates Host with ID 1,
 *	0x2 - indicates not a host (agent)
 *	0x3 - RESERVED
 * Bits 9:8 : If SRIO0 is configured,
 *	0x0 - indicates Host with ID 0,
 *	0x1 - indicates Host with ID 1,
 *	0x2 - indicates not a host (agent)
 *	0x3 - RESERVED
 * Bit 7: If PEI0 is configured,
 *	0x0 - indicates END_POINT
 *	0x1 - indicates ROOT_COMPLEX
 * Bit 6: SATA1 enabled
 * Bit 5: SATA0 enabled
 * Bit 4: SRIO1 enabled
 * Bit 3: SRIO0 enabled
 * Bit 2: PEI2 enabled
 * Bit 1: PEI1 enabled
 * Bit 0: PEI0 enabled
 ******************************************************************************/

void setup_sata_mode(enum SataMode mode, enum SataSpeed speed)
{
	u32 regVal, speedVal = 0, laneid;

	if (mode == SATA0)
		laneid = 6;
	else
		laneid = 7;

	/* setup Tx word mode */
	/* 20 bit mode */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c)), &regVal);
	regVal &= 0xffffffcf;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c)), regVal);
	/* setup Rx word mode */
	/* 20 bit mode */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0xc),
		   &regVal);
	regVal &= 0xfffffcff;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0xc),
		    regVal);
	/* set gearbox word clk sel lane 1 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x14),
		   &regVal);
	regVal |= (0x1) << 20;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x14),
		    regVal);
	/* config tx lane 6 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c)), &regVal);
	/* Disable TX swing boost */
	regVal &= 0xffff07ff;
	/* TX swing level */
	regVal |= (15 << 12);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c)), regVal);
	/* Program SATA speed */
	if (speed == SATA_SPEED_1_5G)
		speedVal = 20;
	else if (speed == SATA_SPEED_3G)
		speedVal = 23;
	else if (speed == SATA_SPEED_6G)
		speedVal = 29;

	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x8),
		    speedVal);

	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x4),
		   &regVal);
	regVal &= 0xffec8fcf;
	/* Set power state of the transmitter
	 * select baud rate
	 * MPLLB select disable
	 * Enable TX MPLL
	 * 20 bit width of TX data
	 */
	regVal |= (3 << 4) | (1 << 12) | (3 << 16) | (0 << 20) | (1 << 22);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x4),
		    regVal);

	/* config rx lane 6 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0xc),
		   &regVal);
	/* Disable LFPS filter
	 * Disable word alignment
	 * Disable divide by 16.5 recovered clock
	 */
	regVal &= 0xff05ff87;
	/* LOS threshold set to 120mVpp
	 * Enable CDR tracking of RX data
	 * Enable CDR tracking gains and duration
	 * Set Receiver VCO lower frequency band mode
	 * Grounded RX termination
	 */
	regVal |= ((0x2 << 4) | (1 << 17) | (1 << 20) | (1 << 22) | (1 << 23));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0xc),
		    regVal);

	/* RX VCO calibration load and reference load value */
	regVal = (12 << 0) | (1440 << 8);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x10),
		    regVal);

	/* Set AFE attenuation level to -2dB
	 * Set the AFE first and second stage varialble Gain Amplifier gain
	 * Set the continous time linear equalizer CTLE boost pole location
	 * Set the CTLE boost level
	 * Set the value of DFE data tap 1
	 */
	regVal = ((0 << 0) | (7 << 4) | (7 << 8) | (0 << 12) | (6 << 16) |
		  (0 << 24));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x18),
		    regVal);
	/* Program SATA speed */
	if (speed == SATA_SPEED_1_5G)
		speedVal = 3;
	else if (speed == SATA_SPEED_3G)
		speedVal = 2;
	else if (speed == SATA_SPEED_6G)
		speedVal = 1;
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x14),
		   &regVal);
	/* Disable AFE/DFE */
	regVal &= 0xff2ccfcf;
	regVal |= (3 << 4) | (speedVal << 12) | (3 << 16);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x14),
		    regVal);

	/* PLLA/SATA */
	/*
	 *	ref_clk_en                  = 1'b1;
	 *	ref_use_pad                 = 1'b1;
	 *	ref_clk_div2_en             = 1'b0;
	 *	ref_range                   = 3'h3;
	 *	ref_clk_mplla_div2_en       = 1'b1;
	 *	ref_clk_mpllb_div2_en       = 1'b1;
	 *	ref_repeat_clk_en           = 1'b0;
	 *	tx_vboost_lvl               = 3'h5;
	 *	vref_ctrl                   = 5'h11;
	 */

	/* Enable ref clk, ref use_pad, ref_clk_div2_en */
	regVal = ((0x1) | ((0x1) << 1) | (0x1 << 2));
	/* Select ref range 78.1-104 MHz */
	regVal |= (0x3 << 4);
	/* Enable ref_clk_mplla_div2_en, ref_clk_mpllb_div2_en
	 * and ref_repeat_clk_en */
	regVal |= ((0x1 << 8) | (0x1 << 9) | (0x1 << 10));
	/* Set tx_vboost_lvl and vref_ctrl */
	regVal |= ((0x3 << 16) | (17 << 20));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18)),
		    regVal);

	/* MPLLA setting
	 * mplla_ssc_en                = 1'b1;
	 * mplla_div_clk_en            = 1'b0;
	 * mplla_ssc_range             = 3'h0;
	 * mplla_ssc_clk_sel   = 2'h2;
	 * mplla_div_multiplier        = 7'h0;
	 */
	/* MPLLA spread-spectrum clock (SSC) and
	 * div_clk enable */
	regVal = (0x1 | (0x1 << 2));
	/*  SSC range  and SSC clk select */
	regVal |= (2 << 20);
	/* mplla_div_multiplier */
	regVal |= (7 << 24);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x8),
		    regVal);

	/* MPLLA settings
	 * mplla_force_en              = 1'b0;
	 * mplla_div8_clk_en   = 1'b0;
	 * mplla_div10_clk_en  = 1'b1;
	 * mplla_div16p5_clk_en        = 1'b0;
	 * mplla_multiplier            = 7'h3c;
	 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x4),
		   &regVal);
	/* Enable MPLLA force, mplla_div8_clk_en, mplla_div10_clk_en
	 * and mplla_div16p5_clk_en */
	/* MPLLA multiplier is 0 */
	regVal |= ((0x1) | (0x1 << 1) | (0x1 << 2) |
		   (0x1 << 3));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x4),
		    regVal);

	/*
	 * mplla_fracn_ctrl            = 9'h0;
	 * mplla_bandwidth             = 7'h2a;
	 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x10),
		   &regVal);
	regVal &= 0xffff0000;
	regVal |= (0x42 << 9);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x10),
		    regVal);
}

void setup_srio_mode(enum SrioMode mode, enum SrioSpeed speed)
{
	u32 regVal;
	u32 phy, lane, rate, width, pll;
	int i = 0;

	if (mode == SRIO0) {
		phy = 1;
		lane = 2;
	} else {
		phy = 0;
		lane = 0;
	}

	/* set wordmode to bypass for lane 0 and lane 1*/
	for (i = lane; i <= (lane + 1); i++) {
		/* set Tx word mode */
		ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (i * 0x1c)),
			   &regVal);
		regVal &= (~(3 << 4));
		regVal |= (0 << 4);
		/* TX iboost level */
		regVal |= (15 << 12);
		ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (i * 0x1c)),
			    regVal);

		/* control for setting the transmitter driver
		 * output pre-emphasis/post-emphasis and amplitude
		 */
		regVal = ((0 << 8) | (0 << 16) | (40 << 0) | (1 << 28));
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0x18 + (i * 0x1c) + 0x8),
			    regVal);
		/* Tx width */
		ncr_read32(NCP_REGION_ID(0x115, 0),
			   (0x18 + (i * 0x1c) + 0x4), &regVal);
		regVal &= 0xffac8fcf;
		/* SEt the power state of the transmitter
		 * TX rate/width
		 */
		rate = (speed == SRIO_SPEED_1_25G) ? 3 :
			((speed == SRIO_SPEED_2_5G) ||
			 (speed == SRIO_SPEED_3_125G)) ? 2 : 1;
		width = 1;
		pll = 0;
		pll = ((speed == SRIO_SPEED_3_125G) ||
		       (speed == SRIO_SPEED_6_25G)) ? 1 : 0;
		regVal |= ((0x3 << 4) | (rate << 12) | (width << 16)
			   | (pll << 20) | (1 << 22));
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0x18 + (i * 0x1c) + 0x4), regVal);

		if ((speed == SRIO_SPEED_3_125G) || (speed == SRIO_SPEED_6_25G))
			regVal = (0x550 << 8) | 0x22;
		else
			regVal = (0x540 << 8) | 0x2a;
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0x18 + (i * 0x1c) + 0x10), regVal);
		ncr_read32(NCP_REGION_ID(0x115, 0),
			   (0x18 + (i * 0x1c) + 0x14), &regVal);
		regVal &= 0xff3c8fcf;
		regVal |= ((0x3 << 4) | (rate << 12) | (width << 16));
		if ((speed == SRIO_SPEED_5G) || (speed == SRIO_SPEED_6_25G))
			regVal |= ((1 << 22) | (1 << 23));
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0x18 + (i * 0x1c) + 0x14), regVal);

	}
	/* set gearbox word clk sel for phy 1 lane 0/lane 1 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x14),
		   &regVal);
	regVal &= (~(0x7 << 20));
	if (speed == SRIO_SPEED_1_25G)
		regVal |= (0x2) << 20;
	else if ((speed == SRIO_SPEED_2_5G) || (speed == SRIO_SPEED_3_125G))
		regVal |= (0x1) << 20;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x14),
		    regVal);
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x14),
		   &regVal);
	regVal &= (~(0x7 << 24));
	if (speed == SRIO_SPEED_1_25G)
		regVal |= (0x2) << 24;
	else if ((speed == SRIO_SPEED_2_5G) || (speed == SRIO_SPEED_3_125G))
		regVal |= (0x1) << 24;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x14),
		    regVal);
	/* PLLA Settings */
	/* Enable ref clk, ref use_pad, ref_clk_div2_en */
	regVal = ((0x1) | ((0x1) << 1) | (0 << 2));
	/* Select ref range 78.1-104 MHz */
	regVal |= (0x6 << 4);
	/* Enable ref_clk_mplla_div2_en, ref_clk_mpllb_div2_en
	 * and ref_repeat_clk_en */
	regVal |= ((0x1 << 8) | (0x1 << 9) | (0 << 10));
	/* Set tx_vboost_lvl and vref_ctrl */
	regVal |= ((0x5 << 16) | (17 << 20));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18)),
		    regVal);

	/* MPLLA setting
	 * mplla_ssc_en                = 1'b0;
	 * mplla_div_clk_en            = 1'b0;
	 * mplla_ssc_range             = 3'h0;
	 * mplla_ssc_clk_sel   = 2'h2;
	 * mplla_div_multiplier        = 7'h0;
	 */
	/* MPLLA spread-spectrum clock (SSC) and
	 * div_clk enable */
	regVal = (0 | (0 << 2));
	/*  SSC range  and SSC clk select */
	regVal |= (0 << 20);
	/* mplla_div_multiplier */
	regVal |= (0 << 24);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x8),
		    regVal);

	switch (speed) {
	case SRIO_SPEED_1_25G:
	case SRIO_SPEED_2_5G:
	case SRIO_SPEED_5G:
		/* 500 MHz PLLA */
		/* MPLLA settings
		 * mplla_force_en              = 1'b0;
		 * mplla_div8_clk_en   = 1'b0;
		 * mplla_div10_clk_en  = 1'b1;
		 * mplla_div16p5_clk_en        = 1'b0;
		 * mplla_multiplier            = 7'h20;
		 */
		ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x4),
			   &regVal);
		/* Disable MPLLA force, mplla_div8_clk_en
		 * Enable mplla_div10_clk_en
		 * and Disable mplla_div16p5_clk_en */
		/* MPLLA multiplier is 32 */
		regVal |= ((0x1 << 2) | (0x20 << 4));
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0xf8 + (phy * 0x18) + 0x4),
			    regVal);
		/*
		 * mplla_fracn_ctrl            = 9'h0;
		 * mplla_bandwidth             = 7'h3a;
		 */
		ncr_read32(NCP_REGION_ID(0x115, 0),
			   (0xf8 + (phy * 0x18) + 0x10),
			   &regVal);
		regVal &= 0xffff0000;
		regVal |= (0x3a << 9);
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0xf8 + (phy * 0x18) + 0x10),
			    regVal);
		break;
	case SRIO_SPEED_3_125G:
	case SRIO_SPEED_6_25G:
		/* 625 MHz PLLB */
		/* MPLLB settings
		 * mpllb_force_en              = 1'b0;
		 * mpllb_div8_clk_en   = 1'b0;
		 * mpllb_div10_clk_en  = 1'b1;
		 * mpllb_div16p5_clk_en        = 1'b0;
		 * mpllb_multiplier            = 7'h28;
		 */
		ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x4),
			   &regVal);
		/* Disable MPLLB force, mpllb_div8_clk_en
		 * Enable mpllb_div10_clk_en
		 * and Disable mpllb_div16p5_clk_en */
		/* MPLLB multiplier is 40 */
		regVal |= (((0x1 << 2) | (0x28 << 4))<<16);
		regVal |= ((0x1 << 2) | (0x20 << 4)); /* Is this needed*/
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0xf8 + (phy * 0x18) + 0x4), regVal);
		/*
		 * mplla_fracn_ctrl            = 9'h0;
		 * mplla_bandwidth             = 7'h5b;
		 */
		ncr_read32(NCP_REGION_ID(0x115, 0),
			   (0xf8 + (phy * 0x18) + 0x10),
			   &regVal);
		regVal &= 0xffff;
		regVal |= ((0x5b << 9)<<16);
		regVal |= (0x3a << 9); /* Is this needed*/
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0xf8 + (phy * 0x18) + 0x10),
			    regVal);
		if (speed == SRIO_SPEED_6_25G) {
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x16, 0x0010);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x24, 0x001E);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x26, 0x0000);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x18, 0x0010);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x44, 0x001E);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x46, 0x0000);
		}
		break;
	}
}

void disable_ltssm(enum PCIMode mode)
{
	u32 val;

	if (!is_5600 && !is_6700)
		return;

	switch (mode) {
	case PEI0:
		/* LTSSM Disable for PEI0 */
		val = pei_readl(pcie_gpreg0 + pcie_cc_gpreg_offset + 0x38);
		val &= (~(0x1));
		pei_writel(val, pcie_gpreg0 + pcie_cc_gpreg_offset + 0x38);
		break;
	case PEI1:
		/* LTSSM Disable for PEI1 */
		if (!is_5600)
			break;
		val = pei_readl(pcie_gpreg1 + pcie_cc_gpreg_offset + 0x38);
		val &= (~(0x1));
		pei_writel(val, pcie_gpreg1 + pcie_cc_gpreg_offset + 0x38);
		break;
	case PEI2:
		/* LTSSM Disable for PEI2 */
		if (!is_5600)
			break;
		val = pei_readl(pcie_gpreg2 + pcie_cc_gpreg_offset + 0x38);
		val &= (~(0x1));
		pei_writel(val, pcie_gpreg2 + pcie_cc_gpreg_offset + 0x38);
		break;
	default:
		pr_err("%s Unsupported PEI %d\n", __func__, mode);
		break;
	};

	mdelay(100);		/* TODO: Why is this needed? */
}

void set_sw_port_config0(enum port_config0 mode)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0, &regVal);
	regVal &= ~(0x7 << 26);
	regVal |= ((mode & 0x7) << 26);
	ncr_write32(NCP_REGION_ID(0x115, 0), 0, regVal);
}

void set_sw_port_config1(enum port_config1 mode)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &regVal);
	regVal &= ~(0x3 << 22);
	regVal |= ((mode & 0x3) << 22);
	ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, regVal);
}

static void set_pipe_port_sel(enum pipe_port pp)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &regVal);
	regVal &= ~(0x3 << 24);
	regVal |= ((pp & 0x3) << 24);
	ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, regVal);
}

static void set_pipe_nphy(enum pipe_nphy nphy)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &regVal);
	regVal &= ~(0x3 << 20);
	regVal |= ((nphy & 0x3) << 20);
	ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, regVal);
}

static void set_srio_mode(enum SrioMode mode, unsigned int ctrl)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0, &regVal);
	if (SRIO0 == mode) {
		regVal &= ~(0x3 << 20);
		regVal |= ((ctrl & 0x3) << 20);
	} else {
		regVal &= ~(0x3 << 24);
		regVal |= ((ctrl & 0x3) << 24);
	}
	ncr_write32(NCP_REGION_ID(0x115, 0), 0, regVal);
}

static void set_srio_speed(enum SrioMode mode, enum SrioSpeed speed)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0x0), 0x4, &regVal);
	if (SRIO0 == mode) {
		regVal &= ~(0x7 << 12);
		regVal |= ((speed & 0x7) << 12);
	} else {
		regVal &= ~(0x7 << 16);
		regVal |= ((speed & 0x7) << 16);
	}
	ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, regVal);
}

static void set_pei0_rc_mode(u32 rc)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0x0), 0x0, &regVal);
	if (rc)
		regVal |= (0x1 << 22);
	else
		regVal &= ~(0x1 << 22);
	ncr_write32(NCP_REGION_ID(0x115, 0x0), 0x0, regVal);
}

void enable_reset(u32 phy)
{
	u32 regVal;

	if (phy == 0) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &regVal);
		regVal |= (1 << 5);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, regVal);
	} else if (phy == 1) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &regVal);
		regVal |= (1 << 14);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, regVal);
	} else if (phy == 2) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &regVal);
		regVal |= (1 << 19);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, regVal);
	} else if (phy == 3) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &regVal);
		regVal |= (1 << 29);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, regVal);
	}
}

static void release_reset(u32 phy)
{
	u32 regVal;

	if (phy == 0) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &regVal);
		regVal &= (~(1 << 5));
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, regVal);
	} else if (phy == 1) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &regVal);
		regVal &= (~(1 << 14));
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, regVal);
	} else if (phy == 2) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &regVal);
		regVal &= (~(1 << 19));
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, regVal);
	} else if (phy == 3) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &regVal);
		regVal &= (~(1 << 29));
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, regVal);
	}
}

static int check_pll_lock(enum PLLMode mode, u32 phy)
{
	u32 regVal;
	int count = 0;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8), &regVal);
	regVal &= (1 << (20 + mode));

	do {
		/* 100 ms delay */
		mdelay(100);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8),
			   &regVal);
		regVal &= (1 << (20 + mode));
	} while ((!regVal) && (count++ < 5));

	return regVal;
}

static int release_srio_reset(enum SrioMode mode, enum SrioSpeed speed)
{
	u32 phy;
	enum PLLMode pll;

	switch (mode) {
	case SRIO0:
		phy = 1;
		break;
	case SRIO1:
		phy = 0;
		break;
	default:
		pr_err("Wrong SRIO%d\n", mode);
		return 1;
	}

	release_reset(phy);
	pll = PLLA;
	if ((SRIO_SPEED_3_125G == speed) ||
	    (SRIO_SPEED_6_25G == speed))
		pll = PLLB;
	if (!check_pll_lock(pll, phy)) {
		pr_err("%s didn't lock\n", pll == PLLA ? "PLLA" : "PLLB");
		return 1;
	}
	return 0;
}

static int check_ack(u32 phy, u32 lane, enum Dir dir)
{
	u32 regVal;
	int count = 0;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8), &regVal);
	if (dir == TX)
		regVal &= (1 << (0 + lane));
	else
		regVal &= (1 << (16 + lane));

	do {
		/* 100 ms delay */
		mdelay(100);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8),
			   &regVal);
		if (dir == TX)
			regVal &= (1 << (0 + lane));
		else
			regVal &= (1 << (16 + lane));
	} while ((!regVal) && (count++ < 5));

	return regVal;
}

int check_srio_ack(enum SrioMode mode)
{
	unsigned int phy;
	unsigned int lane;

	switch (mode) {
	case SRIO0:
		phy = 1;
		break;
	case SRIO1:
		phy = 0;
		break;
	default:
		pr_err("Wrong SRIO%d\n", mode);
		return 1;
	}

	for (lane = 0; lane < 2; lane++) {
		if (!check_ack(phy, lane, RX)) {
			pr_err("RX ACK not set for PHY%d LANE%d\n", phy, lane);
			return 1;
		}
	}

	return 0;
}

void set_tx_clk_ready(void)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x8, &regVal);
	regVal |= 0xff;
	ncr_write32(NCP_REGION_ID(0x115, 0), 0x8, regVal);
}

/* RX clock and data recovery */
int check_rx_valid(u32 phy, u32 lane)
{
	u32 regVal;
	int count = 0;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8), &regVal);
	regVal &= (1 << (24 + (4 * lane)));

	do {
		/* 100 ms delay */
		mdelay(100);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8), &regVal);
		regVal &= (1 << (24 + (4 * lane)));
	} while ((!regVal) && (count++ < 5));

	return regVal;
}

/*
  ------------------------------------------------------------------------------
  powerup_lane
*/

static int
powerup_lane(u32 phy, u32 lane, enum PowerState state, enum Dir dir)
{
	u32 regVal, powerVal;
	u32 offset;
	int ret;

	if (dir == TX)
		offset = 0x18 + (((phy * 2) + lane) * 0x1c) + 0x4;
	else
		offset = 0x18 + (((phy * 2) + lane) * 0x1c) + 0x14;

	ncr_read32(NCP_REGION_ID(0x115, 0), offset, &regVal);
	regVal &= 0xffffffce;
	/* New Transmitter setting request */
	powerVal = (state == P2) ? 3 : ((state == P1) ? 2 : 0);
	regVal |= (powerVal << 4) | (1 << 0);
	ncr_write32(NCP_REGION_ID(0x115, 0), offset, regVal);

	/* Check if ack is set */
	ret = check_ack(phy, lane, dir);
	ncr_read32(NCP_REGION_ID(0x115, 0), offset, &regVal);
	regVal &= ~(1 << 0);
	ncr_write32(NCP_REGION_ID(0x115, 0), offset, regVal);

	return ret;
}

static int
powerup_srio_lanes(enum SrioMode mode, enum PowerState state)
{
	u32 lane;
	u32 phy;

	switch (mode) {
	case SRIO0:
		phy = 1;
		break;
	case SRIO1:
		phy = 0;
		break;
	default:
		pr_err("Wrong SRIO%d\n", mode);
		return 1;
	}

	/* Power up TX/RX lanes */
	for (lane = 0; lane < 2; lane++) {
		if (!powerup_lane(phy, lane, state, TX)) {
			pr_err("TX powerup failed for PHY%d LANE%d\n",
			       phy, lane);
			return 1;
		}
		if (!powerup_lane(phy, lane, state, RX)) {
			pr_err("RX powerup failed for PHY%d LANE%d\n",
			       phy, lane);
			return 1;
		}
	}
	return 0;
}

/*
  ------------------------------------------------------------------------------
  enable_lane
*/

static void
enable_lane(u32 phy, u32 lane, enum Dir dir)
{
	u32 regVal;
	u32 offset;

	if (TX == dir)
		offset = 0x18 + (((phy * 2) + lane) * 0x1c);
	else
		offset = 0x18 + (((phy * 2) + lane) * 0x1c) + 0x14;

	ncr_read32(NCP_REGION_ID(0x115, 0), offset, &regVal);

	if (TX == dir)
		regVal |= (1 << 25);
	else
		regVal |= (1 << 21);

	ncr_write32(NCP_REGION_ID(0x115, 0), offset, regVal);
}

static void enable_srio_lanes(enum SrioMode mode)
{
	u32 lane;
	u32 phy;

	switch (mode) {
	case SRIO0:
		phy = 1;
		break;
	case SRIO1:
		phy = 0;
		break;
	default:
		pr_err("Wrong SRIO%d\n", mode);
		return;
	}

	for (lane = 0; lane < 2; lane++) {
		enable_lane(phy, lane, TX);
		enable_lane(phy, lane, RX);
	}
}

/*
  ==============================================================================
  ==============================================================================
  Public
  ==============================================================================
  ==============================================================================
*/

/*
  ------------------------------------------------------------------------------
  update_settings
*/

static void
update_settings(void)
{
	int i;
	unsigned int region;
	int number_of_serdes;

	/*
	  Make sure the parameters are version 2...
	*/

	if (is_5500 || !is_pei_control_v2)
		return;

	region = NCP_REGION_ID(0x115, 0);

	if (is_5600)
		number_of_serdes = 4;
	else
		number_of_serdes = 1;

	/*
	  Set per serdes values.
	*/

	for (i = 0; i < number_of_serdes; ++i) {
		unsigned int offset;
		unsigned int pic;
		unsigned int ref_range;
		unsigned int value;

		if (is_5600)
			offset = (0xf8 + (i * 0x18));
		else
			offset = 0x44;

		if (0 != (coefficients.primary_input_clock & (0xff << (i * 8))))
			pic = 2;
		else
			pic = 0;

		ref_range = (coefficients.input_ref_clock_range &
			     (0xff << (i * 8))) >> (i * 8);

		ncr_read32(region, offset, &value);
		value &= ~0x72;
		value |= pic;
		value |= ref_range << 4;
		ncr_write32(region, offset, value);
	}

	/*
	  Set per lane values.
	*/

	for (i = 0; i < (number_of_serdes * 2); ++i) {
		unsigned int offset;
		int eq_override;
		unsigned int eq_main;
		unsigned int pre;
		unsigned int post;
		int boost_override;
		unsigned int boost;
		unsigned int value;

		if (4 > i) {
			eq_main = coefficients.lane_0_eq_main;
			pre = coefficients.lane_0_eq_pre;
			post = coefficients.lane_0_eq_post;
			boost = coefficients.lane_0_vboost;
		} else {
			eq_main = coefficients.lane_1_eq_main;
			pre = coefficients.lane_1_eq_pre;
			post = coefficients.lane_1_eq_post;
			boost = coefficients.lane_1_vboost;
		}

		switch (i % 4) {
		case 0:
			eq_main &= 0xbf;
			pre &= 0x3f;
			post &= 0x3f;
			boost &= 0x81;
			break;
		case 1:
			eq_main = (eq_main & 0xbf00) >> 8;
			pre = (pre & 0x3f00) >> 8;
			post = (post & 0x3f00) >> 8;
			boost = (boost & 0x8100) >> 8;
			break;
		case 2:
			eq_main = (eq_main & 0xbf0000) >> 16;
			pre = (pre & 0x3f0000) >> 16;
			post = (post & 0x3f0000) >> 16;
			boost = (boost & 0x810000) >> 16;
			break;
		case 3:
			eq_main = (eq_main & 0xbf000000) >> 24;
			pre = (pre & 0x3f000000) >> 24;
			post = (post & 0x3f000000) >> 24;
			boost = (boost & 0x81000000) >> 24;
			break;
		default:
			pr_err("Error setting coefficients!\n");
			break;
		}

		/* Initialize the eq override bit. */

		if (0 != (eq_main & 0x80))
			eq_override = 1;
		else
			eq_override = 0;

		eq_main &= 0x3f;

		/* Initialize the vboost override bit. */

		if (0 != (boost & 0x80))
			boost_override = 1;
		else
			boost_override = 0;

		boost &= 1;

		/*
		  To set the EQ values, use the cobalt registers
		  (0x115.1+), not the config registers (0x115.0).
		*/

		region = NCP_REGION_ID(0x115, ((i / 2) + 1));

		/*
		  Set VBOOST (5600 only).
		*/

		if (is_5600 && 0 != boost_override) {
			/* Set or clear txN_vboost_en. */

			if (0 == (i % 2))
				offset = 0xe002;
			else
				offset = 0xe202;

			ncr_read32(region, offset, &value);

			if (0 != boost)
				value |= (1 << 6);
			else
				value &= ~(1 << 6);

			value |= (1 << 7); /* override enable */

			ncr_write32(region, offset, value);
		}

		if (0 != eq_override) {
			/* Set EQ main. */

			if (0 == (i % 2))
				offset = 0x24;
			else
				offset = 0x44;

			if (is_6700)
				offset *= 2;

			ncr_read32(region, offset, &value);
			value &= ~0x3f;
			value |= eq_main;
			ncr_write32(region, offset, value);

			/* Set EQ pre/post. */

			if (0 == (i % 2))
				offset = 0x26;
			else
				offset = 0x46;

			if (is_6700)
				offset *= 2;

			ncr_read32(region, offset, &value);
			value &= ~0xfff;
			value |= ((post << 6) | pre);
			ncr_write32(region, offset, value);

			/* Set the override: 0x115.1.<0x16|0x18> bit 4. */

			if (0 == (i % 2))
				offset = 0x16;
			else
				offset = 0x18;

#if defined(CONFIG_AXXIA_ANY_XLF)
			offset *= 2;
#endif

			ncr_read32(region, offset, &value);
			value |= (1 << 4);
			ncr_write32(region, offset, value);
		}
	}

	return;
}

/*
  ------------------------------------------------------------------------------
  pei_setup
*/

static int
pei_setup_56xx(unsigned int control)
{
	unsigned int pci_srio_sata_mode;
	unsigned int rc_mode;
	unsigned int srio0_mode;
	unsigned int srio1_mode;
	unsigned int srio0_speed;
	unsigned int srio1_speed;
	unsigned int srio0_ctrl;
	unsigned int srio1_ctrl;
	unsigned int pei0_mode;
	unsigned int pei1_mode;
	unsigned int pei2_mode;
	u32 reg_val = 0;
	int phy;

	if (0 != pei_trace)
		ncr_start_trace();

	pci_srio_sata_mode = (control & 0x03c00000) >> 22;

	srio0_mode = (control & 0x8) >> 3;
	srio1_mode = (control & 0x10) >> 4;

	srio0_speed = (control & 0x7000) >> 12;
	srio1_speed = (control & 0x38000) >> 15;

	srio0_ctrl = (control & 0x300) >> 8;
	srio1_ctrl = (control & 0xc00) >> 10;

	pei0_mode = (control & 0x1);
	pei1_mode = (control & 0x2) >> 1;
	pei2_mode = (control & 0x4) >> 2;

	rc_mode = (control & 0x80) >> 7;

	if (pei0_mode)
		disable_ltssm(PEI0);
	if (pei1_mode)
		disable_ltssm(PEI1);
	if (pei2_mode)
		disable_ltssm(PEI2);

	for (phy = 0; phy < 4; phy++)
		enable_reset(phy);

	/* Disable all interfaces */
	ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
	reg_val &= ~(0xf | (0x1 << 10) | (0x3 << 29));
	ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
	mdelay(100);		/* TODO: Why is this needed? */

	switch (pci_srio_sata_mode) {
	case 1:
		/*
		  PEI0x4  (HSS10-ch0,1; HSS11-ch0,1)
		  PEI1x4  (HSS12-ch0,1; HSS13-ch0,1)
		*/

		set_sw_port_config0(pc0_PEI0x4);
		set_sw_port_config1(pc1_PEI1x4);
		set_pei0_rc_mode(rc_mode);
		set_pipe_port_sel(pp_0_1);
		set_pipe_nphy(four_phy);

		if (pei0_mode)
			for (phy = 0; phy < 2; phy++)
				release_reset(phy);
		if (pei1_mode)
			for (phy = 2; phy < 4; phy++)
				release_reset(phy);

		ncr_read32(NCP_REGION_ID(0x115, 0), 0x0, &reg_val);
		reg_val |= (pei0_mode << 0) | (pei1_mode << 1);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x0, reg_val);
		break;

	case 2:
		/*
		  PEI0x2  (HSS10-ch0,1)
		  PEI2x2  (HSS11-ch0,1)
		  PEI1x2  (HSS12-ch0,1)
		  UNUSED  (HSS13-ch0,1)
		*/

		set_sw_port_config0(pc0_PEI0x2_PEI2x2);
		set_sw_port_config1(pc1_PEI1x2_PEI2x2);
		set_pei0_rc_mode(rc_mode);
		set_pipe_port_sel(pp_0_1_2_3);

		if (pei0_mode)
			release_reset(0);
		if (pei1_mode)
			release_reset(1);
		if (pei2_mode)
			release_reset(2);

		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val |= (pei0_mode << 0) | (pei1_mode << 1) |
			(pei2_mode << 2);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		break;

	case 3:
		/*
		 * PEI0x2  (HSS10-ch0,1)
		 * SRIO0x2 (HSS11-ch0,1)
		 * UNUSED  (HSS12-ch0,1)
		 * PEI2x2  (HSS13-ch0,1)
		 * INFO: Formerly case 6...
		 */

		set_pei0_rc_mode(rc_mode);
		set_sw_port_config0(pc0_PEI0x2_SRIO0x2);
		set_sw_port_config1(pc1_PEI1x2_PEI2x2);
		set_pipe_port_sel(pp_0_1_2_3);
		set_pipe_nphy(three_phy);

		if (pei0_mode)
			release_reset(0);
		if (pei1_mode)
			release_reset(2);
		if (pei2_mode)
			release_reset(3);

		/* Enable PEI0/PEI1/PEI2 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val |= (pei0_mode << 0) | (pei1_mode << 1) |
			(pei2_mode << 2);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);

		set_srio_mode(SRIO0, srio0_ctrl);
		set_srio_speed(SRIO0, srio0_speed);
		if (srio0_mode) {
			pr_err("Set up sRIO0 -- %d\n", srio0_speed);
			setup_srio_mode(SRIO0, srio0_speed);
			if (release_srio_reset(SRIO0, srio0_speed))
				srio0_mode = 0;
		}

		if (srio0_mode && powerup_srio_lanes(SRIO0, P1))
			srio0_mode = 0;

		/* Set TX clock ready */
		if (srio0_mode) {
			set_tx_clk_ready();

			if (powerup_srio_lanes(SRIO0, P0))
				srio0_mode = 0;
		}
		if (srio0_mode)
			enable_srio_lanes(SRIO0);

		/* Enable SRIO0 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val |= (srio0_mode << 3);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		break;

	case 4:
		/*
		 * SRIO1x2 (HSS10-ch0,1)
		 * SRIO0x2 (HSS11-ch0,1)
		 * UNUSED  (HSS12-ch0,1)
		 * PEI2x2  (HSS13-ch0,1)
		 */

		set_sw_port_config0(pc0_SRIO1x2_SRIO0x2);
		set_sw_port_config1(pc1_PEI1x2_PEI2x2);
		set_pipe_port_sel(pp_0_1_2_3);

		if (pei1_mode)
			release_reset(2);
		if (pei2_mode)
			release_reset(3);

		/* Enable PEI1/PEI2 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val |= (pei1_mode << 1) | (pei2_mode << 2);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);

		set_srio_mode(SRIO0, srio0_ctrl);
		set_srio_mode(SRIO1, srio1_ctrl);
		set_srio_speed(SRIO0, srio0_speed);
		set_srio_speed(SRIO1, srio1_speed);
		if (srio0_mode) {
			pr_debug("Set up sRIO0 -- %d\n", srio0_speed);
			setup_srio_mode(SRIO0, srio0_speed);
			if (release_srio_reset(SRIO0, srio0_speed))
				srio0_mode = 0;
		}
		if (srio1_mode) {
			pr_debug("Set up sRIO1 -- %d\n", srio1_speed);
			setup_srio_mode(SRIO1, srio1_speed);
			if (release_srio_reset(SRIO1, srio1_speed))
				srio1_mode = 0;
		}
		pr_debug("Enabling sRIO .");
		/* Power up TX/RX lanes */
		if (srio0_mode && powerup_srio_lanes(SRIO0, P1))
			srio0_mode = 0;
		if (srio1_mode && powerup_srio_lanes(SRIO1, P1))
			srio1_mode = 0;

		pr_debug(".");
		/* Set TX clock ready */
		if (srio0_mode || srio1_mode)
			set_tx_clk_ready();

		/* Power up TX/RX lanes */
		if (srio0_mode && powerup_srio_lanes(SRIO0, P0))
			srio0_mode = 0;
		if (srio1_mode && powerup_srio_lanes(SRIO1, P0))
			srio1_mode = 0;
		pr_debug(".");

		if (srio0_mode)
			enable_srio_lanes(SRIO0);
		if (srio1_mode)
			enable_srio_lanes(SRIO1);
		pr_debug(".");
		/* Enable SRIO0/SRIO1 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val |= (srio0_mode << 3) | (srio1_mode << 10);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		pr_debug("Done\n");
		break;

	default:
		pr_err("axxia_pei: invalid PCI/SRIO config\n");
		pr_err("pei_setup control=0x%08x pci_srio_sata_mode=%d\n",
			control, pci_srio_sata_mode);

		if (0 != pei_trace)
			ncr_stop_trace();

		return -1;
		break;
	}

	update_settings();

	if (0 != pei_trace)
		ncr_stop_trace();

	return 0;
}

static int
pei_setup_67xx(unsigned int control)
{
	unsigned int pci_mode;
	unsigned int rc_mode;
	unsigned int pei0_mode;
	u32 reg_val = 0;

	pci_mode = (control & 0x00c00000) >> 22;
	pei0_mode = (control & 0x1);
	rc_mode = (control & 0x80) >> 7;

	if (pei0_mode)
		disable_ltssm(PEI0);

	enable_reset(0);

	/* Disable pei0 interfaces */
	ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
	reg_val &= ~(0x1);
	ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
	mdelay(100);		/* TODO: Why is this needed? */

	switch (pci_mode) {

	case 1:
		/*
		 * PEI0x2
		 */
		if (0) {
			set_pei0_rc_mode(rc_mode);
			set_pipe_port_sel(pp_0);
			set_pipe_nphy(one_phy);
		}

		if (pei0_mode)
			release_reset(0);

		/* Enable PEI0 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x0, &reg_val);
		reg_val |= (pei0_mode << 0);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x0, reg_val);
		break;

	case 2:
		/*
		 * PEI0x1
		 */
		set_pei0_rc_mode(rc_mode);
		set_pipe_port_sel(pp_0);
		set_pipe_nphy(one_phy);

		if (pei0_mode)
			release_reset(0);

		/* Enable PEI0 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x0, &reg_val);
		reg_val |= (pei0_mode << 0);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x0, reg_val);
		break;

	default:
		pr_err("axxia_pei: invalid PCI/SRIO config\n");
		pr_err("pei_setup control=0x%08x pci_mode=%d\n",
			control, pci_mode);

	return -1;
	}

	update_settings();

	return 0;
}

/*
  ------------------------------------------------------------------------------
  get_v2_coefficients
*/

static char *names[] = {
		"primary_input_clock",
		"input_ref_clock_range",
		"lane_0_eq_main",
		"lane_0_eq_pre",
		"lane_0_eq_post",
		"lane_0_vboost",
		"lane_1_eq_main",
		"lane_1_eq_pre",
		"lane_1_eq_post",
		"lane_1_vboost"
};

static int
get_v2_coefficients(struct device_node *pei_control)
{
	int i;
	unsigned *lvalue;

	lvalue = &coefficients.primary_input_clock;

	for (i = 0; i < sizeof(names) / sizeof(char *); ++i, ++lvalue) {
		const unsigned int *value;

		value = of_get_property(pei_control, names[i], NULL);

		if (NULL == value) {
			pr_warn("Failed reading %s\n.", names[i]);

			return -1;
		}

		*lvalue = be32_to_cpu(*value);
	}

	return 0;
}

int axxia_pei_setup(unsigned int control, unsigned int force)
{
	int ret = 0;
	int rv = -1;

	mutex_lock(&axxia_pei_mux);

	pr_debug("axxia_pei_setup: control=0x%08x, force = 0x%08x\n",
			control, force);
	axxia_pei.control = control;
	axxia_pei.control_set = 1;

	if (1 == force)
		axxia_pei.initialized = 0;

	if (axxia_pei.initialized)
		goto cleanup;

	/*
	  Previously, if the boot loader set 'control', it did
	  not initialized the PEI.  Start with that
	  assumption.
	*/

	if (0 == axxia_pei.initialized) {
		if (is_5600)
			rv = pei_setup_56xx(control);
		else if (is_6700)
			rv = pei_setup_67xx(control);

		if (0 != rv) {
			pr_err("pcie-axxia: PEI setup failed!\n");

			ret = -EINVAL;
			goto cleanup;
		} else
			axxia_pei.initialized = 1;

		msleep(100);
	}

cleanup:
	mutex_unlock(&axxia_pei_mux);
	return ret;
}
EXPORT_SYMBOL(axxia_pei_setup);

unsigned int axxia_pei_get_control(void)
{
	unsigned int ret;

	mutex_lock(&axxia_pei_mux);
	ret = axxia_pei.control;
	mutex_unlock(&axxia_pei_mux);
	return ret;
}
EXPORT_SYMBOL(axxia_pei_get_control);

int axxia_pei_is_control_set(void)
{
	int ret;

	mutex_lock(&axxia_pei_mux);
	ret = axxia_pei.control_set;
	mutex_unlock(&axxia_pei_mux);
	return ret;
}
EXPORT_SYMBOL(axxia_pei_is_control_set);


/*
  ------------------------------------------------------------------------------
  pei_init
*/

static int
pei_init(void)
{
	/* Use the device tree to determine the Axxia type. */
	if (of_find_compatible_node(NULL, NULL, "lsi,axm5500") ||
	    of_find_compatible_node(NULL, NULL, "lsi,axm5516")) {
		is_5500 = 1;
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axm5616")) {
		is_5600 = 1;
		pcie_gpreg0 = ioremap(0xa003000000, 0x10000);
		pcie_gpreg1 = ioremap(0xa005000000, 0x10000);
		pcie_gpreg2 = ioremap(0xa007000000, 0x10000);
		pcie_rc = ioremap(0xa002000000, 0x1000);
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axc6732")) {
		is_6700 = 1;
		pcie_gpreg0 = ioremap(0xa003000000, 0x10000);
		pcie_rc = ioremap(0xa002000000, 0x1000);
	} else {
		pr_err("No Valid Compatible String Found for PEI!\n");

		return -1;
	}

	pr_debug("is_5500=%d is_5600=%d is_6700=%d\n",
		 is_5500, is_5600, is_6700);

	if ((1 == is_5600) || (1 == is_6700)) {
		struct device_node *pei_control;
		const unsigned int *value;

		memset(&coefficients, 0, sizeof(struct pei_coefficients));
		is_pei_control_available = 0;
		is_pei_control_v2 = 0;

		/* Get the extra parameters. */
		pei_control = of_find_node_by_name(NULL, "pei_control");

		if (!pei_control) {
			pr_warn("No Parameters Available for PEI Setup!\n");

			return 0;
		}

		value = of_get_property(pei_control, "control", NULL);

		if (NULL == value) {
			pr_warn("PEI Control Version is NOT set!\n");

			return 0;
		}

		coefficients.control = be32_to_cpu(*value);
		is_pei_control_available = 1;
		pr_debug("coefficients.control = 0x%x\n",
			 coefficients.control);

		value = of_get_property(pei_control, "version", NULL);

		if (NULL == value) {
			pr_warn("PEI Control Version is NOT set!\n");

			return 0;
		}

		coefficients.version = be32_to_cpu(*value);
		pr_debug("coefficients.version = 0x%x\n",
			 coefficients.version);

		if (2 == coefficients.version) {
			if (0 != get_v2_coefficients(pei_control)) {
				pr_warn("Error reading PEI Coefficients!\n");

				return 0;
			}

			is_pei_control_v2 = 1;

			pr_debug("primary_input_clock=0x%x\n",
				 coefficients.primary_input_clock);
			pr_debug("input_ref_clock_range=0x%x\n",
				 coefficients.input_ref_clock_range);
			pr_debug("lane_0_eq_main=0x%x\n",
				 coefficients.lane_0_eq_main);
			pr_debug("lane_0_eq_pre=0x%x\n",
				 coefficients.lane_0_eq_pre);
			pr_debug("lane_0_eq_post=0x%x\n",
				 coefficients.lane_0_eq_post);
			pr_debug("lane_0_vboost=0x%x\n",
				 coefficients.lane_0_vboost);
			pr_debug("lane_1_eq_main=0x%x\n",
				 coefficients.lane_1_eq_main);
			pr_debug("lane_1_eq_pre=0x%x\n",
				 coefficients.lane_1_eq_pre);
			pr_debug("lane_1_eq_post=0x%x\n",
				 coefficients.lane_1_eq_post);
			pr_debug("lane_1_vboost=0x%x\n",
				 coefficients.lane_1_vboost);
		}
	}

	return 0;
}

arch_initcall(pei_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PEI Setup for Axxia");
