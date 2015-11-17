/*
 * linux/arch/arm/mach-axxia/rapidio.c
 *
 * Helper module for board specific RAPIDIO bus registration
 *
 * Copyright (C) 2013-2014 LSI Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/lsi-ncr.h>
#include <linux/signal.h>

#include <mach/rio.h>

/**
 * axxia_rapidio_board_init -
 *   Perform board-/controller-specific initialization to support
 *   use of RapidIO busses
 *
 * @dev:     [IN] RIO platform device
 * @ndx:     [IN] Which instance of SRIOC driver needs support
 * @port_ndx: [OUT] Which port to use for the specified controller
 *
 * Returns 0 on success or an error code.
 */

int
axxia_rapidio_board_init(struct platform_device *dev, int dev_num, int *port_ndx)
{
	/* Reset the RIO port id to zero for this device */
	void __iomem *gpreg_base = ioremap(0x2010094000, 0x1000);
	unsigned long reg = 0;

	if (gpreg_base == NULL)
		return -EFAULT;

	reg = inl((unsigned long int)(gpreg_base + 0x60));

	reg &= ~(0xf << (dev_num * 4));

	outl_p(reg, (unsigned long int)(gpreg_base + 0x60));

	(*port_ndx) = 0;

	/* Verify that this device is actually enabled */
	if (NULL !=
	    of_find_compatible_node(NULL, NULL, "lsi,axm5500-amarillo")) {
		ncr_read(NCP_REGION_ID(0x115, 0), 0x23c, 4, &reg);

		if ((reg & (1 << (21+(dev_num*4)))) == 0) {
			dev_dbg(&dev->dev, "%s: SRIO%d link not ready\n",
				dev->dev.of_node->full_name, dev_num);
			return -ENXIO;
		}
	}

	iounmap(gpreg_base);

	return 0;
}

/**
 * axxia_rio_fault -
 *   Intercept SRIO bus faults due to unimplemented register locations.
 *   Return 0 to keep 'reads' alive.
 */

static int
axxia_rio_fault(unsigned long addr, unsigned int fsr, struct pt_regs *regs)
{
	/* unsigned long pc = instruction_pointer(regs); */
	/* unsigned long instr = *(unsigned long *)pc; */
	return 0;
}

/**
 * axxia_rapidio_init -
 *   Perform board-specific initialization to support use of RapidIO busses
 *
 * Returns 0 on success or an error code.
 */
int __init
axxia_rapidio_init(void)
{
	hook_fault_code(0x11, axxia_rio_fault, SIGBUS, 0,
			"asynchronous external abort");

	return 0;
}
