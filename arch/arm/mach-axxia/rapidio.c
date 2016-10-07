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
