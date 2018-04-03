/*
 * arch/arm/mach-axxia/io.c
 *
 * Support for the INTEL Axxia boards based on ARM cores.
 *
 * Copyright (C) 2018 INTEL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <asm/page.h>
#include <asm/io.h>

void __iomem *
__axxia_arch_ioremap(phys_addr_t physical_address, size_t size,
		     unsigned int flags)
{
	unsigned long pfn;
	unsigned long offset;

	pfn = (unsigned long)((physical_address >> PAGE_SHIFT) & 0xffffffffULL);
	offset = (unsigned long)(physical_address & (PAGE_SIZE - 1));

	return __arm_ioremap_pfn(pfn, offset, size, flags);
}
EXPORT_SYMBOL(__axxia_arch_ioremap);
