/*
 *  arch/arm/mach-vexpress/include/mach/io.h
 *
 *  Copyright (C) 2003 ARM Limited
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
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <linux/types.h>

#define __io(a)		__typesafe_io(a)
#define __mem_pci(a)	(a)

/*
  Make the first argument to ioremap() be phys_addr_t (64 bits in this
  case) instead of unsigned long.  When __arch_ioremap is defiend,
  __arch_iounmap must be defined also.  Just use the default for
  iounmap().
*/

void __iomem *__axxia_arch_ioremap(phys_addr_t, size_t, unsigned int);
#define __arch_ioremap __axxia_arch_ioremap
#define __arch_iounmap __arm_iounmap

#endif
