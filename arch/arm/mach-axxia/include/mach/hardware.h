/*
 * arch/arm/mach-axxia/include/mach/hardware.h
 *
 * Copyright (c) 2013 LSI Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#ifndef __MACH_HARDWARE_H
#define __MACH_HARDWARE_H

#define AXXIA_UART0_PHYS       0x2010080000
#define AXXIA_UART1_PHYS       0x2010081000
#define AXXIA_UART2_PHYS       0x2010082000
#define AXXIA_UART3_PHYS       0x2010083000

#ifdef CONFIG_DEBUG_LL_AXXIA_UART0
#define AXXIA_DEBUG_UART_VIRT  0xf0080000
#define AXXIA_DEBUG_UART_PHYS  AXXIA_UART0_PHYS
#endif

#endif
