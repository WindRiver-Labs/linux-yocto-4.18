/* arch/arm/mach-axxia/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/hardware.h>

#if !defined(CONFIG_ARM_LPAE)
#error "Axxia Peripherals Are Only Accessible Using the LPAE!"
#endif

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =(AXXIA_DEBUG_UART_PHYS & 0xffffffff)
	ldr	\rv, =AXXIA_DEBUG_UART_VIRT
	.endm

#include <asm/hardware/debug-pl01x.S>
