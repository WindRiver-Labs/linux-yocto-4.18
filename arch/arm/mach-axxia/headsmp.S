/*
 *  linux/arch/arm/mach-axxia/headsmp.S
 *
 *  Cloned from linux/arch/arm/mach-realview/headsmp.S
 *
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>
#include <linux/init.h>

	__CPUINIT

/*
 * Axxia specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 */
ENTRY(axxia_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5
	bic	r0, #0xff000000
	adr	r4, 1f
	ldmia	r4, {r5, r6}
	sub	r4, r4, r5
	add	r6, r6, r4
pen:	ldr	r7, [r6]
	/*
	 * It seems that just looping over read/compare kills starves
	 * ethernet, this will happen if we start the kernel with
	 * maxcpus=X where X < 16. Which we really want in order to
	 * isolate cores.
	 *
	 * FIXME: We should really use wfi or wfe here
	 */
	mov	r4, #0x7000
__delay:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	subs	r4,r4,#1
	bne	__delay
	cmp	r7, r0
	bne	pen

	/*
	 * We've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	b	secondary_startup
ENDPROC(axxia_secondary_startup)

	.align 2
1:	.long	.
	.long	pen_release
