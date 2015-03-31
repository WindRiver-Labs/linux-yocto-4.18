/*
 * fmboot.S - simple register setup code for stand-alone Linux booting
 *
 * Copyright (C) 2011 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */

#define SPIN_TABLE_BASE 0x10000000

	.syntax	unified
	.text

	.globl	_start
_start:
	@
	@ Program architected timer frequency
	@
	mrc	p15, 0, r0, c0, c1, 1		@ CPUID_EXT_PFR1
	lsr	r0, r0, #16
	ands	r0, r0, #1			@ Check generic timer support
	beq	1f
	ldr	r0, =100000000			@ 100MHz timer frequency
	mcr	p15, 0, r0, c14, c0, 0		@ CNTFRQ
1:
	@
	@ CPU initialisation
	@
	mrc	p15, 0, r0, c0, c0, 5		@ MPIDR (ARMv7 only)
	bic 	r0, #0xff000000			@ CPU number
	cmp	r0, #0				@ primary CPU?
	beq	2f

	@
	@ Secondary CPUs
	@
	ldr	r1, =SPIN_TABLE_BASE
	adr	r2, 1f
	ldmia	r2, {r3 - r7}			@ move the code to a location
	stmia	r1, {r3 - r7}			@ less likely to be overridden
	add	r0, r1, #0x20			@ Entry point for secondary CPUs
						@ SPIN_TABLE_BASE+0x20
	mov	r2, #0
	str	r2, [r0, #0]			@ ensure initially zero
	mov	pc, r1				@ branch to the relocated code
1:
	ldr	r1, [r0]
	cmp	r1, #0
	beq	1b
	mov	pc, r1				@ branch to the given address

	@
	@ Kernel parameters
	@
2:	mov	r0, #0				@ Must be zero
	mov	r1, #0				@ Machine type (not needed)
	adr	r2, atags			@ ATAGS pointer
	mov	r3, #0
	ldr	lr, =kernel
	mov     pc, lr				@ jump to the kernel

	.org	0x100
atags:
	@ ATAG_CORE
	.long	2
	.long	0x54410001

	@ ATAG_CMDLINE
	.long	(1f - .) >> 2
	.long	0x54410009
	.asciz	"root=/dev/mmcblk0 rootwait ip=none mem=1024M console=ttyAMA0"

	.align	2
1:

	@ ATAG_NONE
	.long	0
	.long	0x00000000
