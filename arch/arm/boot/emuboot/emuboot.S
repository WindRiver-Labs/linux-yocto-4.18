/*
 * emuboot.S - simple register setup code for stand-alone Linux booting
 *
 * Copyright (C) 2011 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */

#define SPIN_TABLE_BASE 0x10000000

	.syntax	unified
	.text

reset_exception:
	b	_start

undefined_exception:
	b	undefined_exception

supervisor_exception:
	b	supervisor_exception

prefetch_abort:
	b	prefetch_abort

data_abort:
	b	data_abort

unused_exception:
	b	unused_exception

irq_exception:
	b	irq_exception

fiq_exception:
	b	fiq_exception

	.globl	_start
_start:
	@
	@ Program architected timer frequency
	@
	mrc	p15, 0, r0, c0, c1, 1		@ CPUID_EXT_PFR1
	lsr	r0, r0, #16
	ands	r0, r0, #1			@ Check generic timer support
	beq	1f
	@ldr	r0, =6250000			@ 6.25 MHz timer frequency
	ldr	r0, =4096000			@ 4.096 MHz timer frequency
	mcr	p15, 0, r0, c14, c0, 0		@ CNTFRQ

	@
	@ CPU initialisation
	@
1:	mrc	p15, 0, r0, c0, c0, 5		@ MPIDR (ARMv7 only)
	mov	r1, r0
	and	r0, r1,0x3			@ CPU number
	cmp	r0, #0				@ primary CPU in any cluster
	bne	actlr2_set
	lsr	r0, r1, #8
	ands	r0, r0, #3			@ primary CPU in cluster 0
	beq primary

	@
	@ Secondary CPUs
	@
	@ Set the L2ACTLR
	@
secondary:	and	r1, r1,0x3			@ CPU number
	cmp	r1, #0
	bne 	actlr2_set

	mrc	p15, 1, r0, c15, c0, 0		@ L2ACTLR
	@ Set bit 3 - disable clean/evict push to external
	orr	r0, r0, #(0x1 << 3)
	@ Set bit 12 - disable multiple outstanding
	@ WriteClean/WriteBack/Evicts using same AWID
	orr	r0, r0, #(0x1 << 12)
	@ Set bit 13 - disable SharedClean data transfers
	orr	r0, r0, #(0x1 << 13)
	@ Set bit 14 - enable UniqueClena evictions with data
	orr	r0, r0, #(0x1 << 14)
	mcr	p15, 1, r0, c15, c0, 0

	@
	@ Set the ACTLR2
	@
	mrc	p15, 1, r0, c15, c0, 4
	@ Set bit 0 - execute data cache clean as data cache clean/invalidate
	orr	r0, r0, #(0x1 << 0)
	mcr	p15, 1, r0, c15, c0, 4

	@
	@ CPU initialisation
	@
	mrc	p15, 0, r0, c0, c0, 5		@ MPIDR (ARMv7 only)
	and	r0, r0, #15			@ CPU number
	cmp	r0, #0				@ primary CPU?
	beq	2f

	@
	@ Secondary CPUs
	@
	ldr	r1, =SPIN_TABLE_BASE
	adr	r2, 1f
	ldmia	r2, {r3 - r7}			@ move the code to a location
	stmia	r1, {r3 - r7}			@ less likely to be overridden
	add	r0, r1, #0x20			@ Entry point for secondary
						@ CPUs @ SPIN_TABLE_BASE+0x20
	mov	r2, #0
	str	r2, [r0, #0]			@ ensure initially zero
	mov	pc, r1				@ branch to the relocated code
1:
	wfe
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
	.asciz	"root=/dev/nfs rw mem=1024M console=ttyAMA0 ip=dhcp"

	.align	2

1:

	@ ATAG_NONE
	.long	0
	.long	0x00000000
