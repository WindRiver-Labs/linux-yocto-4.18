/*
 *  linux/arch/arm/mach-realview/hotplug.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/of_address.h>
#include <linux/delay.h>

#include <mach/axxia-gic.h>
#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/cp15.h>
#include "lsi_power_management.h"
#include "axxia_circular_queue.h"
extern struct circular_queue_t axxia_circ_q;

extern volatile int pen_release;

static inline void pm_cpu_logical_shutdown(u32 cpu)
{
	u32 val;

	asm volatile(
	"       mrc     p15, 1, %0, c9, c0, 2\n"
	: "=&r" (val)
	: "Ir" (0x1)
	: "cc");

	asm volatile(
	"       mrc     p15, 0, %0, c1, c0, 0\n"
	"       bic     %0, %0, %1\n"
	"       mcr     p15, 0, %0, c1, c0, 0\n"
	: "=&r" (val)
	: "Ir" (CR_C)
	: "cc");

	/* Clear and invalidate all date from L1 data cache */
	flush_cache_all();

	/* Switch the processor over to AMP mode out of SMP */
	asm volatile(
			"       mrc     p15, 0, %0, c1, c0, 1\n"
			"       bic     %0, %0, %1\n"
			"       mcr     p15, 0, %0, c1, c0, 1\n"
			: "=&r" (val)
			: "Ir" (0x40)
			: "cc");

	isb();
	dsb();

	wfi();

}

static inline void pm_L2_logical_shutdown(u32 cpu)
{
	u32 val;

	asm volatile(
	"       mrc     p15, 0, %0, c1, c0, 0\n"
	"       bic     %0, %0, %1\n"
	"       mcr     p15, 0, %0, c1, c0, 0\n"
	: "=&r" (val)
	: "Ir" (CR_C)
	: "cc");


	asm volatile(
			/*
			 * Disable L2 prefetch
			 */
			"       mrc     p15, 1, %0, c15, c0, 3\n"
			"       orr     %0, %0, %1\n"
			"       mcr     p15, 1, %0, c15, c0, 3\n"
			: "=&r" (val)
			: "Ir" (0x400)
			: "cc");

	asm volatile(
	"	mrc		p15, 1, %0, c15, c0, 4\n"
	"	orr	%0, %0, %1\n"
	"	mcr		p15, 1, %0, c15, c0, 4\n"
	: "=&r" (val)
	: "Ir" (0x1)
	: "cc");

	isb();
	dsb();

	flush_cache_all();

	/* Turn the DBG Double Lock quiet */
	asm volatile(
			/*
			 * Turn Off the DBGOSDLR.DLK bit
			 */
			"       mrc     p14, 0, %0, c1, c3, 4\n"
			"       orr     %0, %0, %1\n"
			"       mcr     p14, 0, %0, c1, c3, 4\n"
			: "=&r" (val)
			: "Ir" (0x1)
			: "cc");

	/* Switch the processor over to AMP mode out of SMP */
	asm volatile(
			"       mrc     p15, 0, %0, c1, c0, 1\n"
			"       bic     %0, %0, %1\n"
			"       mcr     p15, 0, %0, c1, c0, 1\n"
			: "=&r" (val)
			: "Ir" (0x40)
			: "cc");

	isb();
	dsb();

	wfi();
}

#ifdef CONFIG_HOTPLUG_CPU_LOW_POWER
static inline void cpu_enter_lowpower_a15(void)
{
	unsigned int v;

	asm volatile(
			"       mrc     p15, 0, %0, c1, c0, 0\n"
			"       bic     %0, %0, %1\n"
			"       mcr     p15, 0, %0, c1, c0, 0\n"
			: "=&r" (v)
			: "Ir" (CR_C)
			: "cc");

	flush_cache_all();

	asm volatile(
			/*
			 * Turn off coherency
			 */
			"       mrc     p15, 0, %0, c1, c0, 1\n"
			"       bic     %0, %0, %1\n"
			"       mcr     p15, 0, %0, c1, c0, 1\n"
			: "=&r" (v)
			: "Ir" (0x40)
			: "cc");

	isb();
	dsb();
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile(
			"mrc	p15, 0, %0, c1, c0, 0\n"
			"	orr	%0, %0, %1\n"
			"	mcr	p15, 0, %0, c1, c0, 0\n"
			"	mrc	p15, 0, %0, c1, c0, 1\n"
			"	orr	%0, %0, %2\n"
			"	mcr	p15, 0, %0, c1, c0, 1\n"
			: "=&r" (v)
			: "Ir" (CR_C), "Ir" (0x40)
			: "cc");
	isb();
	dsb();
}

static void __ref platform_do_lowpower(unsigned int cpu, int *spurious)
{
	int phys_cpu, cluster;

	/*
	 * there is no power-control hardware on this platform, so all
	 * we can do is put the core into WFI; this is safe as the calling
	 * code will have already disabled interrupts
	 */
	for (;;) {
		wfi();

	/*
	* Convert the "cpu" variable to be compatible with the
	* ARM MPIDR register format (CLUSTERID and CPUID):
	*
	* Bits:   |11 10 9 8|7 6 5 4 3 2|1 0
	*         | CLUSTER | Reserved  |CPU
	*/
	phys_cpu = cpu_logical_map(cpu);
	cluster = (phys_cpu / 4) << 8;
	phys_cpu = cluster + (phys_cpu % 4);

	if (pen_release == phys_cpu) {
		/*
		 * OK, proper wakeup, we're done
		 */
		break;
	}

	/*
	 * Getting here, means that we have come out of WFI without
	 * having been woken up - this shouldn't happen
	 *
	 * Just note it happening - when we're woken, we can report
	 * its occurrence.
	 */
	(*spurious)++;
	}
}
#endif

int axxia_platform_cpu_kill(unsigned int cpu)
{

#ifdef CONFIG_HOTPLUG_CPU_COMPLETE_POWER_DOWN
	get_cpu();
	pm_cpu_shutdown(cpu);
	put_cpu();
#endif
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */

void axxia_platform_cpu_die(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU_COMPLETE_POWER_DOWN
	bool last_cpu;

	last_cpu = pm_cpu_last_of_cluster(cpu);
	if (last_cpu)
		pm_L2_logical_shutdown(cpu);
	else
		pm_cpu_logical_shutdown(cpu);

	for (;;)
		wfi();

#else /* CPU low power mode */

	int spurious = 0;

	/*
	 * we're ready for shutdown now, so do it
	 */
	cpu_enter_lowpower_a15();
	pm_in_progress[cpu] = true;

	platform_do_lowpower(cpu, &spurious);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
	cpu_leave_lowpower();

	if (spurious)
		pr_warn("CPU%u: %u spurious wakeup calls\n", cpu, spurious);
#endif

}

int platform_cpu_disable(unsigned int cpu)
{

	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */

	return cpu == 0 ? -EPERM : 0;
}
