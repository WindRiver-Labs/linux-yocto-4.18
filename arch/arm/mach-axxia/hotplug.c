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

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/cp15.h>
#include "lsi_power_management.h"


extern volatile int pen_release;

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
				: "cc"); }


int axxia_platform_cpu_kill(unsigned int cpu)
{
	pm_cpu_shutdown(cpu);
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */

void axxia_platform_cpu_die(unsigned int cpu)
{

	pm_data pm_request;
	int rVal = 0;
	bool lastCpu;

	pm_request.cpu = cpu;
	pm_request.cluster = 0;


	lastCpu = pm_cpu_last_of_cluster(cpu);
	if (lastCpu)
		rVal = pm_cpul2_logical_die(&pm_request);
	else
		rVal = pm_cpu_logical_die(&pm_request);
	if (rVal)
		pr_err("CPU %d failed to die\n", cpu);

	for (;;)
		wfi();

}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
