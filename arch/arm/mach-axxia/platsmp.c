/*
 * linux/arch/arm/mach-axxia/platsmp.c
 *
 * Copyright (C) 2012 LSI Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/mach/map.h>
#include <asm/virt.h>

#include "axxia.h"
#include "lsi_power_management.h"
#include <mach/axxia-gic.h>

#define SYSCON_PHYS_ADDR 0x002010030000ULL

static int  wfe_fixup;
static int wfe_available;

void __iomem *syscon;

inline void
__axxia_arch_wfe(void)
{
	if (0 != wfe_available)
		wfe();

	return;
}
EXPORT_SYMBOL(__axxia_arch_wfe);

/*
+ * Check if we need to enable cross-cluster SEV workaround for a bug in
+ * revision 1.0 silicon (which could cause event signals (from SEV instruction)
+ * to get lost between clusters). As a workaround, we set the bit 7 in CP15
+ * ACTLR (enable WFE as a no-operation) for each core.
 */
static void __init check_fixup_sev(void __iomem *syscon)
{
	u32 pfuse = readl(syscon + 0x34);
	u32 chip_type = pfuse & 0x1f;
	u32 chip_ver  = (pfuse >> 5) & 0x3f;

	/* Set flag for secondary cores */
	wfe_fixup = (chip_type == 0 || chip_type == 9) && (chip_ver == 0);
	pr_info("axxia: Cross-cluster SEV fixup: %s\n", wfe_fixup ? "yes" : "no");
}

static void  do_fixup_sev(void)
{
	u32 tmp;

	if (wfe_fixup) {
		asm volatile("mrc\tp15, 0, %0, c1, c0, 1" : "=r"(tmp));
		tmp |= (1<<7);
		asm volatile("mcr\tp15, 0, %0, c1, c0, 1" :: "r"(tmp));
		isb();
	}
}

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void  write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

static DEFINE_RAW_SPINLOCK(boot_lock);

void  axxia_secondary_init(unsigned int cpu)
{
	int phys_cpu;
	int phys_cluster;

	phys_cpu = cpu_logical_map(cpu);
	phys_cluster = phys_cpu / 4;

	/*
	 * Only execute this when powering up a cpu for hotplug.
	 */
	if (!pm_in_progress[phys_cpu]) {
		/* Fixup for cross-cluster SEV */
		do_fixup_sev();

		axxia_gic_secondary_init();
	} else {

#ifdef CONFIG_HOTPLUG_CPU_COMPLETE_POWER_DOWN
		if (cluster_power_up[phys_cluster])
			pm_cluster_logical_powerup();
		pm_cpu_logical_powerup();
#endif
		get_cpu();
		axxia_gic_secondary_init();
		put_cpu();

#ifdef CONFIG_HOTPLUG_CPU_COMPLETE_POWER_DOWN
		cluster_power_up[phys_cluster] = false;
		pm_in_progress[phys_cpu] = false;
#endif
	}

	/*
	 * Let the primary processor know we're out of the
	 * pen, then head off into the C entry point.
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	_raw_spin_lock(&boot_lock);
	_raw_spin_unlock(&boot_lock);
}

int  axxia_boot_secondary(unsigned int cpu, struct task_struct *idle)
{

	int phys_cpu, cluster;
	unsigned long timeout;
	unsigned long powered_down_cpu;
	u32 i;
	u32 dummy;


	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one.
	 */
	_raw_spin_lock(&boot_lock);

	phys_cpu = cpu_logical_map(cpu);

	powered_down_cpu = pm_get_powered_down_cpu();

	if (powered_down_cpu & (1 << phys_cpu)) {
		pm_in_progress[phys_cpu] = true;
		pm_cpu_powerup(phys_cpu);
	}

	/*
	 * In the Axxia, the bootloader does not put the secondary cores
	 * into a wait-for-event (wfe) or wait-for-interrupt (wfi) state
	 * because of the multi-cluster design (i.e., there's no way for
	 * the primary core in cluster 0 to send an event or interrupt
	 * to secondary cores in the other clusters).
	 *
	 * Instead, the secondary cores are immediately put into a loop
	 * that polls the "pen_release" global and MPIDR register. The two
	 * are compared and if they match, a secondary core then executes
	 * the Axxia secondary startup code.
	 *
	 * Here we convert the "cpu" variable to be compatible with the
	 * ARM MPIDR register format (CLUSTERID and CPUID):
	 *
	 * Bits:   |11 10 9 8|7 6 5 4 3 2|1 0
	 *         | CLUSTER | Reserved  |CPU
	 */
	cluster = (phys_cpu / 4) << 8;
	phys_cpu = cluster + (phys_cpu % 4);

	/* Release the specified core */
	write_pen_release(phys_cpu);

	/* Send a wakeup IPI to get the idled cpu out of WFI state */
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));


	/* Wait for so long, then give up if nothing happens ... */
	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();

		if (pen_release == -1)
			break;

		/* Wait 10 cycles */
		for (i = 0; i < 10; i++)
			dummy = i;
	}

	/*
	 * Now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish.
	 */
	_raw_spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

static __init struct device_node *get_cpu_node(int cpu)
{
	struct device_node *np;

	for_each_node_by_type(np, "cpu") {
		u32 reg;
		if (of_property_read_u32(np, "reg", &reg))
			continue;
		if (reg == cpu_logical_map(cpu))
			return np;
	}

	return NULL;
}

static void __init axxia_smp_prepare_cpus(unsigned int max_cpus)
{
	int cpu_count = 0;
	int cpu;

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return;

	check_fixup_sev(syscon);
	do_fixup_sev();

	if (of_find_compatible_node(NULL, NULL,
				    "lsi,axm5500-sim") != NULL ||
	    of_find_compatible_node(NULL, NULL,
				    "lsi,axm5500-emu") != NULL)
		wfe_available = 0;
	else
		wfe_available = 1;


	/*
	 * Initialise the present map, which describes the set of CPUs actually
	 * populated at the present time.
	 */
	for_each_possible_cpu(cpu) {
		struct device_node *np;
		u32 release_phys;
		u32 *release_virt;

		np = get_cpu_node(cpu);
		if (!np)
			continue;
		if (of_property_read_u32(np, "cpu-release-addr", &release_phys))
			continue;

		/*
		 * Release all physical cpus since we might want to
		 * bring them online later.
		 */
		if (cpu != 0) {
			u32 phys_cpu = cpu_logical_map(cpu);
			u32 tmp = readl(syscon + 0x1010);
			writel(0xab, syscon + 0x1000);
			tmp &= ~(1 << phys_cpu);
			writel(tmp, syscon + 0x1010);
		}

		if (cpu_count < max_cpus) {
			set_cpu_present(cpu, true);
			cpu_count++;
		}

		/*
		 * This is the entry point of the routine that the secondary
		 * cores will execute once they are released from their
		 * "holding pen".
		 */
		if (release_phys != 0) {
			int is_kmapped = pfn_valid(__phys_to_pfn(release_phys));

			if (is_kmapped)
				release_virt = phys_to_virt(release_phys);
			else
				release_virt = ioremap(release_phys, PAGE_SIZE);

			writel_relaxed(virt_to_phys(axxia_secondary_startup),
				       release_virt);
			smp_wmb();
			__cpuc_flush_dcache_area(release_virt, sizeof(u32));

			if (!is_kmapped)
				iounmap(release_virt);
		}
	}
}

struct smp_operations axxia_smp_ops __initdata = {
	.smp_prepare_cpus	= axxia_smp_prepare_cpus,
	.smp_secondary_init	= axxia_secondary_init,
	.smp_boot_secondary	= axxia_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= axxia_platform_cpu_die,
	.cpu_kill		= axxia_platform_cpu_kill,
#endif

};
