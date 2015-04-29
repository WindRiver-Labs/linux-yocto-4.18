/*
 *  linux/arch/arm/mach-axxia/lsi_power_management.c
 *
 *  C *  Created on: Jun 19, 2014
 *      Author: z8cpaul
 *  opyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Created on: Jun 19, 2014
 *      Author: z8cpaul
 */
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <asm/exception.h>
#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/cp15.h>

#include "axxia.h"
#include <mach/axxia-gic.h>
#include "lsi_power_management.h"

#undef DEBUG_CPU_PM

#define PM_WAIT_TIME (10000)
#define MAX_CLUSTER  (4)
#define IPI_IRQ_MASK (0xFFFF)

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

bool pm_in_progress[16];
bool cluster_power_up[4];

static const u32 cluster_to_node[MAX_CLUSTER] = { DKN_CLUSTER0_NODE,
DKN_CLUSTER1_NODE,
DKN_CLUSTER2_NODE,
DKN_CLUSTER3_NODE };

static const u32 cluster_to_poreset[MAX_CLUSTER] = {
PORESET_CLUSTER0,
PORESET_CLUSTER1,
PORESET_CLUSTER2,
PORESET_CLUSTER3 };

static const u32 cluster_to_mask[MAX_CLUSTER] = {
		IPI0_MASK,
		IPI1_MASK,
		IPI2_MASK,
		IPI3_MASK
};

static const u32 ipi_register[MAX_IPI] = {
		NCP_SYSCON_MASK_IPI0,
		NCP_SYSCON_MASK_IPI1,
		NCP_SYSCON_MASK_IPI2,
		NCP_SYSCON_MASK_IPI3,
		NCP_SYSCON_MASK_IPI4,
		NCP_SYSCON_MASK_IPI5,
		NCP_SYSCON_MASK_IPI6,
		NCP_SYSCON_MASK_IPI7,
		NCP_SYSCON_MASK_IPI8,
		NCP_SYSCON_MASK_IPI9,
		NCP_SYSCON_MASK_IPI10,
		NCP_SYSCON_MASK_IPI11,
		NCP_SYSCON_MASK_IPI12,
		NCP_SYSCON_MASK_IPI13,
		NCP_SYSCON_MASK_IPI14,
		NCP_SYSCON_MASK_IPI15,
		NCP_SYSCON_MASK_IPI16,
		NCP_SYSCON_MASK_IPI17,
		NCP_SYSCON_MASK_IPI18
};

enum pm_error_code {
	PM_ERR_DICKENS_IOREMAP = 200,
	PM_ERR_DICKENS_SNOOP_DOMAIN,
	PM_ERR_FAILED_PWR_DWN_RAM,
	PM_ERR_FAILED_STAGE_1,
	PM_ERR_ACK1_FAIL,
	PM_ERR_RAM_ACK_FAIL,
	PM_ERR_FAIL_L2ACK,
	PM_ERR_FAIL_L2HSRAM
};

u32 pm_cpu_powered_down;

/*======================= LOCAL FUNCTIONS ==============================*/
static void pm_set_bits_syscon_register(u32 reg, u32 data);
static void pm_or_bits_syscon_register(u32 reg, u32 data);
static void pm_clear_bits_syscon_register(u32 reg, u32 data);
static bool pm_test_for_bit_with_timeout(u32 reg, u32 bit);
static bool pm_wait_for_bit_clear_with_timeout(u32 reg, u32 bit);
static void pm_dickens_logical_shutdown(u32 cluster);
static int pm_dickens_logical_powerup(u32 cluster);
static int pm_cpu_physical_isolation_and_power_down(int cpu);
static void pm_L2_isolation_and_power_down(int cluster);
static int pm_cpu_physical_connection_and_power_up(int cpu);
static int pm_L2_physical_connection_and_power_up(u32 cluster);
static int pm_L2_logical_powerup(u32 cluster, u32 cpu);

static bool pm_first_cpu_of_cluster(u32 cpu)
{
#ifdef CONFIG_HOTPLUG_CPU_L2_POWER_DOWN

	u32 count = 0;

	switch (cpu) {
	case (0):
	case (1):
	case (2):
	case (3):
		/* This will never happen because cpu 0 will never be turned off */
		break;
	case (4):
	case (5):
	case (6):
	case (7):
		if (pm_cpu_powered_down & (1 << 4))
			count++;
		if (pm_cpu_powered_down & (1 << 5))
			count++;
		if (pm_cpu_powered_down & (1 << 6))
			count++;
		if (pm_cpu_powered_down & (1 << 7))
			count++;
		if (count == 4)
			return true;
		break;
	case (8):
	case (9):
	case (10):
	case (11):
		if (pm_cpu_powered_down & (1 << 8))
			count++;
		if (pm_cpu_powered_down & (1 << 9))
			count++;
		if (pm_cpu_powered_down & (1 << 10))
			count++;
		if (pm_cpu_powered_down & (1 << 11))
			count++;
		if (count == 4)
			return true;
		break;
	case (12):
	case (13):
	case (14):
	case (15):
		if (pm_cpu_powered_down & (1 << 12))
			count++;
		if (pm_cpu_powered_down & (1 << 13))
			count++;
		if (pm_cpu_powered_down & (1 << 14))
			count++;
		if (pm_cpu_powered_down & (1 << 15))
			count++;
		if (count == 4)
			return true;
		break;
	default:
		pr_err("ERROR: the cpu does not exist: %d - %s:%d\n", cpu, __FILE__,
				__LINE__);
		break;
	}
#endif
	return false;
}

bool pm_cpu_last_of_cluster(u32 cpu)
{
#ifdef CONFIG_HOTPLUG_CPU_L2_POWER_DOWN

	u32 count = 0;

	switch (cpu) {
	case (0):
	case (1):
	case (2):
	case (3):
		/* This will never happen because cpu 0 will never be turned off */
		break;
	case (4):
	case (5):
	case (6):
	case (7):
		if (pm_cpu_powered_down & (1 << 4))
			count++;
		if (pm_cpu_powered_down & (1 << 5))
			count++;
		if (pm_cpu_powered_down & (1 << 6))
			count++;
		if (pm_cpu_powered_down & (1 << 7))
			count++;
		if (count == 3)
			return true;
		break;
	case (8):
	case (9):
	case (10):
	case (11):
		if (pm_cpu_powered_down & (1 << 8))
			count++;
		if (pm_cpu_powered_down & (1 << 9))
			count++;
		if (pm_cpu_powered_down & (1 << 10))
			count++;
		if (pm_cpu_powered_down & (1 << 11))
			count++;
		if (count == 3)
			return true;
		break;
	case (12):
	case (13):
	case (14):
	case (15):
		if (pm_cpu_powered_down & (1 << 12))
			count++;
		if (pm_cpu_powered_down & (1 << 13))
			count++;
		if (pm_cpu_powered_down & (1 << 14))
			count++;
		if (pm_cpu_powered_down & (1 << 15))
			count++;
		if (count == 3)
			return true;
		break;
	default:
		pr_err("ERROR: the cpu does not exist: %d - %s:%d\n", cpu,  __FILE__,
				__LINE__);
		break;
	}
#endif
	return false;
}

static void pm_set_bits_syscon_register(u32 reg, u32 data)
{
	writel(data, syscon + reg);
}

static void pm_or_bits_syscon_register(u32 reg, u32 data)
{
	u32 tmp;

	tmp = readl(syscon + reg);
	tmp |= data;
	writel(tmp, syscon + reg);
}


static void pm_clear_bits_syscon_register(u32 reg, u32 data)
{
	u32 tmp;

	tmp = readl(syscon + reg);
	tmp &= ~(data);
	writel(tmp, syscon + reg);
}

static bool pm_test_for_bit_with_timeout(u32 reg, u32 bit)
{

	u32 tmp = 0;
	u32 cnt = 0;

	while (cnt < PM_WAIT_TIME) {
		tmp = readl(syscon + reg);
		if (CHECK_BIT(tmp, bit))
			break;
		cnt++;
	}
	if (cnt == PM_WAIT_TIME) {
		pr_err("reg=0x%x tmp:=0x%x\n", reg, tmp);
		return false;
	}
	return true;
}

static bool pm_wait_for_bit_clear_with_timeout(u32 reg, u32 bit)
{
	u32 cnt = 0;
	u32 tmp = 0;

	while (cnt < PM_WAIT_TIME) {
		tmp = readl(syscon + reg);
		if (!(CHECK_BIT(tmp, bit)))
			break;
		cnt++;
	}
	if (cnt == PM_WAIT_TIME) {
		pr_err("reg=0x%x tmp:=0x%x\n", reg, tmp);
		return false;
	}

	return true;
}
static void pm_dickens_logical_shutdown(u32 cluster)
{
	int i;
	int status;
	u32 bit;
	u32 bit_pos;
	int retries;

	bit = (0x01 << cluster_to_node[cluster]);
	bit_pos = cluster_to_node[cluster];

	for (i = 0; i < DKN_HNF_TOTAL_NODES; ++i) {
		writel(bit,
				dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
						+ DKN_HNF_SNOOP_DOMAIN_CTL_CLR);

		retries = PM_WAIT_TIME;

		do {
			status = readl(
					dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
							+ DKN_HNF_SNOOP_DOMAIN_CTL);
			udelay(1);
		} while ((0 < --retries) && CHECK_BIT(status, bit_pos));

		if (0 == retries) {
			pr_err("DICKENS: Failed to clear the SNOOP main control. LOOP:%d reg: 0x%x\n", i, status);
			return;

		}

	}
	/* Clear the domain cluster */
	writel(bit, dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET) + DKN_MN_DVM_DOMAIN_CTL_CLR);

	/* Check for complete */
	retries = PM_WAIT_TIME;

	do {
		status = readl(
				dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET)
						+ DKN_MN_DVM_DOMAIN_CTL);
		udelay(1);
	} while ((0 < --retries) && CHECK_BIT(status, bit_pos));

	if (0 == retries) {
		pr_err("DICKENS: failed to set DOMAIN OFFSET Reg=0x%x\n", status);
		return;

	}
}

static int pm_dickens_logical_powerup(u32 cluster)
{
	int i;
	u32 status;
	u32 bit;
	u32 bit_pos;
	int retries;
	int rval = 0;


	bit = (0x01 << cluster_to_node[cluster]);
	bit_pos = cluster_to_node[cluster];

	for (i = 0; i < DKN_HNF_TOTAL_NODES; ++i) {
		writel(bit,
				dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
						+ DKN_HNF_SNOOP_DOMAIN_CTL_SET);

		retries = PM_WAIT_TIME;

		do {
			status = readl(
					dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
							+ DKN_HNF_SNOOP_DOMAIN_CTL);
			udelay(1);
		} while ((0 < --retries) && !CHECK_BIT(status, bit_pos));

		if (0 == retries) {
			pr_err("DICKENS: Failed on the SNOOP DONAIN\n");
			return -PM_ERR_DICKENS_SNOOP_DOMAIN;
		}

	}

	/* Clear the domain cluster */
	writel(bit, dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET) + DKN_MN_DVM_DOMAIN_CTL_SET);

	/* Check for complete */
	retries = PM_WAIT_TIME;

	do {
		status = readl(
				dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET)
						+ DKN_MN_DVM_DOMAIN_CTL);
		udelay(1);
	} while ((0 < --retries) && !CHECK_BIT(status, bit_pos));

	if (0 == retries) {
		pr_err("DICKENS: Failed on the SNOOP DONAIN CTL SET\n");
		return -PM_ERR_DICKENS_SNOOP_DOMAIN;
	}

	return rval;
}

static void pm_disable_ipi_interrupts(u32 cpu)
{
	pm_clear_bits_syscon_register(ipi_register[cpu], IPI_IRQ_MASK);
}

static void pm_enable_ipi_interrupts(u32 cpu)
{

	u32 i;
	u32 powered_on_cpu = (~(pm_cpu_powered_down) & IPI_IRQ_MASK);

	pm_set_bits_syscon_register(ipi_register[cpu], powered_on_cpu);

	for (i = 0; i < MAX_CPUS; i++) {
		if ((1 << i) & powered_on_cpu)
			pm_or_bits_syscon_register(ipi_register[i], (1 << cpu));
	}
}

bool pm_cpu_active(u32 cpu)
{

	bool success = false;
	u32 reg;

	reg = readl(syscon + NCP_SYSCON_PWR_QACTIVE);
	if (reg & (1 << cpu))
		success = true;

	return success;

}

void pm_cpu_shutdown(u32 cpu)
{

	bool success;
	u32 reqcpu = cpu_logical_map(cpu);
	u32 cluster = reqcpu / CORES_PER_CLUSTER;
	u32 cluster_mask = (0x01 << cluster);
	bool last_cpu;
	int rval = 0;

	/* Check to see if the cpu is powered up */
	if (pm_cpu_powered_down & (1 << reqcpu)) {
		pr_err("CPU %d is already powered off - %s:%d\n", cpu, __FILE__, __LINE__);
		return;
	}

	/*
	 * Is this the last cpu of a cluster then turn off the L2 cache
	 * along with the CPU.
	 */
	last_cpu = pm_cpu_last_of_cluster(reqcpu);
	if (last_cpu) {

		/* Disable all the interrupts to the cluster gic */
		pm_or_bits_syscon_register(NCP_SYSCON_GIC_DISABLE, cluster_mask);

		/* Remove the cluster from the Dickens coherency domain */
		pm_dickens_logical_shutdown(cluster);

		/* Power down the cpu */
		pm_cpu_physical_isolation_and_power_down(reqcpu);

		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_CSYSREQ_CNT, cluster_mask);
		success = pm_wait_for_bit_clear_with_timeout(NCP_SYSCON_PWR_CACTIVE_CNT, cluster);
		if (!success) {
			pr_err(
					"Failed to keep other cluster count going on cluster %d: %s-%d\n",
					cluster, __FILE__, __LINE__);
			goto pm_shutdown_exit;
		}

		/* Turn off the ACE */
		pm_or_bits_syscon_register(NCP_SYSCON_PWR_ACEPWRDNRQ, cluster_mask);

		/* Wait for ACE to complete power off */
		success = pm_wait_for_bit_clear_with_timeout(NCP_SYSCON_PWR_NACEPWRDNACK, cluster);
		if (!success) {
			pr_err("Failed to power off ACE on cluster %d: %s-%d\n",
					cluster, __FILE__, __LINE__);
			goto pm_shutdown_exit;
		}

		/* Isolate the cluster */
		pm_or_bits_syscon_register(NCP_SYSCON_PWR_ISOLATEL2MISC, cluster_mask);

		/* Wait for WFI L2 to go to standby */
		success = pm_test_for_bit_with_timeout(NCP_SYSCON_PWR_STANDBYWFIL2, cluster);
		if (!success) {
			pr_err("Failed to enter L2 WFI on cluster %d: %s-%d\n",
					cluster, __FILE__, __LINE__);
			goto pm_shutdown_exit;
		}

		/* Power off the L2 */
		pm_L2_isolation_and_power_down(cluster);
		if (rval == 0) {
			pr_info("CPU %d is powered down with cluster: %d\n", reqcpu, cluster);
			pm_cpu_powered_down |= (1 << reqcpu);
		} else
			pr_err("CPU %d failed to power down\n", reqcpu);


	} else {

		rval = pm_cpu_physical_isolation_and_power_down(reqcpu);
		if (rval == 0)
			pm_cpu_powered_down |= (1 << reqcpu);
		else
			pr_err("CPU %d failed to power down\n", reqcpu);
	}

pm_shutdown_exit:
	return;

}

int pm_cpu_powerup(u32 cpu)
{

	bool first_cpu;
	int rval = 0;
	u32 cpu_mask = (0x01 << cpu);

	u32 reqcpu = cpu_logical_map(cpu);
	u32 cluster = reqcpu / CORES_PER_CLUSTER;

	u32 cluster_mask = (0x01 << cluster);

	/*
	 * Is this the first cpu of a cluster to come back on?
	 * Then power up the L2 cache.
	 */
	first_cpu = pm_first_cpu_of_cluster(cpu);
	if (first_cpu) {

		rval = pm_L2_logical_powerup(cluster, cpu);
		if (rval) {
			pr_err("CPU: Failed the logical L2 power up\n");
			goto pm_power_up;
		} else
			pr_info("CPU %d is powered up with cluster: %d\n", reqcpu, cluster);

		cluster_power_up[cluster] = true;
		pm_clear_bits_syscon_register(NCP_SYSCON_GIC_DISABLE, cluster_mask);


	} else {
		/* Set the CPU into reset */
		pm_set_bits_syscon_register(NCP_SYSCON_KEY, VALID_KEY_VALUE);
		pm_or_bits_syscon_register(NCP_SYSCON_PWRUP_CPU_RST, cpu_mask);
	}


	/*
	 * Power up the CPU
	 */
	rval = pm_cpu_physical_connection_and_power_up(cpu);
	if (rval) {
		pr_err("Failed to power up physical connection of cpu: %d\n", cpu);
		goto pm_power_up;
	}

	/*
	 * The key value must be written before the CPU RST can be written.
	 */
	pm_set_bits_syscon_register(NCP_SYSCON_KEY, VALID_KEY_VALUE);
	pm_clear_bits_syscon_register(NCP_SYSCON_PWRUP_CPU_RST,	cpu_mask);

	/*
	 * Clear the powered down mask
	 */
	pm_cpu_powered_down &= ~(1 << cpu);

	/* Enable the CPU IPI */
	pm_enable_ipi_interrupts(cpu);

pm_power_up:
	return rval;
}

unsigned long pm_get_powered_down_cpu(void)
{
	return pm_cpu_powered_down;
}

inline void pm_cpu_logical_powerup(void)
{
	unsigned int v;

	asm volatile(
			"	mrc	p15, 0, %0, c1, c0, 0\n"
			"	orr	%0, %0, %1\n"
			"	mcr	p15, 0, %0, c1, c0, 0\n"
			"	mrc	p15, 0, %0, c1, c0, 0\n"
			"	orr	%0, %0, %2\n"
			"	mcr	p15, 0, %0, c1, c0, 0\n"
			: "=&r" (v)
			: "Ir" (CR_C), "Ir" (CR_I)
			: "cc");

	/*
	 *  Iniitalize the ACTLR2 register (all cores).
	*/

	asm volatile(
			"	mrc		p15, 1, %0, c15, c0, 4\n"
			"	bic	%0, %0, %1\n"
			"	mcr		p15, 1, %0, c15, c0, 4\n"
			: "=&r" (v)
			: "Ir" (0x1)
			: "cc");

	isb();
	dsb();
}

inline void pm_cluster_logical_powerup(void)
{
	unsigned int v;

	/*
	 * Initialize the L2CTLR register (primary core in each cluster).
	 */
	asm volatile(
	"	mrc	p15, 1, %0, c9, c0, 2\n"
	"	orr	%0, %0, %1\n"
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 1, %0, c9, c0, 2"
	  : "=&r" (v)
	  : "Ir" (0x01), "Ir" (0x1 << 21)
	  : "cc");

	isb();
	dsb();

	/*
	 * Initialize the L2ACTLR register (primary core in each cluster).
	 */
	asm volatile(
		"	mrc	p15, 1, r0, c15, c0, 0\n"
		"	orr	%0, %0, %1\n"
		"	orr	%0, %0, %2\n"
		"	orr	%0, %0, %3\n"
		"	orr	%0, %0, %4\n"
		"	orr	%0, %0, %5\n"
		"	mcr	p15, 1, %0, c15, c0, 0"
		: "=&r" (v)
			: "Ir" (0x1 << 3), "Ir" (0x1 << 7), "Ir" (0x1 << 12), "Ir" (0x1 << 13), "Ir" (0x1 << 14)
		 : "cc");
	isb();
	dsb();
}

static int pm_cpu_physical_isolation_and_power_down(int cpu)
{
	int rval = 0;

	bool success;
	u32 mask = (0x01 << cpu);

	/* Disable the CPU IPI */
	pm_disable_ipi_interrupts(cpu);

	/* Initiate power down of the CPU's HS Rams */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPCPURAM, mask);

	/* Wait until the RAM power down is complete */
	success = pm_test_for_bit_with_timeout(NCP_SYSCON_PWR_NPWRUPCPURAM_ACK, cpu);
	if (!success) {
		rval = -PM_ERR_FAILED_PWR_DWN_RAM;
		pr_err("CPU: Failed to power down CPU RAM\n");
		goto power_down_cleanup;
	}

	/* Activate the CPU's isolation clamps */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_ISOLATECPU, mask);

	/* Initiate power down of the CPU logic */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPCPUSTG2, mask);

	udelay(16);

	/* Continue power down of the CPU logic */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPCPUSTG1, mask);

	success = pm_test_for_bit_with_timeout(NCP_SYSCON_PWR_NPWRUPCPUSTG1_ACK, cpu);
	if (!success) {
		rval = -PM_ERR_FAILED_STAGE_1;
		pr_err("CPU: Failed to power down stage 1 cpu\n");
		goto power_down_cleanup;
	}

power_down_cleanup:
	return rval;
}

static int pm_cpu_physical_connection_and_power_up(int cpu)
{
	int rval = 0;

	bool success;
	u32 mask = (0x01 << cpu);

	/* Initiate power up of the CPU */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_PWRUPCPUSTG1, mask);

	/* Wait until CPU logic power is compete */
	success = pm_wait_for_bit_clear_with_timeout(NCP_SYSCON_PWR_NPWRUPCPUSTG1_ACK, cpu);
	if (!success) {
		rval = -PM_ERR_ACK1_FAIL;
		pr_err("CPU: Failed to get ACK from power down stage 1\n");
		goto power_up_cleanup;
	}

	/* Continue stage 2 power up of the CPU*/
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_PWRUPCPUSTG2, mask);

	udelay(16);

	/* Initiate power up of HS Rams */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_PWRUPCPURAM, mask);

	/* Wait until the RAM power up is complete */
	success = pm_wait_for_bit_clear_with_timeout(NCP_SYSCON_PWR_NPWRUPCPURAM_ACK, cpu);
	if (!success) {
		rval = -PM_ERR_RAM_ACK_FAIL;
		pr_err("CPU: Failed to get ACK of power power up\n");
		goto power_up_cleanup;
	}

	/* Release the CPU's isolation clamps */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_ISOLATECPU, mask);

	udelay(16);

power_up_cleanup:

	return rval;

}
/*========================================== L2 FUNCTIONS ========================================*/

static void pm_L2_isolation_and_power_down(int cluster)
{
	u32 mask = (0x1 << cluster);

	/* Enable the chip select for the cluster */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_CHIPSELECTEN, mask);

	/* Disable the hsram */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL2HSRAM, mask);

	switch (cluster) {
	case (0):

#ifdef PM_POWER_OFF_ONLY_DATARAM
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);

#endif
		break;
	case (1):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (2):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (3):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	default:
		pr_err("Illegal cluster: %d > 3\n", cluster);
		break;
	}

	/* Power down stage 2 */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL2LGCSTG2, mask);

	/* Power down stage 1 */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL2LGCSTG1, mask);

}

static int pm_L2_physical_connection_and_power_up(u32 cluster)
{

	bool success;
	u32 mask = (0x1 << cluster);
	int rval = 0;

	/* Power up stage 1 */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL2LGCSTG1, mask);

	/* Wait for the stage 1 power up to complete */
	success = pm_wait_for_bit_clear_with_timeout(NCP_SYSCON_PWR_NPWRUPL2LGCSTG1_ACK, cluster);
	if (!success) {
		pr_err("CPU: Failed to ack the L2 Stage 1 Power up\n");
		rval = -PM_ERR_FAIL_L2ACK;
		goto power_up_l2_cleanup;
	}

	/* Power on stage 2 */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL2LGCSTG2, mask);

	/* Set the chip select */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_CHIPSELECTEN, mask);

	/* Power up the snoop ram */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL2HSRAM, mask);

	/* Wait for the stage 1 power up to complete */
	success = pm_wait_for_bit_clear_with_timeout(NCP_SYSCON_PWR_NPWRUPL2HSRAM_ACK, cluster);
	if (!success) {
		pr_err("CPU: failed to get the HSRAM power up ACK\n");
		rval = -PM_ERR_FAIL_L2HSRAM;
		goto power_up_l2_cleanup;
	}

	switch (cluster) {
	case (0):

#ifdef PM_POWER_OFF_ONLY_DATARAM
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);

#endif
		break;
	case (1):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
	udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (2):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (3):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_set_bits_syscon_register(NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	default:
		pr_err("Illegal cluster: %d > 3\n", cluster);
		break;
	}

	/* Clear the chip select */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_CHIPSELECTEN, mask);

	/* Release the isolation clamps */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_ISOLATEL2MISC, mask);

	/* Turn the ACE bridge power on*/
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_ACEPWRDNRQ, mask);

power_up_l2_cleanup:
	return rval;
}

static int pm_L2_logical_powerup(u32 cluster, u32 cpu)
{

	u32 mask = (0x1 << cluster);
	int rval = 0;
	u32 cluster_mask;

	if (cluster == 0)
		cluster_mask = 0xe;
	else
		cluster_mask = 0xf << (cluster * 4);

	/* put the cluster into a cpu hold */
	pm_or_bits_syscon_register(NCP_SYSCON_RESET_AXIS,
			cluster_to_poreset[cluster]);

	/*
	 * The key value has to be written before the CPU RST can be written.
	 */
	pm_set_bits_syscon_register(NCP_SYSCON_KEY, VALID_KEY_VALUE);
	pm_or_bits_syscon_register(NCP_SYSCON_PWRUP_CPU_RST, cluster_mask);

	/* Hold the chip debug cluster */
	pm_set_bits_syscon_register(NCP_SYSCON_KEY, VALID_KEY_VALUE);
	pm_or_bits_syscon_register(NCP_SYSCON_HOLD_DBG, mask);

	/* Hold the L2 cluster */
	pm_set_bits_syscon_register(NCP_SYSCON_KEY, VALID_KEY_VALUE);
	pm_or_bits_syscon_register(NCP_SYSCON_HOLD_L2, mask);


	/* Cluster physical power up */
	rval = pm_L2_physical_connection_and_power_up(cluster);
	if (rval)
		goto exit_pm_L2_logical_powerup;

	udelay(16);

	/* take the cluster out of a cpu hold */
	pm_clear_bits_syscon_register(NCP_SYSCON_RESET_AXIS,
			cluster_to_poreset[cluster]);

	udelay(64);

	/* Enable the system counter */
	pm_or_bits_syscon_register(NCP_SYSCON_PWR_CSYSREQ_CNT, mask);

	/* Release the L2 cluster */
	pm_set_bits_syscon_register(NCP_SYSCON_KEY, VALID_KEY_VALUE);
	pm_clear_bits_syscon_register(NCP_SYSCON_HOLD_L2, mask);

	/* Release the chip debug cluster */
	pm_set_bits_syscon_register(NCP_SYSCON_KEY, VALID_KEY_VALUE);
	pm_clear_bits_syscon_register(NCP_SYSCON_HOLD_DBG, mask);

	/* Power up the dickens */
	rval = pm_dickens_logical_powerup(cluster);
	if (rval)
		goto exit_pm_L2_logical_powerup;

	/* start L2 */
	pm_clear_bits_syscon_register(NCP_SYSCON_PWR_ACINACTM, mask);

exit_pm_L2_logical_powerup:

	return rval;

}

#ifdef DEBUG_CPU_PM

void pm_debug_read_pwr_registers(void)
{
	u32 reg;

	reg = readl(syscon + 0x1400);
	pr_err("NCP_SYSCON_PWR_CLKEN: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_PWR_ACINACTM);
	pr_err("NCP_SYSCON_PWR_ACINACTM: 0x%x\n", reg);
	reg = readl(syscon + 0x140c);
	pr_err("NCP_SYSCON_PWR_CHIPSELECTEN: 0x%x\n", reg);
	reg = readl(syscon + 0x1410);
	pr_err("NCP_SYSCON_PWR_CSYSREQ_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x1414);
	pr_err("NCP_SYSCON_PWR_CSYSREQ_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1418);
	pr_err("NCP_SYSCON_PWR_CSYSREQ_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x141c);
	pr_err("NCP_SYSCON_PWR_CSYSREQ_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1420);
	pr_err("NCP_SYSCON_PWR_PWRUPL2LGCSTG1: 0x%x\n", reg);
	reg = readl(syscon + 0x1424);
	pr_err("NCP_SYSCON_PWR_PWRUPL2LGCSTG2: 0x%x\n", reg);
	reg = readl(syscon + 0x1428);
	pr_err("NCP_SYSCON_PWR_PWRUPL2HSRAM: 0x%x\n", reg);
	reg = readl(syscon + 0x142c);
	pr_err("NCP_SYSCON_PWR_ACEPWRDNRQ: 0x%x\n", reg);
	reg = readl(syscon + 0x1430);
	pr_err("NCP_SYSCON_PWR_ISOLATEL2MIS: 0x%x\n", reg);
	reg = readl(syscon + 0x1438);
	pr_err("NCP_SYSCON_PWR_NPWRUPL2LGCSTG1_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x143c);
	pr_err("NCP_SYSCON_PWR_NPWRUPL2HSRAM_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1440);
	pr_err("NCP_SYSCON_PWR_STANDBYWFIL2: 0x%x\n", reg);
	reg = readl(syscon + 0x1444);
	pr_err("NCP_SYSCON_PWR_CSYSACK_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x1448);
	pr_err("NCP_SYSCON_PWR_CACTIVE_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x144c);
	pr_err("NCP_SYSCON_PWR_CSYSACK_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1450);
	pr_err("NCP_SYSCON_PWR_CACTIVE_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1454);
	pr_err("NCP_SYSCON_PWR_CSYSACK_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x1458);
	pr_err("NCP_SYSCON_PWR_CACTIVE_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x145c);
	pr_err("NCP_SYSCON_PWR_CSYSACK_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1460);
	pr_err("NCP_SYSCON_PWR_CACTIVE_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1464);
	pr_err("NCP_SYSCON_PWR_NACEPWRDNACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1468);
	pr_err("NCP_SYSCON_PWR_CACTIVEM_EAGM: 0x%x\n", reg);
	reg = readl(syscon + 0x146c);
	pr_err("NCP_SYSCON_PWR_CACTIVEM_EAGS: 0x%x\n", reg);
	reg = readl(syscon + 0x1470);
	pr_err("NCP_SYSCON_PWR_CACTIVES_EAGM: 0x%x\n", reg);
	reg = readl(syscon + 0x1474);
	pr_err("NCP_SYSCON_PWR_CACTIVES_EAGS: 0x%x\n", reg);
	reg = readl(syscon + 0x1480);
	pr_err("NCP_SYSCON_PWR_PWRUPCPUSTG1: 0x%x\n", reg);
	reg = readl(syscon + 0x1484);
	pr_err("NCP_SYSCON_PWR_PWRUPCPUSTG2: 0x%x\n", reg);
	reg = readl(syscon + 0x1488);
	pr_err("NCP_SYSCON_PWR_PWRUPCPURAM: 0x%x\n", reg);
	reg = readl(syscon + 0x148c);
	pr_err("NCP_SYSCON_PWR_ISOLATECPU: 0x%x\n", reg);
	reg = readl(syscon + 0x1490);
	pr_err("NCP_SYSCON_PWR_NPWRUPCPUSTG1_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1494);
	pr_err("NCP_SYSCON_PWR_NPWRUPCPURAM_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1498);
	pr_err("NCP_SYSCON_PWR_QACTIVE: 0x%x\n", reg);
	reg = readl(syscon + 0x149C);
	pr_err("NCP_SYSCON_PWR_STANDBYWFI: 0x%x\n", reg);
	reg = readl(syscon + 0x14A0);
	pr_err("NCP_SYSCON_PWR_STANDBYWFE: 0x%x\n", reg);
	reg = readl(syscon + 0x14A4);
	pr_err("NCP_SYSCON_PWR_DBGNOPWRDWN: 0x%x\n", reg);
	reg = readl(syscon + 0x14A8);
	pr_err("NCP_SYSCON_PWR_DBGPWRUPREQ: 0x%x\n", reg);
	reg = readl(syscon + 0x1040);
	pr_err("NCP_SYSCON_RESET_AXIS: 0x%x\n", reg);
	reg = readl(syscon + 0x1044);
	pr_err("NCP_SYSCON_RESET_AXIS-WORD1: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_RESET_CPU);
	pr_err("NCP_SYSCON_RESET_CPU: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_HOLD_DBG);
	pr_err("NCP_SYSCON_HOLD_DBG: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_HOLD_L2);
	pr_err("NCP_SYSCON_HOLD_L2: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_HOLD_CPU);
	pr_err("NCP_SYSCON_HOLD_CPU: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_PWRUP_CPU_RST);
	pr_err("NCP_SYSCON_PWRUP_CPU_RST: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_RESET_STATUS);
	pr_err("NCP_SYSCON_RESET_STATUS: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_RESET_CORE_STATUS);
	pr_err("NCP_SYSCON_RESET_CORE_STATUS: 0x%x\n", reg);


#if 0
	reg = readl(syscon + NCP_SYSCON_MCG_CSW_CPU);
	pr_err("NCP_SYSCON_MCG_CSW_CPU: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MCG_CSW_SYS);
	pr_err("NCP_SYSCON_MCG_CSW_SYS: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MCG_DIV_CPU);
	pr_err("NCP_SYSCON_MCG_DIV_CPU: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MCG_DIV_SYS);
	pr_err("NCP_SYSCON_MCG_DIV_SYS: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_CLKDEBUG);
	pr_err("NCP_SYSCON_CLKDEBUG: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_EVENT_ENB);
	pr_err("NCP_SYSCON_EVENT_ENB: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_CPU_FAST_INT);
	pr_err("NCP_SYSCON_CPU_FAST_INT: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_GIC_DISABLE);
	pr_err("NCP_SYSCON_GIC_DISABLE: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_CP15SDISABLE);
	pr_err("NCP_SYSCON_CP15SDISABLE: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_LDO_CTL);
	pr_err("NCP_SYSCON_LDO_CTL: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_SHWK_QOS);
	pr_err("NCP_SYSCON_SHWK_QOS: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_FUSE_RTO);
	pr_err("NCP_SYSCON_FUSE_RTO: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_PFUSE);
	pr_err("NCP_SYSCON_PFUSE: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_FUSE_STAT);
	pr_err("NCP_SYSCON_FUSE_STAT: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_SCRATCH);
	pr_err("NCP_SYSCON_SCRATCH: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI0);
	pr_err("NCP_SYSCON_MASK_IPI0: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI1);
	pr_err("NCP_SYSCON_MASK_IPI1: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI2);
	pr_err("NCP_SYSCON_MASK_IPI2: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI3);
	pr_err("NCP_SYSCON_MASK_IPI3: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI4);
	pr_err("NCP_SYSCON_MASK_IPI4: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI5);
	pr_err("NCP_SYSCON_MASK_IPI5: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI6);
	pr_err("NCP_SYSCON_MASK_IPI6: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI7);
	pr_err("NCP_SYSCON_MASK_IPI7: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI8);
	pr_err("NCP_SYSCON_MASK_IPI8: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI9);
	pr_err("NCP_SYSCON_MASK_IPI9: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI10);
	pr_err("NCP_SYSCON_MASK_IPI10: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI11);
	pr_err("NCP_SYSCON_MASK_IPI11: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI12);
	pr_err("NCP_SYSCON_MASK_IPI12: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI13);
	pr_err("NCP_SYSCON_MASK_IPI13: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI14);
	pr_err("NCP_SYSCON_MASK_IPI14: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_MASK_IPI15);
	pr_err("NCP_SYSCON_MASK_IPI15: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_SPARE0);
	pr_err("NCP_SYSCON_SPARE0: 0x%x\n", reg);
	reg = readl(syscon + NCP_SYSCON_STOP_CLK_CPU);
	pr_err("NCP_SYSCON_STOP_CLK_CPU: 0x%x\n", reg);
#endif


}


void pm_dump_L2_registers(void)
{
	u32 reg;


	reg = readl(syscon + 0x1580);
	pr_err("NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x1584);
	pr_err("NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x1588);
	pr_err("NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x158c);
	pr_err("NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x1590);
	pr_err("NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x1594);
	pr_err("NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x1598);
	pr_err("NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x159c);
	pr_err("NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x15a0);
	pr_err("NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x15a4);
	pr_err("NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x15a8);
	pr_err("NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x15ac);
	pr_err("NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0: 0x%x\n", reg);




void pm_dump_dickens(void)
{

	u32 status;
	u32 i;

	for (i = 0; i < DKN_HNF_TOTAL_NODES; ++i) {
		status = readl(
				dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
						+ DKN_HNF_SNOOP_DOMAIN_CTL);
		udelay(1);
		pr_err("DKN_HNF_SNOOP_DOMAIN_CTL[%d]: 0x%x\n", i, status);
	}

	status = readl(
			dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET)
					+ DKN_MN_DVM_DOMAIN_CTL);

	pr_err("DKN_MN_DVM_DOMAIN_CTL: 0x%x\n", status);
}

#endif
