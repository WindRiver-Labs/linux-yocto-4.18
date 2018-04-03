/*
 *  Copyright (C) 2013 INTEL Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/reboot.h>
#include <linux/syscore_ops.h>
#include <linux/proc_fs.h>
#include <linux/prefetch.h>
#include <linux/delay.h>

#include <linux/of.h>
#include <linux/io.h>
#include <linux/axxia-ncr.h>
#include <asm/cacheflush.h>
#include "axxia.h"

static void __iomem *nca;
static void __iomem *apb;
static int ddr_retention_enabled;

enum {
	AXXIA_ENGINE_CAAL,
	AXXIA_ENGINE_CNAL
};

unsigned long
ncp_caal_regions_acp55xx[] = {
	NCP_REGION_ID(0x0b, 0x05),      /* SPPV2   */
	NCP_REGION_ID(0x0c, 0x05),      /* SED     */
	NCP_REGION_ID(0x0e, 0x05),      /* DPI_HFA */
	NCP_REGION_ID(0x14, 0x05),      /* MTM     */
	NCP_REGION_ID(0x14, 0x0a),      /* MTM2    */
	NCP_REGION_ID(0x15, 0x00),      /* MME     */
	NCP_REGION_ID(0x16, 0x05),      /* NCAV2   */
	NCP_REGION_ID(0x16, 0x10),      /* NCAV22  */
	NCP_REGION_ID(0x17, 0x05),      /* EIOAM1  */
	NCP_REGION_ID(0x19, 0x05),      /* TMGR    */
	NCP_REGION_ID(0x1a, 0x05),      /* MPPY    */
	NCP_REGION_ID(0x1a, 0x23),      /* MPPY2   */
	NCP_REGION_ID(0x1a, 0x21),      /* MPPY3   */
	NCP_REGION_ID(0x1b, 0x05),      /* PIC     */
	NCP_REGION_ID(0x1c, 0x05),      /* PAB     */
	NCP_REGION_ID(0x1f, 0x05),      /* EIOAM0  */
	NCP_REGION_ID(0x31, 0x05),      /* ISB     */
	NCP_REGION_ID(0xff, 0xff)
};

unsigned long
ncp_cnal_regions_acp55xx[] = {
	NCP_REGION_ID(0x28, 0x05),      /* EIOASM0 */
	NCP_REGION_ID(0x29, 0x05),      /* EIOASM1 */
	NCP_REGION_ID(0x2a, 0x05),      /* EIOAS2  */
	NCP_REGION_ID(0x2b, 0x05),      /* EIOAS3  */
	NCP_REGION_ID(0x2c, 0x05),      /* EIOAS4  */
	NCP_REGION_ID(0x2d, 0x05),      /* EIOAS5  */
	NCP_REGION_ID(0x32, 0x05),      /* ISBS    */
	NCP_REGION_ID(0xff, 0xff)
};

static void
quiesce_vp_engine(int engine_type)
{
	unsigned long *engine_regions;
	unsigned long ort_off, owt_off;
	unsigned long *region;
	unsigned ort, owt;
	unsigned long buf = 0;
	unsigned short node, target;
	int loop;

	pr_info("quiescing VP engines...\n");

	switch (engine_type) {
	case AXXIA_ENGINE_CNAL:
		engine_regions = ncp_cnal_regions_acp55xx;
		ort_off = 0x1c0;
		owt_off = 0x1c4;
		break;

	case AXXIA_ENGINE_CAAL:
		engine_regions = ncp_caal_regions_acp55xx;
		ort_off = 0xf8;
		owt_off = 0xfc;
		break;

	default:
		return;
	}

	region = engine_regions;

	while (*region != NCP_REGION_ID(0xff, 0xff)) {
		/* set read/write transaction limits to zero */
		ncr_write_nolock(*region, 0x8, 4, &buf);
		ncr_write_nolock(*region, 0xc, 4, &buf);
		region++;
	}

	region = engine_regions;
	loop = 0;

	while (*region != NCP_REGION_ID(0xff, 0xff)) {
		node = (*region & 0xffff0000) >> 16;
		target = *region & 0x0000ffff;
		/* read the number of outstanding read/write transactions */
		ncr_read_nolock(*region, ort_off, 4, &ort);
		ncr_read_nolock(*region, owt_off, 4, &owt);

		if ((ort == 0) && (owt == 0)) {
			/* this engine has been quiesced, move on to the next */
			pr_info("quiesced region 0x%02x.0x%02x\n",
					node, target);
			region++;
		} else {
			if (loop++ > 10000) {
				pr_info(
						"Unable to quiesce region 0x%02x.0x%02x ort=0x%x, owt=0x%x\n",
						node, target, ort, owt);
				region++;
				loop = 0;
				continue;
			}
		}
	}

	return;
}

static inline void cpu_disable_l2_prefetch(void)
{
	unsigned int v;

    /*
     * MRC p15, 1, <Rt>, c15, c0, 3; Read L2 Prefetch Control Register
     * MCR p15, 1, <Rt>, c15, c0, 3; Write L2 Prefetch Control Register
     *
     */
	asm volatile(
	"       mrc     p15, 1, %0, c15, c0, 3\n"
	"       and     %0, %0, #0x0000\n"
	"       mcr     p15, 1, %0, c15, c0, 3\n"
	: "=&r" (v)
	:
	: "cc");

	isb();
}

static inline void
reset_elm_trace(void)
{
	/* reset and disable ELM trace */
	ncr_register_write(htonl(0x000fff04), (unsigned *) (apb + 0x68000));
	ncr_register_write(htonl(0x000fff04), (unsigned *) (apb + 0x78000));

	/* reset ELM statistics */
	ncr_register_write(htonl(0x00001), (unsigned *) (apb + 0x60230));
	ncr_register_write(htonl(0x00001), (unsigned *) (apb + 0x70230));

	/* enable ELM trace */
	ncr_register_write(htonl(0x000fff01), (unsigned *) (apb + 0x68000));
	ncr_register_write(htonl(0x000fff01), (unsigned *) (apb + 0x78000));
}


/*
 * shutdown the system in preparation for a DDR retention reset.
 * This is only needed if initiating the retention reset while the
 * system is running in normal state (i.e. via the /proc filesystem.)
 * If the retention reset is called from within a restart function
 * this should not be necessary.
 */
void
retention_reset_prepare(void)
{
	/*
	 * If the axxia device is in reset then DDR retention is not
	 * possible. Just do an emergency_restart instead.
	 */

#ifdef CONFIG_ARCH_AXXIA_NCR_RESET_CHECK
	if (ncr_reset_active)
		emergency_restart();
#endif	/* CONFIG_ARCH_AXXIA_NCR_RESET_CHECK */

	preempt_disable();

	/* send stop message to other CPUs */
	local_irq_disable();
	local_fiq_disable();
	asm volatile ("dsb" : : : "memory");
	asm volatile ("dmb" : : : "memory");
	system_state = SYSTEM_RESTART;
	smp_send_stop();
	udelay(1000);
}

#define __flush_tlb_ID(void) \
	 asm("mcr p15, 0, %0, c8, c7, 0" \
	     : : "r" (0) : "cc")

static inline void flush_tlb_ID(void)
{
	__flush_tlb_ID();
	dsb();
	isb();
}

static void exercise_stack_ptr(volatile char *recursions)
{
	char stack_var[1024];
	volatile char *p;
	int i;

	p = stack_var;

	for (i = 0; i < 1024; i++)
		*p++ += *recursions;

	if (*recursions > 0) {
		*recursions -= 1;
		exercise_stack_ptr(recursions);
	}

}

void
initiate_retention_reset(void)
{
	unsigned long ctl_244 = 0;
	unsigned long value;
	unsigned cpu_id;
    /*
     * in order to preload the DDR shutdown function into cache
     * we use these variables to do a word-by-word copy of the
     * memory where the function resides. The 'tmp' variable
     * must be declared as volatile to ensure the compiler
     * doesn't optimize this out.
     * Removal of this volatile to resolve the checkpatch warning
     * will break the operation!
     */
	volatile long tmp;
	long *ptmp;
	char recursions = 4;

	if (0 == ddr_retention_enabled) {
		pr_info("DDR Retention Reset is Not Enabled\n");
		return;
	}

	if (NULL == nca || NULL == apb)
		BUG();

	preempt_disable();

	/* TODO - quiesce VP engines */
	quiesce_vp_engine(AXXIA_ENGINE_CAAL);
	quiesce_vp_engine(AXXIA_ENGINE_CNAL);


	/* unlock reset register for later */
	writel(0x000000ab, apb + 0x31000); /* Access Key */

	/* prepare to put DDR in self refresh power-down mode */
	/* first read the CTL_244 register and OR in the LP_CMD value */
	ncr_read_nolock(NCP_REGION_ID(34, 0), 0x3d0, 4, &ctl_244);
	ctl_244 |= 0x000a0000;

	/* belts & braces: put secondary CPUs into reset */
	cpu_id = smp_processor_id();
	value = ~(1 << cpu_id);
	value &= 0xffff;
	flush_cache_all();
	flush_l3();
	asm volatile ("isb" : : : "memory");
	flush_tlb_ID();
	ncr_register_write(htonl(value), (unsigned *) (apb + 0x31030));

	exercise_stack_ptr((volatile char *)&recursions);
	/* load entire ddr_shutdown function into L2 cache */
	ptmp = (long *) ncp_ddr_shutdown;
	do {
		tmp += *ptmp++;
	} while (ptmp < (long *) (ncp_ddr_shutdown + 0x1000));

	isb();

	/* disable L2 prefetching */
	cpu_disable_l2_prefetch();

	/* reset ELM DDR access trace buffer */
	reset_elm_trace();

	/* call cache resident ddr shutdown function */
	ncp_ddr_shutdown(nca, apb, ctl_244);

	return;
}
EXPORT_SYMBOL(initiate_retention_reset);

static ssize_t
axxia_ddr_retention_trigger(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	retention_reset_prepare();
	initiate_retention_reset();
	return 0;
}

static const struct file_operations axxia_ddr_retention_proc_ops = {
	.write      = axxia_ddr_retention_trigger,
	.llseek     = noop_llseek,
};

void
axxia_ddr_retention_init(void)
{
	/*
	* Only available on ASIC systems.
	*/

	if (of_find_compatible_node(NULL, NULL, "axxia,axm5500-amarillo")) {
		/* Create /proc entry. */
		if (!proc_create("driver/axxia_ddr_retention_reset",
				S_IWUSR, NULL, &axxia_ddr_retention_proc_ops)) {
			pr_info("Failed to register DDR retention proc entry\n");
		} else {
			apb = ioremap(0x2010000000, 0x80000);
			nca = ioremap(0x002020100000ULL, 0x20000);
			ddr_retention_enabled = 1;
			pr_info("DDR Retention Reset Initialized\n");
		}
	} else {
		pr_info("DDR Retention Reset is Not Available\n");
	}

	return;
}
