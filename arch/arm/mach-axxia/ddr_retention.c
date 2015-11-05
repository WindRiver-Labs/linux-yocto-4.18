/*
 *  Copyright (C) 2013 LSI Corporation
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
#include <asm/io.h>
#include <asm/cacheflush.h>
#include "axxia.h"
#include "../../drivers/misc/lsi-ncr.h"

static void __iomem *nca;
static void __iomem *apb;
#ifndef CONFIG_SMP
void __iomem *dickens;
#endif

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


/*
  ------------------------------------------------------------------------------
  flush_l3

  This is NOT a general function to flush the L3 cache.  There are a number of
  assumptions that are not usually true...

  1) All other cores are " quiesced".
  2) There is no need to worry about preemption or interrupts.
*/

static void
flush_l3(void)
{

	unsigned long hnf_offsets[] = {
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
	};

	int i;
	unsigned long status;
	int retries;

	for (i = 0; i < (sizeof(hnf_offsets) / sizeof(unsigned long)); ++i)
		writel(0x0, dickens + (0x10000 * hnf_offsets[i]) + 0x10);

	for (i = 0; i < (sizeof(hnf_offsets) / sizeof(unsigned long)); ++i) {
		retries = 10000;

		do {
			status = readl(dickens +
				       (0x10000 * hnf_offsets[i]) + 0x18);
			udelay(1);
		} while ((0 < --retries) && (0x0 != (status & 0xf)));

		if (0 == retries)
			BUG();
	}

	for (i = 0; i < (sizeof(hnf_offsets) / sizeof(unsigned long)); ++i)
		writel(0x3, dickens + (0x10000 * hnf_offsets[i]) + 0x10);

	for (i = 0; i < (sizeof(hnf_offsets) / sizeof(unsigned long)); ++i) {
		retries = 10000;

		do {
			status = readl(dickens +
				       (0x10000 * hnf_offsets[i]) + 0x18);
			udelay(1);
		} while ((0 < --retries) && (0xc != (status & 0xf)));

		if (0 == retries)
			BUG();
	}

	asm volatile ("dsb" : : : "memory");

}

static void
quiesce_vp_engine(int engineType)
{
	unsigned long *pEngineRegions;
	unsigned long ortOff, owtOff;
	unsigned long *pRegion;
	unsigned ort, owt;
	unsigned long buf = 0;
	unsigned short node, target;
	int loop;

	pr_info("quiescing VP engines...\n");

	switch (engineType) {
	case AXXIA_ENGINE_CNAL:
		pEngineRegions = ncp_cnal_regions_acp55xx;
		ortOff = 0x1c0;
		owtOff = 0x1c4;
		break;

	case AXXIA_ENGINE_CAAL:
		pEngineRegions = ncp_caal_regions_acp55xx;
		ortOff = 0xf8;
		owtOff = 0xfc;
		break;

	default:
		return;
	}

	pRegion = pEngineRegions;

	while (*pRegion != NCP_REGION_ID(0xff, 0xff)) {
		/* set read/write transaction limits to zero */
		ncr_write(*pRegion, 0x8, 4, &buf);
		ncr_write(*pRegion, 0xc, 4, &buf);
		pRegion++;
	}

	pRegion = pEngineRegions;
	loop = 0;

	while (*pRegion != NCP_REGION_ID(0xff, 0xff)) {
		node = (*pRegion & 0xffff0000) >> 16;
		target = *pRegion & 0x0000ffff;
		/* read the number of outstanding read/write transactions */
		ncr_read(*pRegion, ortOff, 4, &ort);
		ncr_read(*pRegion, owtOff, 4, &owt);

		if ((ort == 0) && (owt == 0)) {
			/* this engine has been quiesced, move on to the next */
			pr_info("quiesced region 0x%02x.0x%02x\n",
					node, target);
			pRegion++;
		} else {
			if (loop++ > 10000) {
				pr_info(
						"Unable to quiesce region 0x%02x.0x%02x ort=0x%x, owt=0x%x\n",
						node, target, ort, owt);
				pRegion++;
				loop = 0;
				continue;
			}
		}
	}
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


extern void ncp_ddr_shutdown(void *, void *,  unsigned long);


void
initiate_retention_reset(void)
{
	unsigned long ctl_244 = 0;
	unsigned long value;
	unsigned cpu_id;

	volatile long tmp;
	volatile long *ptmp;

	if (0 == ddr_retention_enabled) {
		pr_info("DDR Retention Reset is Not Enabled\n");
		return;
	}

	if (NULL == nca || NULL == apb || NULL == dickens)
		BUG();

	preempt_disable();
	cpu_id = smp_processor_id();

	/* send stop message to other CPUs */
	local_irq_disable();
	local_fiq_disable();
	asm volatile ("dsb" : : : "memory");
	asm volatile ("dmb" : : : "memory");
	system_state = SYSTEM_RESTART;
	smp_send_stop();
	udelay(1000);

	flush_cache_all();
	flush_l3();

	/* TODO - quiesce VP engines */
	quiesce_vp_engine(AXXIA_ENGINE_CAAL);
	quiesce_vp_engine(AXXIA_ENGINE_CNAL);


	/* unlock reset register for later */
	writel(0x000000ab, apb + 0x31000); /* Access Key */

	/* prepare to put DDR in self refresh power-down mode */
	/* first read the CTL_244 register and OR in the LP_CMD value */
	ncr_read(NCP_REGION_ID(34, 0), 0x3d0, 4, &ctl_244);
	ctl_244 |= 0x000a0000;

	/* belts & braces: put secondary CPUs into reset */
	value = ~(1 << cpu_id);
	value &= 0xffff;
	ncr_register_write(htonl(value), (unsigned *) (apb + 0x31030));

	/* load entire ddr_shutdown function into L2 cache */
	ptmp = (long *) ncp_ddr_shutdown;
	do {
		tmp += *ptmp++;
	} while (ptmp < (long *) (ncp_ddr_shutdown + 0x1000));

	asm volatile ("isb" : : : "memory");

	/* disable L2 prefetching */
	cpu_disable_l2_prefetch();

	/* reset ELM DDR access trace buffer */
	reset_elm_trace();

	/* call cache resident ddr shutdown function */
	ncp_ddr_shutdown(nca, apb, ctl_244);
}
EXPORT_SYMBOL(initiate_retention_reset);

static ssize_t
axxia_ddr_retention_trigger(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
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

	if (of_find_compatible_node(NULL, NULL, "lsi,axm5500-amarillo")) {
		/* Create /proc entry. */
		if (!proc_create("driver/axxia_ddr_retention_reset",
				S_IWUSR, NULL, &axxia_ddr_retention_proc_ops)) {
			pr_info("Failed to register DDR retention proc entry\n");
		} else {
			apb = ioremap(0x2010000000, 0x80000);
			nca = ioremap(0x002020100000ULL, 0x20000);
			dickens = ioremap(0x2000000000, 0x1000000);
			ddr_retention_enabled = 1;
			pr_info("DDR Retention Reset Initialized\n");
		}
	} else {
		pr_info("DDR Retention Reset is Not Available\n");
	}
}
