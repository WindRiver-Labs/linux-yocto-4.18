/*
 * arch/arm/mach-axxia/perf_event_platform.c
 *
 * Support for the LSI Axxia boards based on ARM cores.
 *
 * Copyright (C) 2014 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/bitmap.h>
#include <linux/cpu_pm.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <asm/cputype.h>
#include <asm/irq_regs.h>
#include <asm/pmu.h>

#include <linux/kthread.h>
#include <linux/sched.h>

#include <linux/cpu.h>
#include <linux/reboot.h>
#include <linux/syscore_ops.h>

#include <linux/proc_fs.h>

#include <linux/io.h>
#include <asm/cacheflush.h>
#include <../../../drivers/misc/lsi-ncr.h>

#include "perf_event_platform.h"

#include "smon.h"

/*
 * Include code for individual block support
 */

#include "perf_event_pcx.c"
#include "perf_event_vp.c"
#include "perf_event_memc.c"

/*
 * General platform perf code, muxed out to individual blocks
 */

int platform_pmu_event_idx(struct perf_event *event)
{
	return 0;
}

int platform_pmu_event_init(struct perf_event *event)
{
	uint64_t ev = event->attr.config;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if ((ev < AXM_55XX_PLATFORM_BASE) || (ev > AXM_55XX_PLATFORM_MAX))
		return -ENOENT;

	event->hw.config = ev - AXM_55XX_PLATFORM_BASE;

	event->hw.idx = -1;
	event->hw.config_base = 1;

/*
 if (event->group_leader != event) {
  printk("This is not the group leader!\n");
  printk("event->group_leader 0x%x\n", (unsigned int)event->group_leader);
 }
*/

	if (event->attr.exclude_user)
		return -ENOTSUPP;
	if (event->attr.exclude_kernel)
		return -ENOTSUPP;
	if (event->attr.exclude_idle)
		return -ENOTSUPP;

	event->hw.last_period = event->hw.sample_period;
	local64_set(&event->hw.period_left, event->hw.last_period);
/*
 event->destroy = hw_perf_event_destroy;
*/
	local64_set(&event->count, 0);

	if (ev >= AXM_55XX_VP_BASE && ev <= AXM_55XX_VP_MAX)
		vp_pmu_event_init(ev - AXM_55XX_VP_BASE, event);
	else if (ev >= AXM_55XX_PCX_BASE && ev <= AXM_55XX_PCX_MAX)
		pcx_pmu_event_init(ev - AXM_55XX_PCX_BASE, event);
	else if (ev >= AXM_55XX_MEMC_BASE && ev <= AXM_55XX_MEMC_MAX)
		memc_pmu_event_init(ev - AXM_55XX_MEMC_BASE, event);
	else
		pr_info("Platform perf, undefined event, %llu\n", ev);

	return 0;
}

static int platform_pmu_event_add(struct perf_event *event, int flags)
{
	uint64_t ev = event->attr.config;

	if (ev >= AXM_55XX_VP_BASE && ev <= AXM_55XX_VP_MAX)
		vp_pmu_event_add(ev - AXM_55XX_VP_BASE, event);
	else if (ev >= AXM_55XX_PCX_BASE && ev <= AXM_55XX_PCX_MAX)
		pcx_pmu_event_add(ev - AXM_55XX_PCX_BASE, event);
	else if (ev >= AXM_55XX_MEMC_BASE && ev <= AXM_55XX_MEMC_MAX)
		memc_pmu_event_add(ev - AXM_55XX_MEMC_BASE, event);

	return 0;
}

static void platform_pmu_event_del(struct perf_event *event, int flags)
{
	uint64_t ev = event->attr.config;
	uint32_t n;

	if (ev >= AXM_55XX_VP_BASE && ev <= AXM_55XX_VP_MAX) {
		n = vp_pmu_event_del(ev - AXM_55XX_VP_BASE, event, flags);
		local64_add(n, &event->count);
	} else if (ev >= AXM_55XX_PCX_BASE && ev <= AXM_55XX_PCX_MAX) {
		n = pcx_pmu_event_del(ev - AXM_55XX_PCX_BASE, event, flags);
		local64_add(n, &event->count);
	} else if (ev >= AXM_55XX_MEMC_BASE && ev <= AXM_55XX_MEMC_MAX) {
		n = memc_pmu_event_del(ev - AXM_55XX_MEMC_BASE, event, flags);
		local64_add(n, &event->count);
	} else {
		local64_set(&event->count, 0);
	}
}

static void platform_pmu_event_start(struct perf_event *event, int flags)
{
}

static void platform_pmu_event_stop(struct perf_event *event, int flags)
{
}

static void platform_pmu_event_read(struct perf_event *event)
{
	uint64_t ev = event->attr.config;
	uint32_t n;

	if (ev >= AXM_55XX_VP_BASE && ev <= AXM_55XX_VP_MAX) {
		n = vp_pmu_event_read(ev - AXM_55XX_VP_BASE, event, 0);
		local64_add(n, &event->count);
	} else if (ev >= AXM_55XX_PCX_BASE && ev <= AXM_55XX_PCX_MAX) {
		n = pcx_pmu_event_read(ev - AXM_55XX_PCX_BASE, event, 0);
		local64_add(n, &event->count);
	} else if (ev >= AXM_55XX_MEMC_BASE && ev <= AXM_55XX_MEMC_MAX) {
		n = memc_pmu_event_read(ev - AXM_55XX_MEMC_BASE, event, 0);
		local64_add(n, &event->count);
	}
}

/*
 * Device
 */

static void axmperf_device_release(struct device *dev)
{
	pr_warn("AXM55xxPlatformPerf release device\n");
}

static struct platform_device axmperf_device = {
	.name = "AXM55xxPlatformPerf",
	.id = 0,
	.dev = {
		.release = axmperf_device_release,
		},
};

/*
 * Driver
 */

#define PLATFORM_PMU_NAME_LEN 32

struct lsi_platform_pmu {
	struct pmu pmu;
	char name[PLATFORM_PMU_NAME_LEN];
};

static int axmperf_probe(struct platform_device *dev)
{
	int ret;
	struct lsi_platform_pmu *axm_pmu;

	axm_pmu = kzalloc(sizeof(struct lsi_platform_pmu), GFP_KERNEL);
	if (!axm_pmu) {
		pr_warn("Failed platform perf memory alloc!\n");
		return -ENOMEM;
	}

	axm_pmu->pmu = (struct pmu) {
		.attr_groups = 0,
		.event_init = platform_pmu_event_init,
		.add = platform_pmu_event_add,
		.del = platform_pmu_event_del,
		.start = platform_pmu_event_start,
		.stop = platform_pmu_event_stop,
		.read = platform_pmu_event_read,
		.event_idx = platform_pmu_event_idx,
	};

	sprintf(axm_pmu->name, "LSI AXM55xx Platform");

	ret = perf_pmu_register(&axm_pmu->pmu, axm_pmu->name, PERF_TYPE_RAW);

	if (ret == 0)
		pr_info("axxia platform perf enabled\n");
	else
		pr_info("axxia platform perf failed\n");

	vp_startup_init();
	pcx_startup_init();
	memc_startup_init();

	return ret;
}

static const struct of_device_id lsi_platformperf_match[] = {
	{ .compatible = "lsi,axm-platformperf", },
	{},
};

static struct platform_driver axmperf_driver = {
	.driver = {
		.name = "AXM55xxPlatformPerf",
		.of_match_table = lsi_platformperf_match,
		.owner = THIS_MODULE,
		},
	.probe = axmperf_probe,
};

static int __init axmperf_init(void)
{
	platform_driver_register(&axmperf_driver);

	return 0;
}

static void __exit axmperf_exit(void)
{
	pr_warn("AXM55xx platform perf exit!\n");
	platform_driver_unregister(&axmperf_driver);
	platform_device_unregister(&axmperf_device);
}

module_init(axmperf_init);
module_exit(axmperf_exit);
MODULE_LICENSE("GPL");
