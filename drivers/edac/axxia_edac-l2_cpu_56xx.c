/*
 * drivers/edac/axxia_edac-l2_cpu_56xx.c
 *
 * EDAC Driver for Intel's Axxia 5600 System Memory Controller
 *
 * Copyright (C) 2016 Intel Inc.
 *
 * This file may be distributed under the terms of the
 * GNU General Public License.
 */

#define CREATE_TRACE_POINTS

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/lsi-ncr.h>
#include <linux/edac.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <trace/events/edac.h>
#include "edac_core.h"
#include "edac_module.h"
#include "axxia_l2_56xx.h"


#if defined(CONFIG_EDAC_AXXIA_L2_CPU_5600)
#define INTEL_EDAC_MOD_STR     "axxia56xx_edac"
#endif

#if defined(CONFIG_EDAC_AXXIA_L2_CPU_6700)
#define INTEL_EDAC_MOD_STR     "axxia67xx_edac"
#endif

#define CORES_PER_CLUSTER 4

#define SYSCON_PERSIST_SCRATCH 0xdc
#define L2_PERSIST_SCRATCH_BIT (0x1 << 5)
#define CPU_PERSIST_SCRATCH_BIT (0x1 << 6)

/* Private structure for common edac device */
struct intel_edac_dev_info {
	struct platform_device *pdev;
	char *ctl_name;
	char *blk_name;
	int edac_idx;
	struct regmap *syscon;
	struct edac_device_ctl_info *edac_dev;
	void (*check)(struct edac_device_ctl_info *edac_dev);
};

void log_cpumerrsr(void *edac)
{
	struct edac_device_ctl_info *edac_dev = edac;
	u64 val, clear_val;
	u32 count0, count1;
	struct intel_edac_dev_info *dev_info;

	dev_info = edac_dev->pvt_info;

	/* Read S3_1_c15_c2_2 for CPUMERRSR_EL1 counts */
	val = read_cpumerrsr();
	trace_edac_l1cache_syndrome(val);

	if (val & 0x8000000000000000) {
		regmap_update_bits(dev_info->syscon,
				   SYSCON_PERSIST_SCRATCH,
				   CPU_PERSIST_SCRATCH_BIT,
				   CPU_PERSIST_SCRATCH_BIT);
		pr_emerg("CPU uncorrectable error\n");
		machine_restart(NULL);
	}

	if (val & 0x80000000) {
		int cpu = get_cpu();

		count0 = ((val) & 0x000000ff00000000) >> 32;
		count1 = ((val) & 0x0000ff0000000000) >> 40;

		/* increment correctable error counts */
		trace_edac_l1cache_counter(count0 + count1);

		if (count0 || count1)
			edac_device_handle_multi_ce(edac_dev, 0, cpu,
				count0 + count1, edac_dev->ctl_name);

		/* Clear the valid bit */
		clear_val = 0x80000000;
		write_cpumerrsr(clear_val);
		put_cpu();
	}
}


/* Check for CPU Errors */
static void intel_cpu_error_check(struct edac_device_ctl_info *edac_dev)
{
	/* execute on current cpu */
	log_cpumerrsr(edac_dev);

	/* send ipis to execute on other cpus */
	smp_call_function(log_cpumerrsr, edac_dev, 1);

}

void log_l2merrsr(void *edac)
{
	struct edac_device_ctl_info *edac_dev = edac;
	u64 val, clear_val;
	u32 count0, count1;
	struct intel_edac_dev_info *dev_info;

	dev_info = edac_dev->pvt_info;

	val = read_l2merrsr();
	trace_edac_l2cache_syndrome(val);
	if (val & 0x8000000000000000) {
		regmap_update_bits(dev_info->syscon,
				   SYSCON_PERSIST_SCRATCH,
				   L2_PERSIST_SCRATCH_BIT,
				   L2_PERSIST_SCRATCH_BIT);
		pr_emerg("L2 uncorrectable error\n");
		machine_restart(NULL);
	}
	if (val & 0x80000000) {
		int cpu = get_cpu();

		count0 = ((val) & 0x000000ff00000000) >> 32;
		count1 = ((val) & 0x0000ff0000000000) >> 40;

		/* increment correctable error counts */
		trace_edac_l2cache_counter(count0 + count1);
		if (count0 || count1)
			edac_device_handle_multi_ce(edac_dev, 0,
				cpu/CORES_PER_CLUSTER,
				count0 + count1, edac_dev->ctl_name);

		/* Clear the valid bit */
		clear_val = 0x80000000;
		write_l2merrsr(clear_val);
		put_cpu();
	}
}

/* Check for L2 Errors */
static void intel_l2_error_check(struct edac_device_ctl_info *edac_dev)
{
	/* 4 cores per cluster */
	int nr_cluster_ids = ((nr_cpu_ids - 1) / CORES_PER_CLUSTER) + 1;
	int i, j, cpu;

	/* execute on current cpu */
	log_l2merrsr(edac_dev);

	for (i = 0; i < nr_cluster_ids; i++) {
		/* No need to run on local cluster. */
		if (i == (get_cpu() / CORES_PER_CLUSTER)) {
			put_cpu();
			continue;
		}
		/*
		 * Have some core in each cluster execute this,
		 * Start with the first core on that cluster.
		 */
		cpu = i * CORES_PER_CLUSTER;
		for (j = cpu; j < cpu + CORES_PER_CLUSTER; j++) {
			if (cpu_online(j)) {
				smp_call_function_single(j, log_l2merrsr,
					edac_dev, 1);
				break;
			}
		}
		put_cpu();
	}
}

static void intel_add_edac_devices(struct platform_device *pdev,
	int num)
{
	struct intel_edac_dev_info *dev_info = NULL;
	/* 4 cores per cluster */
	int nr_cluster_ids = ((nr_cpu_ids - 1) / CORES_PER_CLUSTER) + 1;
	struct device_node *np = pdev->dev.of_node;

	dev_info = devm_kzalloc(&pdev->dev, sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info)
		return;

	dev_info->ctl_name = kstrdup(np->name, GFP_KERNEL);
	if (num == 0) {
		dev_info->blk_name = "cpumerrsr";
		dev_info->check = intel_cpu_error_check;
	} else {
		dev_info->blk_name = "l2merrsr";
		dev_info->check = intel_l2_error_check;
	}
	dev_info->pdev = pdev;
	dev_info->edac_idx = edac_device_alloc_index();
	dev_info->syscon =
			syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(dev_info->syscon)) {
		pr_info("%s: syscon lookup failed\n", np->name);
		goto err1;
	}

	if (num == 0) {
		/* cpu L1 */
		dev_info->edac_dev =
		edac_device_alloc_ctl_info(0, dev_info->ctl_name,
			1, dev_info->blk_name, num_possible_cpus(), 0, NULL,
			0, dev_info->edac_idx);
	} else {
		/* cluster L2 */
		dev_info->edac_dev =
		edac_device_alloc_ctl_info(0, dev_info->ctl_name,
			1, dev_info->blk_name, nr_cluster_ids, 0, NULL,
			0, dev_info->edac_idx);
	}
	if (!dev_info->edac_dev) {
		pr_info("No memory for edac device\n");
		goto err1;
	}

	dev_info->edac_dev->log_ce = 0;
	dev_info->edac_dev->pvt_info = dev_info;
	dev_info->edac_dev->dev = &dev_info->pdev->dev;
	dev_info->edac_dev->ctl_name = dev_info->ctl_name;
	dev_info->edac_dev->mod_name = INTEL_EDAC_MOD_STR;
	dev_info->edac_dev->dev_name = dev_name(&dev_info->pdev->dev);

	dev_info->edac_dev->edac_check = dev_info->check;

	if (edac_device_add_device(dev_info->edac_dev) != 0) {
		pr_info("Unable to add edac device for %s\n",
				dev_info->ctl_name);
		goto err2;
	}

	return;
err2:
	edac_device_free_ctl_info(dev_info->edac_dev);
err1:
	platform_device_unregister(dev_info->pdev);
}

static int intel_edac_cpu_probe(struct platform_device *pdev)
{
	edac_op_state = EDAC_OPSTATE_POLL;
	intel_add_edac_devices(pdev, 0);
	return 0;
}

static int intel_edac_cpu_remove(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
	return 0;
}

static int intel_edac_l2_probe(struct platform_device *pdev)
{
	edac_op_state = EDAC_OPSTATE_POLL;
	intel_add_edac_devices(pdev, 1);
	return 0;
}

static int intel_edac_l2_remove(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
	return 0;
}

static const struct of_device_id intel_edac_l2_match[] = {
#if defined(CONFIG_EDAC_AXXIA_L2_CPU_5600)

	{
	.compatible = "intel,cortex-a57-l2-cache",
	},

#endif

#if defined(CONFIG_EDAC_AXXIA_L2_CPU_6700)

	{
	.compatible = "intel,cortex-a53-l2-cache",
	},

#endif

	{},
};

static struct platform_driver intel_edac_l2_driver = {
	.probe = intel_edac_l2_probe,
	.remove = intel_edac_l2_remove,
	.driver = {
		.name = "intel_edac_l2",
		.of_match_table = intel_edac_l2_match,
	}
};
static const struct of_device_id intel_edac_cpu_match[] = {
#if defined(CONFIG_EDAC_AXXIA_L2_CPU_5600)

	{
	.compatible = "intel,cortex-a57-cpu",
	},

#endif

#if defined(CONFIG_EDAC_AXXIA_L2_CPU_6700)

	{
	.compatible = "intel,cortex-a53-cpu",
	},

#endif
	{},
};

static struct platform_driver intel_edac_cpu_driver = {
	.probe = intel_edac_cpu_probe,
	.remove = intel_edac_cpu_remove,
	.driver = {
		.name = "intel_edac_cpu",
		.of_match_table = intel_edac_cpu_match,
	}
};

module_platform_driver(intel_edac_cpu_driver);
module_platform_driver(intel_edac_l2_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Majtyka <marekx.majtyka@intel.com>");
