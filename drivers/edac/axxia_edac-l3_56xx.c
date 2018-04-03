/*
 * drivers/edac/axxia_edac-l3_56xx.c
 *
 * EDAC Driver for Intel's Axxia 5600 L3 (DICKENS)
 * EDAC Driver for Intel's Axxia 6700 L3 (SHELLEY)
 *
 * Copyright (C) 2017 Intel Inc.
 *
 * This file may be distributed under the terms of the
 * GNU General Public License.
 */
#define CREATE_TRACE_POINTS

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/axxia-ncr.h>
#include <linux/edac.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/arm-smccc.h>
#include <trace/events/edacl3.h>
#include "edac_module.h"
#include "axxia_edac.h"

#if defined(CONFIG_EDAC_AXXIA_L3_5600)
#define INTEL_EDAC_MOD_STR     "axxia56xx_edac"
#define CCN_XP_NODES			11
#define	CCN_HNI_NODES			1
#endif

#if defined(CONFIG_EDAC_AXXIA_L3_6700)
#define INTEL_EDAC_MOD_STR     "axxia67xx_edac"
#define CCN_XP_NODES			18
#define	CCN_HNI_NODES			2
#endif

#define SYSCON_PERSIST_SCRATCH 0xdc
#define L3_PERSIST_SCRATCH_BIT (0x1 << 4)

#define CCN_DT_PMOVSR			0x0198
#define CCN_DT_PMOVSR_CLR		0x01a0

#define CCN_MN_ERRINT_STATUS		0x0008
#define CCN_MN_ERRINT_STATUS__INTREQ__DESSERT		0x11
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__ENABLE	0x02
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__DISABLED	0x20
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__DISABLE	0x22
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_ENABLE	0x04
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_DISABLED	0x40
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_DISABLE	0x44
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__ENABLE	0x08
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLED	0x80
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLE	0x88

#define CCN_MN_ERR_SIG_VAL_63_0		0x0300
#define CCN_MN_ERR_SIG_VAL_63_0__DT			(1 << 1)
#define CCN_MN_ERROR_TYPE_VALUE		0x0320

#define CCN_REGION_SIZE	0x10000

#define CCN_HNI_NODE_BIT		8
#define CCN_HNF_NODES			8
#define CCN_DT_NODE_BASE_ADDR		(1 * CCN_REGION_SIZE)
#define CCN_HNI_NODE_BASE_ADDR(i)	(0x80000 + (i) * CCN_REGION_SIZE)
#define CCN_HNF_NODE_BASE_ADDR(i)	(0x200000 + (i) * CCN_REGION_SIZE)
#define CCN_XP_NODE_BASE_ADDR(i)	(0x400000 + (i) * CCN_REGION_SIZE)

#define CCN_NODE_ERR_SYND_REG0		0x400
#define CCN_NODE_ERR_SYND_REG1		0x408
#define CCN_NODE_ERR_SYND_CLR		0x480

union dickens_hnf_err_syndrome_reg0 {
	struct __packed {
	#ifdef CPU_BIG_ENDIAN
		unsigned long long err_extnd			: 1;
		unsigned long long first_err_vld		: 1;
		unsigned long long err_class			: 2;
		unsigned long long mult_err			: 1;
		unsigned long long err_count			: 16;
		unsigned long long reserved_err_syndrome_reg0_20: 23;
		unsigned long long err_count_set		: 12;
		unsigned long long err_count_ovrflw		: 1;
		unsigned long long err_count_match		: 1;
		unsigned long long err_count_type		: 2;
		unsigned long long par_err_id			: 1;
		unsigned long long err_id			: 3;
	#else
		unsigned long long err_id			: 3;
		unsigned long long par_err_id			: 1;
		unsigned long long err_count_type		: 2;
		unsigned long long err_count_match		: 1;
		unsigned long long err_count_ovrflw		: 1;
		unsigned long long err_count_set		: 12;
		unsigned long long reserved_err_syndrome_reg0_20: 23;
		unsigned long long err_count			: 16;
		unsigned long long mult_err			: 1;
		unsigned long long err_class			: 2;
		unsigned long long first_err_vld		: 1;
		unsigned long long err_extnd			: 1;
	#endif
	} reg0;
	u64 value;
};

union dickens_hnf_err_syndrome_reg1 {
	struct __packed {
	#ifdef CPU_BIG_ENDIAN
		unsigned long long reserved_err_syndrome_reg1_55: 9;
		unsigned long long err_srcid			: 7;
		unsigned long long reserved_err_syndrome_reg1_46: 2;
		unsigned long long err_optype			: 2;
		unsigned long long err_addr			: 44;
	#else
		unsigned long long err_addr			: 44;
		unsigned long long err_optype			: 2;
		unsigned long long reserved_err_syndrome_reg1_46: 2;
		unsigned long long err_srcid			: 7;
		unsigned long long reserved_err_syndrome_reg1_55: 9;
	#endif
	} reg1;
	u64 value;
};

union dickens_hnf_err_syndrome_clr {
	struct __packed {
	#ifdef CPU_BIG_ENDIAN
		unsigned long long reserved_err_syndrome_clr_63	: 1;
		unsigned long long first_err_vld_clr		: 1;
		unsigned long long reserved_err_syndrome_clr_60	: 2;
		unsigned long long mult_err_clr			: 1;
		unsigned long long reserved_err_syndrome_clr_0	: 59;
	#else
		unsigned long long reserved_err_syndrome_clr_0	: 59;
		unsigned long long mult_err_clr			: 1;
		unsigned long long reserved_err_syndrome_clr_60	: 2;
		unsigned long long first_err_vld_clr		: 1;
		unsigned long long reserved_err_syndrome_clr_63	: 1;
	#endif
	} clr;
	u64 value;
};

struct event_data {
	u64 err_synd_reg0;
	u64 err_synd_reg1;
	int idx;
};

/* Private structure for common edac device */
struct intel_edac_dev_info {
	struct platform_device *pdev;
	char *ctl_name;
	char *blk_name;
	int edac_idx;
	int irq_used;
	struct event_data data[CCN_HNF_NODES];
	struct event_data data_xp[CCN_XP_NODES];
	struct event_data data_hni[CCN_HNI_NODES];
	struct regmap *syscon;
	void __iomem *dickens_L3;
	struct edac_device_ctl_info *edac_dev;
	void (*check)(struct edac_device_ctl_info *edac_dev);
};

static void clear_node_error(void __iomem *addr)
{
	union dickens_hnf_err_syndrome_clr err_syndrome_clr;

	err_syndrome_clr.value = 0x0;
	err_syndrome_clr.clr.first_err_vld_clr = 0x1;
	err_syndrome_clr.clr.mult_err_clr = 0x1;
	writeq(err_syndrome_clr.value, addr);
}

static irqreturn_t
ccn_pmu_overflow_handler(struct intel_edac_dev_info *dev_info)
{
	u64 pmovsr = 0;

	pmovsr = readq(dev_info->dickens_L3 +
			CCN_DT_NODE_BASE_ADDR + CCN_DT_PMOVSR);
	if (!pmovsr)
		return IRQ_NONE;

	/*
	 * TODO
	 * Add perf implementation for handling this
	 * skipped for now - low priority
	 */

	writeq(pmovsr, dev_info->dickens_L3 +
		CCN_DT_NODE_BASE_ADDR  + CCN_DT_PMOVSR_CLR);

	return IRQ_HANDLED;
}

static irqreturn_t ccn_irq_thread(int irq, void *device)
{
	struct intel_edac_dev_info *dev_info = device;
	struct edac_device_ctl_info *edac_dev = dev_info->edac_dev;
	union dickens_hnf_err_syndrome_reg0 err_syndrome_reg0;
	union dickens_hnf_err_syndrome_reg1 err_syndrome_reg1;
	struct arm_smccc_res r;
	unsigned int count = 0;
	int i;

	/* only HNF nodes are of our interest */
	for (i = 0; i < CCN_HNF_NODES; ++i) {
		err_syndrome_reg0.value = dev_info->data[i].err_synd_reg0;
		err_syndrome_reg1.value = dev_info->data[i].err_synd_reg1;

		dev_info->data[i].err_synd_reg0 = 0;
		dev_info->data[i].err_synd_reg1 = 0;

		if (err_syndrome_reg0.reg0.first_err_vld) {
			if (err_syndrome_reg0.reg0.err_class & 0x3) {
				regmap_update_bits(dev_info->syscon,
						   SYSCON_PERSIST_SCRATCH,
						   L3_PERSIST_SCRATCH_BIT,
						   L3_PERSIST_SCRATCH_BIT);
				/* Fatal error */
				pr_emerg("L3 uncorrectable error\n");
				arm_smccc_smc(0xc4000027,
					CCN_MN_ERRINT_STATUS__INTREQ__DESSERT,
					0, 0, 0, 0, 0, 0, &r);
				machine_restart(NULL);
			}
			count = err_syndrome_reg0.reg0.err_count;
			if (count)
				edac_device_handle_multi_ce(edac_dev, 0,
					dev_info->data[i].idx,
					count, edac_dev->ctl_name);
		}
	}

	/* Interrupt deasserted */
	arm_smccc_smc(0xc4000027, CCN_MN_ERRINT_STATUS__INTREQ__DESSERT,
			0, 0, 0, 0, 0, 0, &r);

	trace_edacl3_smc_results(&r);

	return IRQ_HANDLED;
}

static irqreturn_t ccn_irq_handler(int irq, void *device)
{
	struct intel_edac_dev_info *dev_info = device;
	void __iomem *ccn_base = dev_info->dickens_L3;

	irqreturn_t res = IRQ_NONE;
	u64 err_sig_val[3];
	u64 err_type_value[4];
	u64 err_or;
	u64 err_synd_reg0 = 0, err_synd_reg1 = 0;
	int i;

	/* PMU overflow is a special case - for the future */
	err_or = err_sig_val[0] = readq(ccn_base + CCN_MN_ERR_SIG_VAL_63_0);
	if (err_or & CCN_MN_ERR_SIG_VAL_63_0__DT) {
		err_or &= ~CCN_MN_ERR_SIG_VAL_63_0__DT;
		res = ccn_pmu_overflow_handler(dev_info);
	}

	/* Have to read all err_sig_vals to clear them */
	for (i = 1; i < ARRAY_SIZE(err_sig_val); i++) {
		err_sig_val[i] = readq(ccn_base +
				CCN_MN_ERR_SIG_VAL_63_0 + i * 8);
		err_or |= err_sig_val[i];
	}

	trace_edacl3_sig_vals(err_sig_val[0], err_sig_val[1],
				err_sig_val[2]);

	i = 0;
	err_type_value[i]   = readq(ccn_base + CCN_MN_ERROR_TYPE_VALUE);
	err_type_value[++i] = readq(ccn_base + CCN_MN_ERROR_TYPE_VALUE + 0x8);
	err_type_value[++i] = readq(ccn_base + CCN_MN_ERROR_TYPE_VALUE + 0x10);
	err_type_value[++i] = readq(ccn_base + CCN_MN_ERROR_TYPE_VALUE + 0x20);

	trace_edacl3_error_types(err_type_value[0], err_type_value[1],
					err_type_value[2], err_type_value[3]);

	/* check hni node */
	for (i = 0; i < CCN_HNI_NODES; ++i) {
		if ((0x1 << (CCN_HNI_NODE_BIT + i)) & err_sig_val[0]) {
			err_synd_reg0 = readq(ccn_base +
					CCN_HNI_NODE_BASE_ADDR(i) +
					CCN_NODE_ERR_SYND_REG0);
			err_synd_reg1 = readq(ccn_base +
					CCN_HNI_NODE_BASE_ADDR(i) +
					CCN_NODE_ERR_SYND_REG1);

			trace_edacl3_syndromes(err_synd_reg0,
							err_synd_reg1);
			dev_info->data_hni[i].err_synd_reg0 = err_synd_reg0;
			dev_info->data_hni[i].err_synd_reg1 = err_synd_reg1;
			dev_info->data_hni[i].idx = 0;

			clear_node_error(ccn_base + CCN_HNI_NODE_BASE_ADDR(i) +
						CCN_NODE_ERR_SYND_CLR);
		}
	}

	/* go through all hnf nodes */
	for (i = 0; i < CCN_HNF_NODES; ++i) {
		/* when enabled process */
		if ((0x1 << (32 + i)) & err_sig_val[1]) {
			err_synd_reg0 = readq(ccn_base +
						CCN_HNF_NODE_BASE_ADDR(i) +
						CCN_NODE_ERR_SYND_REG0);
			err_synd_reg1 = readq(ccn_base +
						CCN_HNF_NODE_BASE_ADDR(i) +
						CCN_NODE_ERR_SYND_REG1);

			trace_edacl3_syndromes(err_synd_reg0,
							err_synd_reg1);

			dev_info->data[i].err_synd_reg0 = err_synd_reg0;
			dev_info->data[i].err_synd_reg1 = err_synd_reg1;
			dev_info->data[i].idx = i;

			clear_node_error(ccn_base + CCN_HNF_NODE_BASE_ADDR(i) +
						CCN_NODE_ERR_SYND_CLR);
		}
	}

	/* process XP errors only and only if bit[0] in enabled */
	if (0x1 & err_sig_val[2]) {
		/* look into all XP nodes */
		for (i = 0; i < CCN_XP_NODES; ++i) {
			err_synd_reg0 = readq(ccn_base +
						CCN_XP_NODE_BASE_ADDR(i) +
						CCN_NODE_ERR_SYND_REG0);

			dev_info->data_xp[i].err_synd_reg0 = err_synd_reg0;
			dev_info->data_xp[i].err_synd_reg1 = 0;

			trace_edacl3_syndromes(err_synd_reg0,
						err_synd_reg1);

			clear_node_error(ccn_base + CCN_XP_NODE_BASE_ADDR(i) +
				CCN_NODE_ERR_SYND_CLR);
		}
	}

	if (err_or)
		dev_err(&dev_info->pdev->dev,
			"Error reported in %016llx %016llx %016llx.\n",
			err_sig_val[2], err_sig_val[1], err_sig_val[0]);

	/* HERE all error data collected, but interrupt not deasserted */
	return IRQ_WAKE_THREAD;
}


/* Check for L3 Errors */
static void intel_l3_error_check(struct edac_device_ctl_info *edac_dev)
{
	void __iomem *addr;
	union dickens_hnf_err_syndrome_reg0 err_syndrome_reg0;
	union dickens_hnf_err_syndrome_clr err_syndrome_clr;
	unsigned int count = 0;
	int instance;
	struct intel_edac_dev_info *dev_info;

	err_syndrome_clr.value = 0x0;
	err_syndrome_clr.clr.first_err_vld_clr = 0x1;
	err_syndrome_clr.clr.mult_err_clr = 0x1;

	dev_info = (struct intel_edac_dev_info *) edac_dev->pvt_info;
	addr = dev_info->dickens_L3 + CCN_HNF_NODE_BASE_ADDR(0);

	for (instance = 0;
		instance < CCN_HNF_NODES;
		instance++, addr += CCN_REGION_SIZE) {

		err_syndrome_reg0.value =
			readq(addr + CCN_NODE_ERR_SYND_REG0);

		trace_edacl3_syndromes(err_syndrome_reg0.value,
						(u64) 0);
		/* First error valid */
		if (err_syndrome_reg0.reg0.first_err_vld) {
			if (err_syndrome_reg0.reg0.err_class & 0x3) {
				regmap_update_bits(dev_info->syscon,
						   SYSCON_PERSIST_SCRATCH,
						   L3_PERSIST_SCRATCH_BIT,
						   L3_PERSIST_SCRATCH_BIT);
				/* Fatal error */
				pr_emerg("L3 uncorrectable error\n");
				machine_restart(NULL);
			}
			count = err_syndrome_reg0.reg0.err_count;
			if (count)
				edac_device_handle_multi_ce(edac_dev, 0,
					instance, count, edac_dev->ctl_name);

			/* clear the valid bit */
			clear_node_error(addr + CCN_NODE_ERR_SYND_CLR);
		}
	}
}

static int intel_edac_l3_probe(struct platform_device *pdev)
{
	struct intel_edac_dev_info *dev_info = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct resource *r;

	struct arm_smccc_res ret;

	dev_info = devm_kzalloc(&pdev->dev, sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info)
		return -ENOMEM;

	dev_info->ctl_name = kstrdup(np->name, GFP_KERNEL);
	dev_info->blk_name = "l3merrsr";
	dev_info->pdev = pdev;
	dev_info->edac_idx = edac_device_alloc_index();

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		pr_err("Unable to get mem resource\n");
		goto err1;
	}

	dev_info->dickens_L3 = devm_ioremap(&pdev->dev, r->start,
					    resource_size(r));
	if (!dev_info->dickens_L3) {
		pr_err("INTEL_L3 devm_ioremap error\n");
		goto err1;
	}

	dev_info->syscon =
		syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(dev_info->syscon)) {
		pr_info("%s: syscon lookup failed\n",
			np->name);
		goto err1;
	}
	dev_info->edac_dev =
		edac_device_alloc_ctl_info(0, dev_info->ctl_name,
					   1, dev_info->blk_name,
					   CCN_HNI_NODES, 0, NULL, 0,
					   dev_info->edac_idx);
	if (!dev_info->edac_dev) {
		pr_info("No memory for edac device\n");
		goto err1;
	}

	dev_info->edac_dev->log_ce = 0;

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r)
		return -EINVAL;

	/*
	 * Check if we can use the interrupt here.
	 * We are not interested in PMU events, so let's try to disable it.
	 * Once -1 return, it means old uboot without ccn service.
	 * Then only polling mechanism is allowed, as it was before.
	 */
	arm_smccc_smc(0xc4000027, CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLE,
			0, 0, 0, 0, 0, 0, &ret);

	if (ret.a0 != ARM_SMCCC_UNKNOWN)
		dev_info->irq_used = 1;

	dev_info->edac_dev->pvt_info = dev_info;
	dev_info->edac_dev->dev = &dev_info->pdev->dev;
	dev_info->edac_dev->ctl_name = dev_info->ctl_name;
	dev_info->edac_dev->mod_name = INTEL_EDAC_MOD_STR;
	dev_info->edac_dev->dev_name = dev_name(&dev_info->pdev->dev);

	if (dev_info->irq_used) {
		edac_op_state = EDAC_OPSTATE_INT;
		dev_info->edac_dev->edac_check = NULL;
	} else {
		edac_op_state = EDAC_OPSTATE_POLL;
		dev_info->edac_dev->edac_check = intel_l3_error_check;
	}

	if (edac_device_add_device(dev_info->edac_dev) != 0) {
		pr_info("Unable to add edac device for %s\n",
				dev_info->ctl_name);
		goto err2;
	}

	if (dev_info->irq_used) {
		if (devm_request_threaded_irq(&dev_info->pdev->dev, r->start,
			ccn_irq_handler, ccn_irq_thread, IRQF_ONESHOT,
			dev_name(&dev_info->pdev->dev), dev_info))
			goto err2;
	}

	return 0;
err2:
	edac_device_free_ctl_info(dev_info->edac_dev);
err1:
	platform_device_unregister(dev_info->pdev);
	return 1;
}

static int intel_edac_l3_remove(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
	return 0;
}

static const struct of_device_id intel_edac_l3_match[] = {
#if defined(CONFIG_EDAC_AXXIA_L2_CPU_5600)

	{
	.compatible = "intel,ccn504-l3-cache",
	},

#endif

#if defined(CONFIG_EDAC_AXXIA_L2_CPU_6700)

	{
	.compatible = "intel,ccn512-l3-cache",
	},

#endif
	{},
};

static struct platform_driver intel_edac_l3_driver = {
	.probe = intel_edac_l3_probe,
	.remove = intel_edac_l3_remove,
	.driver = {
		.name = "intel_edac_l3",
		.of_match_table = intel_edac_l3_match,
	}
};

module_platform_driver(intel_edac_l3_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Majtyka <marekx.majtyka@intel.com>");
