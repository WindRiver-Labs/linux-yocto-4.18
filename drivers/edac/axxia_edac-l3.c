 /*
  * drivers/edac/axxia_edac-l3.c
  *
  * EDAC Driver for Avago's Axxia 5500 for L3 cache
  *
  * Copyright (C) 2018 INTEL Inc.
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
  *
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/axxia-ncr.h>
#include <linux/edac.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include "edac_module.h"
#include "axxia_edac.h"

#define AXXIA_EDAC_MOD_STR     "axxia_edac"

#define APB2_PERSIST_SCRATCH 0xdc
#define L3_PERSIST_SCRATCH_BIT (0x1 << 4)

/* Private structure for common edac device */
struct axxia_edac_dev_info {
	struct platform_device *pdev;
	char *ctl_name;
	char *blk_name;
	int edac_idx;
	struct regmap *syscon;
	void __iomem *dickens_L3;
	struct edac_device_ctl_info *edac_dev;
	void (*check)(struct edac_device_ctl_info *edac_dev);
};

/* Check for L3 Errors */
static void axxia_l3_error_check(struct edac_device_ctl_info *edac_dev)
{
	unsigned long regVal1, regVal2;
	unsigned count = 0;
	int i, instance;
	struct axxia_edac_dev_info *dev_info;

	dev_info = (struct axxia_edac_dev_info *) edac_dev->pvt_info;

	for (instance = 0; instance < 8; instance++) {
		regVal1 = readl(dev_info->dickens_L3 + (instance * 0x10000));
		regVal2 = readl(dev_info->dickens_L3 +
			(instance * 0x10000) + 4);
		/* First error valid */
		if (regVal2 & 0x40000000) {
			if (regVal2 & 0x30000000) {
				regmap_update_bits(dev_info->syscon,
						   APB2_PERSIST_SCRATCH,
						   L3_PERSIST_SCRATCH_BIT,
						   L3_PERSIST_SCRATCH_BIT);
				/* Fatal error */
				pr_emerg("L3 uncorrectable error\n");
				machine_restart(NULL);
			}
			count = (regVal2 & 0x07fff800) >> 11;
			for (i = 0; i < count; i++)
				edac_device_handle_ce(edac_dev, 0,
					instance, edac_dev->ctl_name);
			/* clear the valid bit */
			writel(0x48000000, dev_info->dickens_L3 +
				(instance * 0x10000) + 84);
		}
	}
}

static int axxia_edac_l3_probe(struct platform_device *pdev)
{
	struct axxia_edac_dev_info *dev_info = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct resource *r;

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
		pr_err("AXXIA_L3 devm_ioremap error\n");
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
					   8, 0, NULL, 0,
					   dev_info->edac_idx);
	if (!dev_info->edac_dev) {
		pr_info("No memory for edac device\n");
		goto err1;
	}

	dev_info->edac_dev->pvt_info = dev_info;
	dev_info->edac_dev->dev = &dev_info->pdev->dev;
	dev_info->edac_dev->ctl_name = dev_info->ctl_name;
	dev_info->edac_dev->mod_name = AXXIA_EDAC_MOD_STR;
	dev_info->edac_dev->dev_name = dev_name(&dev_info->pdev->dev);
	edac_op_state = EDAC_OPSTATE_POLL;
	dev_info->edac_dev->edac_check = axxia_l3_error_check;

	if (edac_device_add_device(dev_info->edac_dev) != 0) {
		pr_info("Unable to add edac device for %s\n",
				dev_info->ctl_name);
		goto err2;
	}

	return 0;
err2:
	edac_device_free_ctl_info(dev_info->edac_dev);
err1:
	platform_device_unregister(dev_info->pdev);
	return 1;
}

static int axxia_edac_l3_remove(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
	return 0;
}

static struct of_device_id axxia_edac_l3_match[] = {
	{
	.compatible = "axxia,ccn504-l3-cache",
	},
	{},
};

static struct platform_driver axxia_edac_l3_driver = {
	.probe = axxia_edac_l3_probe,
	.remove = axxia_edac_l3_remove,
	.driver = {
		.name = "axxia_edac_l3",
		.of_match_table = axxia_edac_l3_match,
	}
};

module_platform_driver(axxia_edac_l3_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sangeetha Rao <sangeetha.rao@avagotech.com>");
