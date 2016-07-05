/**
 * dwc3-axxia.c - Axxia Specific Glue layer
 *
 * Copyright (C) 2015 Intel Corporation - http://www.intel.com
 *
 * Author: Sangeetha Rao <sangeetha.rao@intel.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of_platform.h>

static u64 adwc3_dma_mask;

struct dwc3_axxia {
	struct device			*dev;
};

static int axxia_dwc3_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*node = pdev->dev.of_node;
	struct dwc3_axxia	*adwc;
	int			error;

	adwc = devm_kzalloc(dev, sizeof(*adwc), GFP_KERNEL);
	if (!adwc)
		return -ENOMEM;

	platform_set_drvdata(pdev, adwc);

	adwc->dev = dev;

	adwc3_dma_mask = dma_get_mask(dev);
	dev->dma_mask = &adwc3_dma_mask;


	error = of_platform_populate(node, NULL, NULL, dev);
	if (error) {
		dev_err(&pdev->dev, "failed to create dwc3 core\n");
		goto err_core;
	}

	return 0;

err_core:

	return error;
}

static int axxia_dwc3_remove_core(struct device *dev, void *c)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static int axxia_dwc3_remove(struct platform_device *pdev)
{

	device_for_each_child(&pdev->dev, NULL, axxia_dwc3_remove_core);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

void
arch_setup_pdev_archdata(struct platform_device *pdev)
{
	if (strncmp(pdev->name, "xhci-hcd", strlen("xhci-hcd")) == 0)
		pdev->dev.archdata.dma_coherent = 1;
}

static const struct of_device_id adwc3_of_match[] = {
	{ .compatible = "intel,axxia-dwc3", },
	{},
};
MODULE_DEVICE_TABLE(of, adwc3_of_match);

static struct platform_driver adwc3_driver = {
	.probe		= axxia_dwc3_probe,
	.remove		= axxia_dwc3_remove,
	.driver		= {
		.name	= "axxia-dwc3",
		.of_match_table	= adwc3_of_match,
	},
};

module_platform_driver(adwc3_driver);

MODULE_ALIAS("platform:axxia-dwc3");
MODULE_AUTHOR("Sangeetha Rao <sangeetha.rao@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 Intel's Axxia Glue Layer");
