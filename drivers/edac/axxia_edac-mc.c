 /*
  * drivers/edac/axxia_edac-mc.c
  *
  * EDAC Driver for Avago's Axxia 5500 System Memory Controller
  *
  * Copyright (C) 2010 LSI Inc.
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
#include <linux/edac.h>
#include <mach/ncr.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include "edac_core.h"
#include "edac_module.h"

#define LSI_EDAC_MOD_STR     "lsi_edac"

#define APB2_SER3_PHY_ADDR        0x002010030000ULL
#define APB2_SER3_PHY_SIZE   0x1000

#define SM_INT_STATUS_REG 0x410
#define SM_INT_STATUS_CLEAR_REG 0x548
#define SM_INT_STATUS_MASK_REG 0x414

#define APB2_PERSIST_SCRATCH 0xdc
#define SMEM_PERSIST_SCRATCH_BIT (0x1 << 3)

static int log = 1;
module_param(log, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(log, "Log each error to kernel log.");

static int machine_restrt = 1;
module_param(machine_restrt, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(machine_restrt, "Machine restart on fatal error.");

/*
  AXM55xx memory controller interrupt status bits:

  Bit [24] = The software-initiated control word write has completed.
  Bit [23] = The user-initiated DLL resync has completed.
  Bit [22] = A state change has been detected on the dfi_init_complete signal
	     after initialization.
  Bit [21] = The assertion of the INHIBIT_DRAM_CMD parameter has successfully
	     inhibited the command queue.
  Bit [20] = The register interface-initiated mode register write has completed
	     and another mode register write may be issued.
  Bit [19] = A parity error has been detected on the address/control bus on a
	     registered DIMM.
  Bit [18] = The leveling operation has completed.
  Bit [17] = A leveling operation has been requested.
  Bit [16] = A DFI update error has occurred. Error information can be found in
	     the UPDATE_ERROR_STATUS parameter.
  Bit [15] = A write leveling error has occurred. Error information can be found
	     in the WRLVL_ERROR_STATUS parameter.
  Bit [14] = A read leveling gate training error has occurred. Error information
	     can be found in the RDLVL_ERROR_STATUS parameter.
  Bit [13] = A read leveling error has occurred. Error information can be found
	     in the RDLVL_ERROR_STATUS parameter.
  Bit [12] = The user has programmed an invalid setting associated with user
	     words per burst. Examples: Setting param_reduc when burst
	     length = 2. A 1:2 MC:PHY clock ratio with burst length = 2.
  Bit [11] = A wrap cycle crossing a DRAM page has been detected. This is
	     unsupported & may result in memory data corruption.
  Bit [10] = The BIST operation has been completed.
  Bit [09] = The low power operation has been completed.
  Bit [08] = The MC initialization has been completed.
  Bit [07] = An error occurred on the port command channel.
  Bit [06] = Multiple uncorrectable ECC events have been detected.
  Bit [05] = An uncorrectable ECC event has been detected.
  Bit [04] = Multiple correctable ECC events have been detected.
  Bit [03] = A correctable ECC event has been detected.
  Bit [02] = Multiple accesses outside the defined PHYSICAL memory space have
	     occurred.
  Bit [01] = A memory access outside the defined PHYSICAL memory space has
	     occurred.
  Bit [00] = The memory reset is valid on the DFI bus.

  Of these, 1, 2, 3, 4, 5, 6, 7, 11, and 19 are of interest.
*/
#define SM_INT_MASK (0x1f7f719)

enum events {
	EV_ILLEGAL = 0,
	EV_MULT_ILLEGAL,
	EV_CORR_ECC,
	EV_MULT_CORR_ECC,
	EV_UNCORR_ECC,
	EV_MULT_UNCORR_ECC,
	EV_PORT_ERROR,
	EV_WRAP_ERROR,
	EV_PARITY_ERROR,
	NR_EVENTS
};

static const u32 event_mask[NR_EVENTS] = {
	[EV_ILLEGAL]          = 0x00000002,
	[EV_MULT_ILLEGAL]     = 0x00000004,
	[EV_CORR_ECC]         = 0x00000008,
	[EV_MULT_CORR_ECC]    = 0x00000010,
	[EV_UNCORR_ECC]       = 0x00000020,
	[EV_MULT_UNCORR_ECC]  = 0x00000040,
	[EV_PORT_ERROR]       = 0x00000080,
	[EV_WRAP_ERROR]       = 0x00000800,
	[EV_PARITY_ERROR]     = 0x00080000,
};

static const struct event_logging {
	int         fatal;
	const char *level;
	const char *name;
} event_logging[NR_EVENTS] = {
	[EV_ILLEGAL]         = {0, KERN_ERR, "Illegal access"},
	[EV_MULT_ILLEGAL]    = {0, KERN_ERR, "Illegal access"},
	[EV_CORR_ECC]        = {0, KERN_NOTICE, "Correctable ECC error"},
	[EV_MULT_CORR_ECC]   = {0, KERN_NOTICE, "Correctable ECC error"},
	[EV_UNCORR_ECC]      = {1, KERN_CRIT, "Uncorrectable ECC error"},
	[EV_MULT_UNCORR_ECC] = {1, KERN_CRIT, "Uncorrectable ECC error"},
	[EV_PORT_ERROR]      = {0, KERN_CRIT, "Port error"},
	[EV_WRAP_ERROR]      = {0, KERN_CRIT, "Wrap error"},
	[EV_PARITY_ERROR]    = {0, KERN_CRIT, "Parity error"},
};

/* Private structure for common edac device */
struct lsi_edac_dev_info {
	struct platform_device *pdev;
	char *ctl_name;
	char *blk_name;
	int edac_idx;
	u32 sm_region;
	struct regmap *syscon;
	void __iomem *apb2ser3_region;
	struct edac_device_ctl_info *edac_dev;
	void (*check)(struct edac_device_ctl_info *edac_dev);
};

static irqreturn_t
smmon_isr(int interrupt, void *device)
{
	struct lsi_edac_dev_info *edac_dev = device;
	u32 status;
	unsigned long set_val;
	int i;

	if (ncr_read(edac_dev->sm_region, SM_INT_STATUS_REG,
		4, &status)) {
		pr_err("%s: Error reading interrupt status\n",
		       dev_name(&edac_dev->pdev->dev));
		return IRQ_NONE;
	}

	for (i = 0; i < NR_EVENTS; ++i) {
		if ((status & event_mask[i]) != 0) {
			if (machine_restrt && event_logging[i].fatal) {
				if (IS_ERR(edac_dev->syscon)) {
					set_val = readl(
						edac_dev->apb2ser3_region
						+ APB2_PERSIST_SCRATCH);
					/* set bit 3 in pscratch reg */
					set_val = set_val
						| SMEM_PERSIST_SCRATCH_BIT;
					writel(set_val,
						edac_dev->apb2ser3_region +
						APB2_PERSIST_SCRATCH);
				} else
					regmap_update_bits(edac_dev->syscon,
					APB2_PERSIST_SCRATCH,
					SMEM_PERSIST_SCRATCH_BIT,
					SMEM_PERSIST_SCRATCH_BIT);
				pr_emerg("%s uncorrectable error\n",
					edac_dev->ctl_name);
				machine_restart(NULL);
			}
		}
	}

	/* Clear interrupt */
	ncr_write(edac_dev->sm_region, SM_INT_STATUS_CLEAR_REG,
		4, &status);

	return IRQ_HANDLED;
}

static void lsi_sm_error_check(struct edac_device_ctl_info *edac_dev)
{
	unsigned long sm_reg_val, clear_val;
	struct lsi_edac_dev_info *dev_info;

	dev_info = (struct lsi_edac_dev_info *) edac_dev->pvt_info;

	/* SM0 is instance 0 */
	ncr_read(dev_info->sm_region, SM_INT_STATUS_REG, 4, &sm_reg_val);
	if (sm_reg_val & 0x18) {
		/* single bit and multiple bit correctable errors */
		edac_device_handle_ce(edac_dev, 0, 0, edac_dev->ctl_name);
		/* Clear bits */
		clear_val = 0x18;
		ncr_write(dev_info->sm_region, SM_INT_STATUS_CLEAR_REG,
			4, &clear_val);
	}
}


static int lsi_edac_mc_probe(struct platform_device *pdev)
{
	static int count;
	struct lsi_edac_dev_info *dev_info = NULL;
	/* 4 cores per cluster */
	struct resource *io;
	struct device_node *np = pdev->dev.of_node;
	int irq = -1, rc = 0;
	u32 mask;

	dev_info = devm_kzalloc(&pdev->dev, sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info)
		return -ENOMEM;

	dev_info->ctl_name = kstrdup(np->name, GFP_KERNEL);
	dev_info->blk_name = "ECC";
	edac_op_state = EDAC_OPSTATE_POLL;

	dev_info->pdev = pdev;
	dev_info->edac_idx = edac_device_alloc_index();

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		dev_err(&pdev->dev, "Unable to get mem resource\n");
		goto err1;
	}
	dev_info->sm_region = io->start;
	dev_info->syscon =
		syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(dev_info->syscon)) {
		pr_info("%s: syscon lookup failed hence using hardcoded register address\n",
			np->name);
		dev_info->apb2ser3_region = ioremap(APB2_SER3_PHY_ADDR,
			APB2_SER3_PHY_SIZE);
		if (!dev_info->apb2ser3_region) {
			pr_err("ioremap of apb2ser3 region failed\n");
			goto err1;
		}

	}
	/* Disable all memory controller interrupts */
	mask = 0xffffffff;
	ncr_write(dev_info->sm_region, SM_INT_STATUS_MASK_REG,
		4, &mask);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		goto err1;
	if (count == 0) {
		rc = devm_request_irq(&pdev->dev, irq,
			smmon_isr, IRQF_ONESHOT,
			"smmon0", dev_info);
	} else {
		rc = devm_request_irq(&pdev->dev, irq,
			smmon_isr, IRQF_ONESHOT,
			"smmon1", dev_info);
	}
	if (rc)
		goto err1;
	dev_info->edac_dev =
		edac_device_alloc_ctl_info(0, dev_info->ctl_name,
					   1, dev_info->blk_name, 1, 0,
					   NULL, 0, dev_info->edac_idx);
	if (!dev_info->edac_dev) {
		pr_info("No memory for edac device\n");
		goto err1;
	}
	/* Enable memory controller interrupts. We need to disable
	 * the interrupt while unmasking it, since otherwise there
	 * will be a locking conflict in ncr_write/ncr_read when the
	 * ISR tries to read interrupt status.
	 */
	disable_irq(irq);
	mask = SM_INT_MASK;
	ncr_write(dev_info->sm_region, SM_INT_STATUS_MASK_REG,
			4, &mask);

	dev_info->edac_dev->pvt_info = dev_info;
	dev_info->edac_dev->dev = &dev_info->pdev->dev;
	dev_info->edac_dev->ctl_name = dev_info->ctl_name;
	dev_info->edac_dev->mod_name = LSI_EDAC_MOD_STR;
	dev_info->edac_dev->dev_name = dev_name(&dev_info->pdev->dev);
	dev_info->edac_dev->edac_check = lsi_sm_error_check;


	if (edac_device_add_device(dev_info->edac_dev) != 0) {
		pr_info("Unable to add edac device for %s\n",
				dev_info->ctl_name);
		goto err2;
	}
	enable_irq(irq);
	return 0;
err2:
	edac_device_free_ctl_info(dev_info->edac_dev);
err1:
	platform_device_unregister(dev_info->pdev);
	return 1;
}

static int lsi_edac_mc_remove(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
	return 0;
}

static const struct of_device_id lsi_edac_smmon_match[] = {
	{ .compatible = "lsi,smmon" },
	{ }
};
MODULE_DEVICE_TABLE(platform, lsi_edac_smmon_match);

static struct platform_driver lsi_edac_mc_driver = {
	.probe = lsi_edac_mc_probe,
	.remove = lsi_edac_mc_remove,
	.driver = {
		.name = "lsi_edac_smmon",
		.of_match_table = lsi_edac_smmon_match,
	}
};
module_platform_driver(lsi_edac_mc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sangeetha Rao <sangeetha.rao@avagotech.com>");
