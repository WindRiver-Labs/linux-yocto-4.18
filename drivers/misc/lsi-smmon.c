/*
 *  Copyright (C) 2013 LSI Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Error monitor for system memory.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/reboot.h>

#include <mach/ncr.h>

#define APB2_SER3_PHY_ADDR        0x002010030000ULL
#define APB2_SER3_PHY_SIZE   0x1000

static int log = 1;
module_param(log, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(log, "Log each error to kernel log.");

static int machineRestart = 1;
module_param(machineRestart, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(machineRestart, "Machine restart on fatal error.");
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
#define SM_INT_MASK (0x1f7f701)

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

struct smmon_attr {
	struct device_attribute attr;
	int                     event;
};

#define SMMON_ATTR(_name, _event) \
	{ \
		.attr = __ATTR(_name, S_IRUGO, smmon_show, NULL), \
		.event = _event \
	}

struct sm_dev {
	struct platform_device *pdev;
	u32 region; /* NCR region address */
	void __iomem *apb2ser3_region;
	u32 counter[NR_EVENTS];
};


static ssize_t
smmon_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sm_dev *sm = dev_get_drvdata(dev);
	struct smmon_attr *sma = container_of(attr, struct smmon_attr, attr);
	return sprintf(buf, "%u", sm->counter[sma->event]);
}

static struct smmon_attr smmon_attr_counter[] = {
	SMMON_ATTR(illegal_access, EV_ILLEGAL),
	SMMON_ATTR(illegal_access_mult, EV_MULT_ILLEGAL),
	SMMON_ATTR(correctable_ecc_error, EV_CORR_ECC),
	SMMON_ATTR(correctable_ecc_error_mult, EV_MULT_CORR_ECC),
	SMMON_ATTR(uncorrectable_ecc_error, EV_UNCORR_ECC),
	SMMON_ATTR(uncorrectable_ecc_error_mult, EV_MULT_UNCORR_ECC),
	SMMON_ATTR(port_error, EV_PORT_ERROR),
	SMMON_ATTR(wrap_error, EV_WRAP_ERROR),
	SMMON_ATTR(parity_error, EV_PARITY_ERROR),
};

static struct attribute *smmon_attr[] = {
	&smmon_attr_counter[EV_ILLEGAL].attr.attr,
	&smmon_attr_counter[EV_MULT_ILLEGAL].attr.attr,
	&smmon_attr_counter[EV_CORR_ECC].attr.attr,
	&smmon_attr_counter[EV_MULT_CORR_ECC].attr.attr,
	&smmon_attr_counter[EV_UNCORR_ECC].attr.attr,
	&smmon_attr_counter[EV_MULT_UNCORR_ECC].attr.attr,
	&smmon_attr_counter[EV_PORT_ERROR].attr.attr,
	&smmon_attr_counter[EV_WRAP_ERROR].attr.attr,
	&smmon_attr_counter[EV_PARITY_ERROR].attr.attr,
	NULL
};

static struct attribute_group smmon_attr_group = {
	.name  = "counters",
	.attrs = smmon_attr
};

static irqreturn_t
smmon_isr(int interrupt, void *device)
{
	struct sm_dev *sm = device;
	u32 status;
	unsigned long setVal;
	int i;

	if (ncr_read(sm->region, 0x410, 4, &status)) {
		pr_err("%s: Error reading interrupt status\n",
		       dev_name(&sm->pdev->dev));
		return IRQ_NONE;
	}

	for (i = 0; i < NR_EVENTS; ++i) {
		if ((status & event_mask[i]) != 0) {
			++sm->counter[i];
			if (machineRestart && event_logging[i].fatal) {
				setVal = readl(sm->apb2ser3_region + 0xdc);
				/* set bit 3 in pscratch reg */
				setVal = (setVal) | (0x1 << 3);
				writel(setVal, sm->apb2ser3_region + 0xdc);
				pr_info("CPU uncorrectable error\n");
				machine_restart(NULL);
			}
			if (log)
				printk_ratelimited("%s%s: %s\n",
						   event_logging[i].level,
						   dev_name(&sm->pdev->dev),
						   event_logging[i].name);
		}
	}

	/* Clear interrupt */
	ncr_write(sm->region, 0x548, 4, &status);

	return IRQ_HANDLED;
}

static int
smmon_probe(struct platform_device *pdev)
{
	struct sm_dev *sm;
	struct resource *io;
	int irq;
	u32 mask;
	int rc = 0;
	struct device_node *np = pdev->dev.of_node;


	sm = devm_kzalloc(&pdev->dev, sizeof(*sm), GFP_KERNEL);
	if (!sm) {
		rc = -ENOMEM;
		goto out;
	}
	sm->pdev = pdev;

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		rc = -EINVAL;
		goto out;
	}
	sm->region = io->start;

	sm->apb2ser3_region = of_iomap(np, 1);
	if (!sm->apb2ser3_region)
		sm->apb2ser3_region = ioremap(APB2_SER3_PHY_ADDR,
				APB2_SER3_PHY_SIZE);

	/* Disable all memory controller interrupts */
	mask = 0xffffffff;
	ncr_write(sm->region, 0x414, 4, &mask);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		rc = irq;
		goto out;
	}

	rc = devm_request_irq(&pdev->dev, irq, smmon_isr,
			      IRQF_ONESHOT, dev_name(&pdev->dev), sm);
	if (rc)
		goto out;

	rc = sysfs_create_group(&pdev->dev.kobj, &smmon_attr_group);
	if (rc)
		goto out;

	dev_set_drvdata(&pdev->dev, sm);
	pr_info("%s: Memory controller monitor\n", dev_name(&pdev->dev));

	/* Enable memory controller interrupts. We need to disable the
	 * interrupt while unmasking it, since otherwise there will be a
	 * locking conflict in ncr_write/ncr_read when the ISR tries to read
	 * interrupt status.
	 */
	disable_irq(irq);
	mask = SM_INT_MASK;
	ncr_write(sm->region, 0x414, 4, &mask);
	enable_irq(irq);
out:
	return rc;
}

static int
smmon_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &smmon_attr_group);
	return 0;
}

static const struct of_device_id smmon_id_table[] = {
	{ .compatible = "lsi,smmon" },
	{ }
};
MODULE_DEVICE_TABLE(platform, smmon_id_table);

static struct platform_driver smmon_driver = {
	.driver = {
		.name = "lsi-smmon",
		.of_match_table = smmon_id_table
	},
	.probe = smmon_probe,
	.remove = smmon_remove,
};

module_platform_driver(smmon_driver);
