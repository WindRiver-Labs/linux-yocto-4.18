/*
 * Reset driver for Axxia devices
 *
 * Copyright (C) 2014 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#define SC_PSCRATCH            0x00dc
#define SC_CRIT_WRITE_KEY      0x2000
#define SC_RESET_CONTROL       0x2008
#define   RSTCTL_RST_CHIP	(1<<1)
#define   RSTCTL_RST_SYS	(1<<0)

static struct regmap *syscon;

static int ddr_retention_enabled;

void
initiate_retention_reset(void)
{
	if (0 == ddr_retention_enabled) {
		pr_info("DDR Retention Reset is Not Enabled\n");
		return;
	}

	if (WARN_ON(!syscon))
		return;

	/* set retention reset bit in pscratch */
	regmap_write(syscon, SC_PSCRATCH, 1);

	/* trap into secure monitor to do the reset */
	machine_restart(NULL);
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
	* this feature is only meaningful on ASIC systems,
	* but for now we allow it on simulator
	*/

/*	if (of_find_compatible_node(NULL, NULL, "lsi,axm5500-amarillo")) { */
	if (1) {
		/* Create /proc entry. */
		if (!proc_create("driver/axxia_ddr_retention_reset",
				S_IWUSR, NULL, &axxia_ddr_retention_proc_ops)) {
			pr_info("Failed to register DDR retention proc entry\n");
		} else {
			ddr_retention_enabled = 1;
			pr_info("DDR Retention Reset Initialized\n");
		}
	} else {
		pr_info("DDR Retention Reset is Not Available\n");
	}
}


static int axxia_restart_handler(struct notifier_block *this,
				 unsigned long mode, void *cmd)
{
	/* Access Key (0xab) */
	regmap_write(syscon, SC_CRIT_WRITE_KEY, 0xab);

	/* Assert chip reset */
	regmap_update_bits(syscon, SC_RESET_CONTROL,
			   RSTCTL_RST_CHIP, RSTCTL_RST_CHIP);

	return NOTIFY_DONE;
}

static struct notifier_block axxia_restart_nb = {
	.notifier_call = axxia_restart_handler,
	.priority = 128,
};

static int axxia_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	syscon = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
	if (IS_ERR(syscon)) {
		pr_err("%s: syscon lookup failed\n", dev->of_node->name);
		return PTR_ERR(syscon);
	}

	err = register_restart_handler(&axxia_restart_nb);
	if (err)
		dev_err(dev, "cannot register restart handler (err=%d)\n", err);

	return err;
}

static const struct of_device_id of_axxia_reset_match[] = {
	{ .compatible = "intel,axm56xx-reset",},
	{},
};
MODULE_DEVICE_TABLE(of, of_axxia_reset_match);

static struct platform_driver axxia_reset_driver = {
	.probe = axxia_reset_probe,
	.driver = {
		.name = "axxia-reset",
		.of_match_table = of_axxia_reset_match,
	},
};

static int __init axxia_reset_init(void)
{
	axxia_ddr_retention_init();
	return platform_driver_register(&axxia_reset_driver);
}
device_initcall(axxia_reset_init);
