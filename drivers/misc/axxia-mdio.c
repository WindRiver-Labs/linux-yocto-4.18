/*
 * Copyright (C) 2017 Intel <john.jacques@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/io.h>

/* MDIO Registers */
#define MDIO_CONTROL		0x00
#define   CONTROL_BUSY		(1<<31) /* MDIO cycle in progress (RO) */
#define   CONTROL_NOPRE		(1<<30) /* Suppress preamble */
#define   CONTROL_CL22		(0<<29) /* Clause-22 */
#define   CONTROL_CL45		(1<<29) /* Clause-45 */
#define   CONTROL_READ		(2<<27) /* Read operation */
#define   CONTROL_WRITE		(1<<27) /* Write operation */
#define   CONTROL_IFSEL		(1<<26) /* Interface select */
#define   CONTROL_PHYREG(_n)	(((_n) & 0x1F) << 21)
#define   CONTROL_PHYID(_n)	(((_n) & 0x1F) << 16)
#define   CONTROL_DATA(_n)	(((_n) & 0xFFFF) << 0)
#define MDIO_STATUS		0x04
#define   STATUS_BUSY		(1<<31)
#define   STATUS_DONE		(1<<30)
#define MDIO_CLK_OFFSET		0x08
#define MDIO_CLK_PERIOD		0x0c

/* MDIO bus driver private data */
struct axxia_mdio_priv {
	void __iomem *base;
	struct mii_bus *bus;
};

static inline void __iomem *
bus_to_regs(struct mii_bus *bus)
{
	return ((struct axxia_mdio_priv *)bus->priv)->base;
}

static int
axxia_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	void __iomem *base = bus_to_regs(bus);
	u32 ctrl;
	u32 data;

	/* Set the mdio_done (status) bit. */
	writel(STATUS_DONE | readl(base + MDIO_STATUS), base + MDIO_STATUS);

	/* Write the command. */
	ctrl = (CONTROL_READ |
		CONTROL_PHYID(mii_id) |
		CONTROL_PHYREG(regnum));

	if (regnum & MII_ADDR_C45)
		ctrl |= CONTROL_CL45;

	writel(ctrl, base + MDIO_CONTROL);

	/* Wait for the mdio_done (status) bit to clear. */
	while ((readl(base + MDIO_STATUS) & STATUS_DONE) != 0)
		cpu_relax();

	/* Wait for the mdio_busy (control) bit to clear. */
	do {
		data = readl(base + MDIO_CONTROL);
	} while ((data & CONTROL_BUSY) != 0);

	return data & 0xFFFF;
}

static int
axxia_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 value)
{
	void __iomem *base = bus_to_regs(bus);
	u32 ctrl;

	/* Wait for mdio_busy (control) to be clear. */
	while ((readl(base + MDIO_CONTROL) & CONTROL_BUSY) != 0)
		cpu_relax();

	/* Set the mdio_busy (status) bit. */
	writel(STATUS_DONE | readl(base + MDIO_STATUS), base + MDIO_STATUS);

	/* Write the command. */
	ctrl = (CONTROL_WRITE |
		CONTROL_PHYID(mii_id) |
		CONTROL_PHYREG(regnum) |
		CONTROL_DATA(value));

	if (regnum & MII_ADDR_C45)
		ctrl |= CONTROL_CL45;

	writel(ctrl, base + MDIO_CONTROL);

	/* Wait for the mdio_done (status) bit to clear. */
	while ((readl(base + MDIO_STATUS) & STATUS_DONE) != 0)
		cpu_relax();

	/* Wait for the mdio_busy (control) bit to clear. */
	while ((readl(base + MDIO_CONTROL) & CONTROL_BUSY) != 0)
		cpu_relax();

	return 0;
}

static int
axxia_mdio_probe(struct platform_device *pdev)
{
	struct device_node     *np = pdev->dev.of_node;
	struct axxia_mdio_priv *priv = NULL;
	struct resource        *res;
	int                     err;
	u32                     clk_offset = 0x10;
	u32                     clk_period = 0x2c;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);

	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res)
		return -ENODEV;

	priv->bus = mdiobus_alloc();

	if (!priv->bus)
		return -ENOMEM;

	priv->bus->name  = "Axxia MDIO",
	priv->bus->read  = axxia_mdio_read,
	priv->bus->write = axxia_mdio_write,
	priv->bus->priv  = priv;
	snprintf(priv->bus->id, MII_BUS_ID_SIZE, pdev->name);

	priv->base = devm_ioremap_resource(&pdev->dev, res);

	if (!priv->base) {
		dev_err(&pdev->dev, "Failed to map registers\n");
		err = -ENODEV;
		goto err_ret;
	}

	priv->bus->parent = &pdev->dev;
	dev_set_drvdata(&pdev->dev, priv->bus);

	of_property_read_u32(np, "lsi,mdio-clk-offset", &clk_offset);
	of_property_read_u32(np, "lsi,mdio-clk-period", &clk_period);

	writel(clk_offset, priv->base + MDIO_CLK_OFFSET);
	writel(clk_period, priv->base + MDIO_CLK_PERIOD);
	writel(0, priv->base + MDIO_CONTROL);

	err = of_mdiobus_register(priv->bus, np);

	if (err) {
		dev_err(&pdev->dev, "Failed to register MDIO bus\n");
		goto err_ret;
	}

	return 0;

err_ret:

	if (priv && priv->bus)
		kfree(priv->bus);

	return err;
}

static int
axxia_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = dev_get_drvdata(&pdev->dev);

	mdiobus_unregister(bus);
	dev_set_drvdata(&pdev->dev, NULL);
	mdiobus_free(bus);

	return 0;
}

static struct of_device_id axxia_mdio_match[] = {
	{ .compatible = "lsi,axm-mdio", },
	{ .compatible = "intel,axxia-mdio0", },
	{ .compatible = "intel,axxia-mdio1", },
	{},
};

MODULE_DEVICE_TABLE(of, axxia_mdio_match);

static struct platform_driver axxia_mdio_driver = {
	.driver = {
		.name           = "axxia-mdio",
		.owner          = THIS_MODULE,
		.of_match_table = axxia_mdio_match,
	},
	.probe  = axxia_mdio_probe,
	.remove = axxia_mdio_remove,
};

module_platform_driver(axxia_mdio_driver);

MODULE_AUTHOR("John Jacques");
MODULE_DESCRIPTION("Axxia MDIO Bus Driver");
MODULE_LICENSE("GPL v2");
