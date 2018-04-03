/*
 * drivers/net/ethernet/axxia/axxia_acp_mdio.c
 *
 * Copyright (C) 2013 INTEL Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/irqdomain.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>

#define BZ33327_WA

/* MDIO Access */

struct axxia_mdio_priv {
	unsigned long base;
};

static struct axxia_mdio_priv *mdio_priv;
static DEFINE_SPINLOCK(mdio_lock);

#define MDIO_CONTROL_RD_DATA ((void *)(mdio_priv->base + 0x0))
#define MDIO_STATUS_RD_DATA  ((void *)(mdio_priv->base + 0x4))
#define MDIO_CLK_OFFSET      ((void *)(mdio_priv->base + 0x8))
#define MDIO_CLK_PERIOD      ((void *)(mdio_priv->base + 0xc))

#ifdef CONFIG_ARM
static u32 read_reg(u32 *addr)
{
	return readl_relaxed((void __iomem *)addr);
}

static void write_reg(u32 *addr, u32 value)
{
	writel_relaxed(value, (void __iomem *)addr);
}
#else
static u32 read_reg(u32 *addr)
{
	return in_le32((unsigned *)addr);
}

static void write_reg(u32 *addr, u32 value)
{
	out_le32((unsigned *)addr, (int)value);
}
#endif

/* acp_mdio_read */

int
acp_mdio_read(unsigned long address, unsigned long offset,
	      unsigned short *value, int clause_45)
{
	unsigned long command = 0;
	unsigned long status;
	unsigned long flags;

	spin_lock_irqsave(&mdio_lock, flags);
#if defined(BZ33327_WA)
	/* Set the mdio_busy (status) bit. */
	status = read_reg(MDIO_STATUS_RD_DATA);
	status |= 0x40000000;
	write_reg(MDIO_STATUS_RD_DATA, status);
#endif /* BZ33327_WA */

	if (clause_45 == 0) {
		/* Write the command. */
		command = 0x10000000;              /* op_code: read */
		command |= (address & 0x1f) << 16; /* port_addr (tgt device) */
		command |= (offset & 0x1f) << 21;  /* device_addr (tgt reg) */
		write_reg(MDIO_CONTROL_RD_DATA, command);
	} else {
		/* Step 1: Write the address. */

		/* Write the address */
		command = 0x20000000;                    /* Clause 45 = 1 */
		command |= 0x00000000;                   /* op_code: 0 */
		command |= 0x04000000;                   /* interface_sel = 1 */
		command |= ((offset & 0x1f000000) >> 3); /* device_addr (target
							  * device_type)
							  */
		command |= (address & 0x1f) << 16;       /* port_addr (target
							  * device)
							  */
		command |= (offset & 0xffff);            /* addr_or_data (target
							  * register)
							  */
		write_reg(MDIO_CONTROL_RD_DATA, command);

		/* Wait for the mdio_busy (status) bit to clear. */
		do {
			status = read_reg(MDIO_STATUS_RD_DATA);
		} while (0 != (status & 0x40000000));

		/* Wait for the mdio_busy (control) bit to clear. */
		do {
			command = read_reg(MDIO_CONTROL_RD_DATA);
		} while (0 != (command & 0x80000000));

		/* Step 2: Read the value. */

		/* Set the mdio_busy (status) bit. */
		status = read_reg(MDIO_STATUS_RD_DATA);
		status |= 0x40000000;
		write_reg(MDIO_STATUS_RD_DATA, status);

		command = 0x20000000;                    /* Clause 45 = 1 */
		command |= 0x10000000;                   /* op_code: read */
		command |= 0x04000000;                   /* interface_sel = 1 */
		command |= ((offset & 0x1f000000) >> 3); /* device_addr (target
							  * device_type)
							  */
		command |= (address & 0x1f) << 16;       /* port_addr (target
							  * device)
							  */
		write_reg(MDIO_CONTROL_RD_DATA, command);
	}

#if defined(BZ33327_WA)
	/* Wait for the mdio_busy (status) bit to clear. */
	do {
		status = read_reg(MDIO_STATUS_RD_DATA);
	} while (0 != (status & 0x40000000));
#endif				/* BZ33327_WA */

	/* Wait for the mdio_busy (control) bit to clear. */
	do {
		command = read_reg(MDIO_CONTROL_RD_DATA);
	} while (0 != (command & 0x80000000));

	*value = (unsigned short)(command & 0xffff);
	spin_unlock_irqrestore(&mdio_lock, flags);

	return 0;
}
EXPORT_SYMBOL(acp_mdio_read);

/* acp_mdio_write */

int
acp_mdio_write(unsigned long address, unsigned long offset,
	       unsigned short value, int clause_45)
{
	unsigned long command = 0;
	unsigned long status;
	unsigned long flags;

	spin_lock_irqsave(&mdio_lock, flags);

	/* Wait for mdio_busy (control) to be clear. */
	do {
		command = read_reg(MDIO_CONTROL_RD_DATA);
	} while (0 != (command & 0x80000000));

#if defined(BZ33327_WA)
	/* Set the mdio_busy (status) bit. */
	status = read_reg(MDIO_STATUS_RD_DATA);
	status |= 0x40000000;
	write_reg(MDIO_STATUS_RD_DATA, status);
#endif /* BZ33327_WA */

	if (clause_45 == 0) {
		/* Write the command. */
		command = 0x08000000;              /* op_code: write */
		command |= (address & 0x1f) << 16; /* port_addr (tgt device) */
		command |= (offset & 0x1f) << 21;  /* device_addr (tgt reg) */
		command |= (value & 0xffff);       /* value */
		write_reg(MDIO_CONTROL_RD_DATA, command);
	} else {
		/* Step 1: Write the address. */

		/* Write the address */
		command = 0x20000000;                    /* Clause 45 = 1 */
		command |= 0x00000000;                   /* op_code: 0 */
		command |= 0x04000000;                   /* interface_sel = 1 */
		command |= ((offset & 0x1f000000) >> 3); /* device_addr (target
							  * device_type)
							  */
		command |= (address & 0x1f) << 16;       /* port_addr (target
							  * device)
							  */
		command |= (offset & 0xffff);            /* addr_or_data (target
							  * register)
							  */
		write_reg(MDIO_CONTROL_RD_DATA, command);

		/* Wait for the mdio_busy (status) bit to clear. */
		do {
			status = read_reg(MDIO_STATUS_RD_DATA);
		} while (0 != (status & 0x40000000));

		/* Wait for the mdio_busy (control) bit to clear. */
		do {
			command = read_reg(MDIO_CONTROL_RD_DATA);
		} while (0 != (command & 0x80000000));

		/* Step 2: Write the value. */

		/* Set the mdio_busy (status) bit. */
		status = read_reg(MDIO_STATUS_RD_DATA);
		status |= 0x40000000;
		write_reg(MDIO_STATUS_RD_DATA, status);

		command = 0x20000000;                    /* Clause 45 = 1 */
		command |= 0x08000000;                   /* op_code: write */
		command |= 0x04000000;                   /* interface_sel = 1 */
		command |= ((offset & 0x1f000000) >> 3); /* device_addr (target
							  * device_type)
							  */
		command |= (address & 0x1f) << 16;       /* port_addr (target
							  * device)
							  */
		command |= (value & 0xffff);             /* addr_or_data=value*/
		write_reg(MDIO_CONTROL_RD_DATA, command);
	}

#if defined(BZ33327_WA)
	/* Wait for the mdio_busy (status) bit to clear. */
	do {
		status = read_reg(MDIO_STATUS_RD_DATA);
	} while (0 != (status & 0x40000000));
#endif	/* BZ33327_WA */

	/* Wait for the mdio_busy (control) bit to clear. */
	do {
		command = read_reg(MDIO_CONTROL_RD_DATA);
	} while (0 != (command & 0x80000000));

	spin_unlock_irqrestore(&mdio_lock, flags);

	return 0;
}
EXPORT_SYMBOL(acp_mdio_write);

/* acp_mdio_initialize */

static void
acp_mdio_initialize(int offset, int period)
{
	write_reg(MDIO_CLK_OFFSET, offset);
	write_reg(MDIO_CLK_PERIOD, period);

	return;
}

/* acp_wrappers_init */

int __init
acp_mdio_init(void)
{
	int rc = -ENODEV;
	struct device_node *np = NULL;
	const u32 *field;
	void __iomem *map;
	u64 mdio_address;
	u32 mdio_size;
	u32 mdio_offset = 0;
	u32 mdio_period = 0;

	pr_info("MDIO: Initializing Axxia Wrappers.\n");

	mdio_priv = kzalloc(sizeof(struct axxia_mdio_priv), GFP_KERNEL);
	if (!mdio_priv)
		return -ENOMEM;

	np = of_find_node_by_type(np, "network");

	while (np &&
	       !of_device_is_compatible(np, "axxia,acp-femac") &&
	       !of_device_is_compatible(np, "acp-femac"))
		np = of_find_node_by_type(np, "network");

	if (!np) {
		pr_warn("MDIO: No compatible devices found.\n");
		rc = -EINVAL;
		goto error;
	}

	field = of_get_property(np, "mdio-reg", NULL);

	if (!field) {
		pr_crit("MDIO: Unable to read mdio-reg property!\n");
		rc = -EINVAL;
		goto error;
	}

	mdio_address = of_translate_address(np, field);

	if (mdio_address == OF_BAD_ADDR) {
		pr_crit("MDIO: of_translate_address failed!\n");
		rc = -EINVAL;
		goto error;
	}

	mdio_size = field[3];
	map = ioremap(mdio_address, mdio_size);

	if (!map) {
		pr_crit("MDIO: Unable to ioremap!\n");
		rc = -ENOMEM;
		goto error;
	}

	mdio_priv->base = (unsigned long)map;
	field = of_get_property(np, "mdio-clock-offset", NULL);

	if (field)
		mdio_offset = ntohl(field[0]);

	field = of_get_property(np, "mdio-clock-period", NULL);

	if (field)
		mdio_period = ntohl(field[0]);

	if (0 != mdio_offset && 0 != mdio_period)
		acp_mdio_initialize(mdio_offset, mdio_period);

error:
	return rc;
}

module_init(acp_mdio_init);

MODULE_AUTHOR("INTEL Corporation");
MODULE_DESCRIPTION("Timing Test");
MODULE_LICENSE("GPL");
