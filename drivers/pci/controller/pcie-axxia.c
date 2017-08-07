/*
 * PCIe host controller driver for Intel's AXXIA X9/LF devices
 *
 * Copyright (C) 2015 Intel Electronics Co., Ltd.
 *		http://www.intel.com
 *
 * Author: Sangeetha Rao <sangeetha.rao@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/proc_fs.h>
#include <linux/axxia-pei.h>

#include "pcie-axxia.h"
#include "../pci.h"

#define PEI_GENERAL_CORE_CTL_REG 0x38
#define PEI_SII_PWR_MGMT_REG 0xD4
#define PEI_SII_DBG_0_MON_REG 0xEC

#define PEI_SMLH_LINK_UP (0x1 << 12)
#define PEI_SMLH_LINK_STATE (0x3f << 4)
#define PEI_RDLH_LINK_UP (0x1 << 0)

/* Synopsis specific PCIE configuration registers */
#define PCIE_PORT_LINK_CONTROL          0x710
#define PORT_LINK_MODE_MASK             (0x3f << 16)
#define PORT_LINK_MODE_1_LANES          (0x1 << 16)
#define PORT_LINK_MODE_2_LANES          (0x3 << 16)
#define PORT_LINK_MODE_4_LANES          (0x7 << 16)
#define PORT_LINK_MODE_8_LANES          (0xf << 16)

#define PCIE_LINK_WIDTH_SPEED_CONTROL   0x80C
#define PORT_LOGIC_SPEED_CHANGE         (0x1 << 17)
#define PORT_LOGIC_LINK_WIDTH_MASK      (0x1f << 8)
#define PORT_LOGIC_LINK_WIDTH_1_LANES   (0x1 << 8)
#define PORT_LOGIC_LINK_WIDTH_2_LANES   (0x2 << 8)
#define PORT_LOGIC_LINK_WIDTH_4_LANES   (0x4 << 8)
#define PORT_LOGIC_LINK_WIDTH_8_LANES   (0x8 << 8)

#define PCIE_GEN3_EQ_CONTROL_OFF        0x8a8

#define PCIE_MSI_ADDR_LO                0x820
#define PCIE_MSI_ADDR_HI                0x824
#define PCIE_MSI_INTR0_ENABLE           0x828
#define PCIE_MSI_INTR0_MASK             0x82C
#define PCIE_MSI_INTR0_STATUS           0x830

#define PCIE_ATU_VIEWPORT               0x900
#define PCIE_ATU_REGION_INBOUND         (0x1 << 31)
#define PCIE_ATU_REGION_OUTBOUND        (0x0 << 31)
#define PCIE_ATU_REGION_INDEX3          (0x3 << 0)
#define PCIE_ATU_REGION_INDEX2          (0x2 << 0)
#define PCIE_ATU_REGION_INDEX1          (0x1 << 0)
#define PCIE_ATU_REGION_INDEX0          (0x0 << 0)
#define PCIE_ATU_CR1                    0x904
#define PCIE_ATU_TYPE_MEM               (0x0 << 0)
#define PCIE_ATU_TYPE_IO                (0x2 << 0)
#define PCIE_ATU_TYPE_CFG0              (0x4 << 0)
#define PCIE_ATU_TYPE_CFG1              (0x5 << 0)
#define PCIE_ATU_CR2                    0x908
#define PCIE_ATU_ENABLE                 (0x1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE        (0x1 << 30)
#define PCIE_ATU_LOWER_BASE             0x90C
#define PCIE_ATU_UPPER_BASE             0x910
#define PCIE_ATU_LIMIT                  0x914
#define PCIE_ATU_LOWER_TARGET           0x918
#define PCIE_ATU_BUS(x)                 (((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)                 (((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)                (((x) & 0x7) << 16)
#define PCIE_ATU_UPPER_TARGET           0x91C

#define CC_GPREG_EDG_IRQ_STAT                    0x210
#define CC_GPREG_EDG_IRQ_MASK                    0x214
#define CC_GPREG_EDG_IRQ_STAT_HI                 0x250
#define CC_GPREG_EDG_IRQ_MASK_HI                 0x254
#define MSI_ASSERTED                    (0x1 << 24)
#define RADM_INTD_DEASSERTED            (0x1 << 9)
#define RADM_INTC_DEASSERTED            (0x1 << 8)
#define RADM_INTB_DEASSERTED            (0x1 << 7)
#define RADM_INTA_DEASSERTED            (0x1 << 6)
#define RADM_INTD_ASSERTED              (0x1 << 5)
#define RADM_INTC_ASSERTED              (0x1 << 4)
#define RADM_INTB_ASSERTED              (0x1 << 3)
#define RADM_INTA_ASSERTED              (0x1 << 2)

#define CC_GPREG_LVL_IRQ_MASK	0x204
#define MSI_CNTRL_INT              (0x1 << 9)

#define AXI_GPREG_MSTR		0x0
#define CFG_MSI_MODE		(0x1 << 29)

/* SYSCON */
#define AXXIA_SYSCON_BASE             0x8002C00000


static inline uint32_t axxia_mmio_read_32(uintptr_t addr)
{
	return *(uint32_t *)addr;
}

int
axxia_is_x9(void)
{
	if (of_find_compatible_node(NULL, NULL, "lsi,axm5616"))
		return 1;

	return 0;
}

struct axxia_pcie {
	struct pcie_port pp;
	struct axxia_pcie *next;
};

static struct axxia_pcie *axxia_pcie_controllers;

static unsigned long global_io_offset;

static inline void axxia_pcie_readl_rc(struct pcie_port *pp, u32 reg, u32 *val)
{
	*val = readl(pp->dbi_base + reg);
}

static inline void axxia_pcie_writel_rc(struct pcie_port *pp, u32 val, u32 reg)
{
	writel(val, pp->dbi_base + reg);
}

static inline void axxia_cc_gpreg_writel(struct pcie_port *pp, u32 val, u32 reg)
{
	writel(val, pp->cc_gpreg_base + reg);
}

static inline void axxia_cc_gpreg_readl(struct pcie_port *pp, u32 reg, u32 *val)
{
	*val = readl(pp->cc_gpreg_base + reg);
}

static inline void axxia_axi_gpreg_writel(struct pcie_port *pp, u32 val,
					  u32 reg)
{
	writel(val, pp->axi_gpreg_base + reg);
}

static inline void axxia_axi_gpreg_readl(struct pcie_port *pp, u32 reg,
					 u32 *val)
{
	*val = readl(pp->axi_gpreg_base + reg);
}

int axxia_pcie_cfg_read(void __iomem *addr, int where, int size, u32 *val)
{
	*val = readl(addr);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;
	else if (size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

int axxia_pcie_cfg_write(void __iomem *addr, int where, int size, u32 val)
{
	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr + (where & 2));
	else if (size == 1)
		writeb(val, addr + (where & 3));
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

static int axxia_pcie_rd_own_conf(struct pcie_port *pp, int where,
				  int size, u32 *val)
{
	int ret;

	ret = axxia_pcie_cfg_read(pp->dbi_base + (where & ~0x3),
				  where, size, val);
	return ret;
}

static int axxia_pcie_wr_own_conf(struct pcie_port *pp, int where,
				  int size, u32 val)
{
	int ret;

	ret = axxia_pcie_cfg_write(pp->dbi_base + (where & ~0x3), where,
				   size, val);
	return ret;
}

static void axxia_pcie_prog_viewport_cfg0(struct pcie_port *pp, u32 busdev)
{
	u32 upper_base;

	/* Program viewport 0 : OUTBOUND : CFG0 */
	axxia_pcie_writel_rc(pp,
			     PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX0,
			     PCIE_ATU_VIEWPORT);
	axxia_pcie_writel_rc(pp, pp->cfg0_base, PCIE_ATU_LOWER_BASE);
	/* set upper base bits [1:0] for X9, bits[7:0] for XLF */
	upper_base = (pp->cfg0_base >> 32);
	upper_base &= (axxia_is_x9()) ? 0x3 : 0xff;
	axxia_pcie_writel_rc(pp, upper_base, PCIE_ATU_UPPER_BASE);
	axxia_pcie_writel_rc(pp, pp->cfg0_base + pp->cfg0_size - 1,
			     PCIE_ATU_LIMIT);
	axxia_pcie_writel_rc(pp, busdev, PCIE_ATU_LOWER_TARGET);
	axxia_pcie_writel_rc(pp, 0, PCIE_ATU_UPPER_TARGET);
	axxia_pcie_writel_rc(pp, PCIE_ATU_TYPE_CFG0, PCIE_ATU_CR1);
	axxia_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}


static void axxia_pcie_prog_viewport_cfg1(struct pcie_port *pp, u32 busdev)
{
	u32 upper_base;

	/* Program viewport 1 : OUTBOUND : CFG1 */
	axxia_pcie_writel_rc(pp,
			     PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX1,
			     PCIE_ATU_VIEWPORT);
	axxia_pcie_writel_rc(pp, PCIE_ATU_TYPE_CFG1, PCIE_ATU_CR1);
	axxia_pcie_writel_rc(pp, pp->cfg1_base, PCIE_ATU_LOWER_BASE);
	upper_base = (pp->cfg1_base >> 32);
	upper_base &= (axxia_is_x9()) ? 0x3 : 0xff;
	axxia_pcie_writel_rc(pp, upper_base, PCIE_ATU_UPPER_BASE);

	axxia_pcie_writel_rc(pp, pp->cfg1_base + pp->cfg1_size - 1,
			     PCIE_ATU_LIMIT);
	axxia_pcie_writel_rc(pp, busdev, PCIE_ATU_LOWER_TARGET);
	axxia_pcie_writel_rc(pp, 0, PCIE_ATU_UPPER_TARGET);
	axxia_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}

static void axxia_pcie_prog_viewport_mem_outbound(struct pcie_port *pp)
{
	u32 upper_base;

	/* Program viewport 0 : OUTBOUND : MEM */
	axxia_pcie_writel_rc(pp,
			     PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX2,
			     PCIE_ATU_VIEWPORT);
	axxia_pcie_writel_rc(pp, PCIE_ATU_TYPE_MEM, PCIE_ATU_CR1);
	axxia_pcie_writel_rc(pp, pp->mem_mod_base, PCIE_ATU_LOWER_BASE);
	upper_base = (pp->mem_mod_base >> 32);
	upper_base &= (axxia_is_x9()) ? 0x3 : 0xff;
	axxia_pcie_writel_rc(pp, upper_base, PCIE_ATU_UPPER_BASE);
	axxia_pcie_writel_rc(pp, pp->mem_mod_base + pp->mem_size - 1,
			     PCIE_ATU_LIMIT);
	axxia_pcie_writel_rc(pp, pp->mem_bus_addr, PCIE_ATU_LOWER_TARGET);
	axxia_pcie_writel_rc(pp, upper_32_bits(pp->mem_bus_addr),
			     PCIE_ATU_UPPER_TARGET);
	axxia_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}


static void axxia_pcie_prog_viewport_io_outbound(struct pcie_port *pp)
{
	/* Program viewport 1 : OUTBOUND : IO */
	axxia_pcie_writel_rc(pp,
			     PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX3,
			     PCIE_ATU_VIEWPORT);
	axxia_pcie_writel_rc(pp, PCIE_ATU_TYPE_IO, PCIE_ATU_CR1);
	axxia_pcie_writel_rc(pp, pp->io_mod_base, PCIE_ATU_LOWER_BASE);
	axxia_pcie_writel_rc(pp, pp->io_mod_base + pp->io_size - 1,
			     PCIE_ATU_LIMIT);
	axxia_pcie_writel_rc(pp, pp->io_bus_addr, PCIE_ATU_LOWER_TARGET);
	axxia_pcie_writel_rc(pp, upper_32_bits(pp->io_bus_addr),
			     PCIE_ATU_UPPER_TARGET);
	axxia_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}


static int axxia_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				    u32 devfn, int where, int size, u32 *val)
{
	int ret = PCIBIOS_SUCCESSFUL;
	u32 address, busdev;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		PCIE_ATU_FUNC(PCI_FUNC(devfn));
	address = where & ~0x3;

	if (bus->parent->number == pp->root_bus_nr) {
		axxia_pcie_prog_viewport_cfg0(pp, busdev);
		ret = axxia_pcie_cfg_read(pp->va_cfg0_base + address, where,
					  size, val);
		axxia_pcie_prog_viewport_mem_outbound(pp);
	} else {
		axxia_pcie_prog_viewport_cfg1(pp, busdev);
		ret = axxia_pcie_cfg_read(pp->va_cfg1_base + address, where,
					  size, val);
		axxia_pcie_prog_viewport_io_outbound(pp);
	}
	return ret;
}

static int axxia_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				    u32 devfn, int where, int size, u32 val)
{
	int ret = PCIBIOS_SUCCESSFUL;
	u32 address, busdev;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		PCIE_ATU_FUNC(PCI_FUNC(devfn));
	address = where & ~0x3;

	if (bus->parent->number == pp->root_bus_nr) {
		axxia_pcie_prog_viewport_cfg0(pp, busdev);
		ret = axxia_pcie_cfg_write(pp->va_cfg0_base + address,
					   where, size, val);
		axxia_pcie_prog_viewport_mem_outbound(pp);
	} else {
		axxia_pcie_prog_viewport_cfg1(pp, busdev);
		ret = axxia_pcie_cfg_write(pp->va_cfg1_base + address,
					   where, size, val);
		axxia_pcie_prog_viewport_io_outbound(pp);
	}
	return ret;
}

static int axxia_pcie_valid_config(struct pcie_port *pp,
				   struct pci_bus *bus, int dev)
{
	/* If there is no link, then there is no device */
	if (bus->number != pp->root_bus_nr) {
		if (!axxia_pcie_link_up(pp))
			return 0;
	}

	/* access only one slot on each root port */
	if (bus->number == pp->root_bus_nr && dev > 0)
		return 0;

	/*
	 * do not read more than one device on the bus directly attached
	 * to RC's (Virtual Bridge's) DS side.
	 */
	if (bus->primary == pp->root_bus_nr && dev > 0)
		return 0;

	return 1;
}


/*
 * Read PCI config space
 */
static int
axxia_pciex_read_config(struct pci_bus *bus, unsigned int devfn,
			int offset, int len, u32 *val) {
	struct pcie_port *pp = bus->sysdata;
	int ret;

	if (axxia_pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (bus->number != pp->root_bus_nr)
		ret = axxia_pcie_rd_other_conf(pp, bus, devfn,
					       offset, len, val);
	else
		ret = axxia_pcie_rd_own_conf(pp, offset, len, val);

	return ret;
}

/*
 * Write PCI config space.
 */
static int
axxia_pciex_write_config(struct pci_bus *bus, unsigned int devfn,
			 int offset, int len, u32 val)
{
	struct pcie_port *pp = bus->sysdata;
	int ret;

	if (axxia_pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number != pp->root_bus_nr)
		ret = axxia_pcie_wr_other_conf(pp, bus, devfn,
					       offset, len, val);
	else
		ret = axxia_pcie_wr_own_conf(pp, offset, len, val);

	return ret;
}
static struct pci_ops axxia_pciex_pci_ops = {
	.read  = axxia_pciex_read_config,
	.write = axxia_pciex_write_config,
};

static struct irq_chip axxia_dw_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static int axxia_dw_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
				 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &axxia_dw_msi_irq_chip,
				 handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops axxia_msi_domain_ops = {
	.map = axxia_dw_pcie_msi_map,
};


void axxia_dw_pcie_msi_init(struct pcie_port *pp)
{
	u64 msi_target;
	pp->msi_data = __get_free_pages(GFP_KERNEL, 0);

	msi_target = virt_to_phys((void *)pp->msi_data);
	dev_info(pp->dev,
		"%s: MSI addr virt: %lx, phys: %llx\n",
		__func__, pp->msi_data, msi_target);
	/* program the msi_data */
	axxia_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4,
			    (u32)(msi_target & 0xffffffff));
	axxia_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4,
			    (u32)(msi_target >> 32 & 0xffffffff));

}

/* MSI int handler */
irqreturn_t axxia_dw_handle_msi_irq(struct pcie_port *pp)
{
	unsigned long val;
	int i, pos, irq;
	irqreturn_t ret = IRQ_NONE;

	for (i = 0; i < MAX_MSI_CTRLS; i++) {
		axxia_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4,
				(u32 *)&val);
		if (val) {
			ret = IRQ_HANDLED;
			pos = 0;
			while ((pos = find_next_bit(&val, 32, pos)) != 32) {
				irq = irq_find_mapping(pp->irq_domain,
						i * 32 + pos);
				axxia_pcie_wr_own_conf(pp,
						PCIE_MSI_INTR0_STATUS + i * 12,
						4, 1 << pos);
				generic_handle_irq(irq);
				pos++;
			}
		}
	}
	return ret;
}

static void axxia_pcie_msi_init(struct pcie_port *pp)
{
	axxia_dw_pcie_msi_init(pp);
}

void axxia_dw_pcie_handle_msi_irq(struct pcie_port *pp, int offset)
{
	unsigned long val;
	int i, pos, irq;

	for (i = 0; i < MAX_MSI_CTRLS; i++) {
		axxia_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4,
				(u32 *)&val);
		if (val) {
			pos = 0;
			while ((pos = find_next_bit(&val, 32, pos)) != 32) {

				dev_dbg(pp->dev,
				 "msi valid i = %d, val = %lx, pos = %d\n",
							i, val, pos);
				irq = irq_find_mapping(pp->irq_domain,
						i * 32 + pos);
				axxia_pcie_wr_own_conf(pp,
						PCIE_MSI_INTR0_STATUS + i * 12,
						4, 1 << pos);
				generic_handle_irq(irq);
				pos++;
			}
		}
	}
}

static void axxia_pcie_msi_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct pcie_port *pp = irq_desc_get_handler_data(desc);
	u32 offset = irq - pp->msi_irqs[0];
	struct irq_chip *chip = irq_desc_get_chip(desc);

	dev_dbg(pp->dev, "%s, irq %d i %d\n", __func__, irq, offset);

	/*
	 * The chained irq handler installation would have replaced normal
	 * interrupt driver handler so we need to take care of mask/unmask and
	 * ack operation.
	 */
	chained_irq_enter(chip, desc);
	axxia_dw_pcie_handle_msi_irq(pp, offset);
	chained_irq_exit(chip, desc);
}

static void axxia_pcie_enable_interrupts(struct pcie_port *pp)
{
	u32 val;
	int i;

	/* Unmask */
	axxia_cc_gpreg_readl(pp, CC_GPREG_EDG_IRQ_MASK, &val);
	val |= (RADM_INTD_DEASSERTED | RADM_INTC_DEASSERTED |
		RADM_INTB_DEASSERTED | RADM_INTA_DEASSERTED |
		RADM_INTD_ASSERTED | RADM_INTC_ASSERTED |
		RADM_INTB_ASSERTED | RADM_INTA_ASSERTED);
	axxia_cc_gpreg_writel(pp, val, CC_GPREG_EDG_IRQ_MASK);
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		/* unmask MSI */
		if (pp->num_msi_irqs == 0) {
			axxia_cc_gpreg_readl(pp,
				CC_GPREG_EDG_IRQ_MASK_HI, &val);
			val |= MSI_ASSERTED;
			axxia_cc_gpreg_writel(pp, val,
				CC_GPREG_EDG_IRQ_MASK_HI);
		} else {
			for (i = 0; i < pp->num_msi_irqs; i++) {
				irq_set_chained_handler(pp->msi_irqs[i],
					axxia_pcie_msi_irq_handler);
				irq_set_handler_data(pp->msi_irqs[i], pp);
			}
		}
		axxia_pcie_msi_init(pp);
	}
}

int axxia_pcie_link_up(struct pcie_port *pp)
{
	u32 rdlh_lnk, smlh_lnk, smlh_state;

	axxia_cc_gpreg_readl(pp, PEI_SII_PWR_MGMT_REG, &smlh_lnk);
	axxia_cc_gpreg_readl(pp, PEI_SII_DBG_0_MON_REG, &rdlh_lnk);

	axxia_cc_gpreg_readl(pp, PEI_SII_PWR_MGMT_REG, &smlh_state);
	smlh_state = (smlh_state & PEI_SMLH_LINK_STATE) >> 4;

	if ((smlh_state != 0x11) && (smlh_state != 0x23)) {
		dev_err(pp->dev, "PCIe LINK IS NOT UP: smlh_state = %x\n",
			smlh_state);
		return 0;
	}
	return 1;
}

void axxia_pcie_setup_rc(struct pcie_port *pp)
{
	u32 val;
	u32 membase;
	u32 memlimit;

	axxia_pcie_rd_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, &val);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (pp->lanes) {
	case 2:
		val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
		break;
	case 4:
		val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
		break;
	case 8:
		val |= PORT_LOGIC_LINK_WIDTH_8_LANES;
		break;
	case 1:
	default:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
	}
	axxia_pcie_wr_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, val);

	/* Set the number of lanes based on the device tree. */
	axxia_pcie_rd_own_conf(pp, PCIE_PORT_LINK_CONTROL, 4, &val);
	val &= ~PORT_LINK_MODE_MASK;

	switch (pp->lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	case 2:
		val |= PORT_LINK_MODE_2_LANES;
		break;
	case 4:
		val |= PORT_LINK_MODE_4_LANES;
		break;
	case 8:
		val |= PORT_LINK_MODE_8_LANES;
		break;
	default:
		break;
	}
	axxia_pcie_wr_own_conf(pp, PCIE_PORT_LINK_CONTROL, 4, val);

	/* Add Mikes tweak for GEN3_EQ_CONTROL */
	axxia_pcie_writel_rc(pp, 0x1017201, PCIE_GEN3_EQ_CONTROL_OFF);

	/* setup bus numbers */
	axxia_pcie_readl_rc(pp, PCI_PRIMARY_BUS, &val);
	val &= 0xff000000;
	val |= 0x00010100;
	axxia_pcie_writel_rc(pp, val, PCI_PRIMARY_BUS);

	/* setup memory base, memory limit */
	membase = ((u32)pp->mem_base & 0xfff00000) >> 16;
	memlimit = (pp->mem_size + (u32)pp->mem_base) & 0xfff00000;
	val = memlimit | membase;
	axxia_pcie_writel_rc(pp, val, PCI_MEMORY_BASE);

	/* setup command register */
	axxia_pcie_readl_rc(pp, PCI_COMMAND, &val);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	axxia_pcie_writel_rc(pp, val, PCI_COMMAND);

	/* LTSSM enable */
	axxia_cc_gpreg_readl(pp, PEI_GENERAL_CORE_CTL_REG, &val);
	msleep(100);
	val |= 0x1;
	axxia_cc_gpreg_writel(pp, val, PEI_GENERAL_CORE_CTL_REG);
	msleep(100);
}


static int axxia_pcie_establish_link(struct pcie_port *pp)
{
	/* setup root complex */
	axxia_pcie_setup_rc(pp);

	if (axxia_pcie_link_up(pp))
		dev_info(pp->dev, "Link up\n");
	else
		return 1;

	return 0;
}

static void pci_axxia_program_rc_class(struct pcie_port *pp)
{
	u32 dbi_ro_wr_en;
	/* program correct class for RC */
	axxia_pcie_readl_rc(pp, 0x8bc, &dbi_ro_wr_en);
	/* DBI_RO_WR_EN */
	if (!(dbi_ro_wr_en & 0x1))
		axxia_pcie_writel_rc(pp, (dbi_ro_wr_en | 0x1), 0x8bc);
	axxia_pcie_wr_own_conf(pp, PCI_CLASS_DEVICE, 2, PCI_CLASS_BRIDGE_PCI);
	/* DBI_RO_WR_EN */
	if (!(dbi_ro_wr_en & 0x1))
		axxia_pcie_writel_rc(pp, dbi_ro_wr_en, 0x8bc);
}

static irqreturn_t axxia_pcie_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;
	u32 val;
	irqreturn_t ret;

	axxia_cc_gpreg_readl(pp, CC_GPREG_EDG_IRQ_STAT, &val);
	if (val & RADM_INTD_DEASSERTED)
		pr_info("RADM_INTD_DEASSERTED\n");
	if (val & RADM_INTC_DEASSERTED)
		pr_info("RADM_INTC_DEASSERTED\n");
	if (val & RADM_INTB_DEASSERTED)
		pr_info("RADM_INTB_DEASSERTED\n");
	if (val & RADM_INTA_DEASSERTED)
		pr_info("RADM_INTA_DEASSERTED\n");
	if (val & RADM_INTD_ASSERTED)
		pr_info("RADM_INTD_ASSERTED\n");
	if (val & RADM_INTC_ASSERTED)
		pr_info("RADM_INTC_ASSERTED\n");
	if (val & RADM_INTB_ASSERTED)
		pr_info("RADM_INTB_ASSERTED\n");
	if (val & RADM_INTA_ASSERTED)
		pr_info("RADM_INTA_ASSERTED\n");
	/* Clear the legacy interrupts */
	axxia_cc_gpreg_writel(pp, val,
			      CC_GPREG_EDG_IRQ_STAT);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		if (pp->num_msi_irqs == 0) {
			axxia_cc_gpreg_readl(pp,
				CC_GPREG_EDG_IRQ_STAT_HI, &val);
			if (val & MSI_ASSERTED) {
				ret = axxia_dw_handle_msi_irq(pp);
				axxia_cc_gpreg_writel(pp, MSI_ASSERTED,
					      CC_GPREG_EDG_IRQ_STAT_HI);
				return ret;
			}
		}
	}
	return IRQ_HANDLED;
}

static void axxia_dw_pcie_msi_clear_irq(struct pcie_port *pp, int irq)
{
	unsigned int res, bit, val;

	res = (irq / 32) * 12;
	bit = irq % 32;
	axxia_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
	val &= ~(1 << bit);
	axxia_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);
}

static void clear_irq_range(struct pcie_port *pp, unsigned int irq_base,
			    unsigned int nvec, unsigned int pos)
{
	unsigned int i;

	for (i = 0; i < nvec; i++) {
		irq_set_msi_desc_off(irq_base, i, NULL);
		/* Disable corresponding interrupt on MSI controller */
		axxia_dw_pcie_msi_clear_irq(pp, pos + i);
	}

	bitmap_release_region(pp->msi_irq_in_use, pos, order_base_2(nvec));
}

static void axxia_dw_pcie_msi_set_irq(struct pcie_port *pp, int irq)
{
	unsigned int res, bit, val;

	res = (irq / 32) * 12;
	bit = irq % 32;
	axxia_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
	val |= 1 << bit;
	axxia_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);
}

static int assign_irq(int no_irqs, struct msi_desc *desc, int *pos)
{
	int irq, pos0, i;
	struct pcie_port *pp = (struct pcie_port *)msi_desc_to_pci_sysdata(desc);

	pos0 = bitmap_find_free_region(pp->msi_irq_in_use, MAX_MSI_IRQS,
				       order_base_2(no_irqs));
	if (pos0 < 0)
		goto no_valid_irq;

	irq = irq_find_mapping(pp->irq_domain, pos0);
	if (!irq)
		goto no_valid_irq;

	/*
	 * irq_create_mapping (called from dw_pcie_host_init) pre-allocates
	 * descs so there is no need to allocate descs here. We can therefore
	 * assume that if irq_find_mapping above returns non-zero, then the
	 * descs are also successfully allocated.
	 */
	for (i = 0; i < no_irqs; i++) {
		if (irq_set_msi_desc_off(irq, i, desc) != 0) {
			clear_irq_range(pp, irq, i, pos0);
			goto no_valid_irq;
		}
		/*Enable corresponding interrupt in MSI interrupt controller */
		axxia_dw_pcie_msi_set_irq(pp, pos0 + i);
	}

	*pos = pos0;
	return irq;

no_valid_irq:
	*pos = pos0;
	return -ENOSPC;
}

static void axxia_msi_setup_msg(struct pcie_port *pp, unsigned int irq, u32 pos)
{
	struct msi_msg msg;
	u64 msi_target;

	msi_target = virt_to_phys((void *)pp->msi_data);
	msg.address_lo = (u32)(msi_target & 0xffffffff);
	msg.address_hi = (u32)(msi_target >> 32 & 0xffffffff);
	msg.data = pos;

	pci_write_msi_msg(irq, &msg);
}

static int axxia_dw_msi_setup_irq(struct msi_controller *chip,
				  struct pci_dev *pdev,
				  struct msi_desc *desc)
{
	int irq, pos;
	struct pcie_port *pp = pdev->bus->sysdata;

	if (desc->msi_attrib.is_msix)
		return -EINVAL;

	irq = assign_irq(1, desc, &pos);
	if (irq < 0)
		return irq;

	axxia_msi_setup_msg(pp, irq, pos);
	return 0;
}

static void axxia_dw_msi_teardown_irq(struct msi_controller *chip,
				      unsigned int irq)
{
	struct irq_data *data = irq_get_irq_data(irq);
	struct msi_desc *msi = irq_data_get_msi_desc(data);
	struct pcie_port *pp = (struct pcie_port *)msi_desc_to_pci_sysdata(msi);

	clear_irq_range(pp, irq, 1, data->hwirq);
}

static struct msi_controller axxia_dw_pcie_msi_chip = {
	.setup_irq = axxia_dw_msi_setup_irq,
	.teardown_irq = axxia_dw_msi_teardown_irq,
};

int axxia_pcie_host_init(struct pcie_port *pp)
{
	struct device_node *np = pp->dev->of_node;
	struct platform_device *pdev = to_platform_device(pp->dev);
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	u32 val, na, ns;
	int ret;
	struct pci_bus *bus;
	unsigned long mem_offset;
	LIST_HEAD(res);
	int i;

	/* Find the address cell size and the number of cells in order to get
	 * the untranslated address.
	 */
	of_property_read_u32(np, "#address-cells", &na);
	ns = of_n_size_cells(np);

	if (of_pci_range_parser_init(&parser, np)) {
		dev_err(pp->dev, "missing ranges property\n");
		return -EINVAL;
	}

	/* Get the I/O and memory ranges from DT */
	for_each_of_pci_range(&parser, &range) {
		unsigned long restype = range.flags & IORESOURCE_TYPE_BITS;

		if (restype == IORESOURCE_IO) {
			of_pci_range_to_resource(&range, np, &pp->io);
			pp->io.name = "I/O";
			pp->io.start = max_t(resource_size_t,
					     PCIBIOS_MIN_IO,
					     range.pci_addr + global_io_offset);
			pp->io.end = min_t(resource_size_t,
					   IO_SPACE_LIMIT,
					   range.pci_addr + range.size
					   + global_io_offset - 1);
			pp->io_size = resource_size(&pp->io);
			pp->io_bus_addr = range.pci_addr;
			pp->io_base = range.cpu_addr;

			/* Find the untranslated IO space address */
			pp->io_mod_base = of_read_number(parser.range -
							 parser.np + na, ns);
		}
		if (restype == IORESOURCE_MEM) {
			of_pci_range_to_resource(&range, np, &pp->mem);
			pp->mem.name = "MEM";
			pp->mem_size = resource_size(&pp->mem);
			pp->mem_bus_addr = range.pci_addr;

			/* Find the untranslated MEM space address */
			pp->mem_mod_base = of_read_number(parser.range -
							  parser.np + na, ns);
			pp->mem_mod_base = pp->mem.start;
		}
		if (restype == 0) {
			of_pci_range_to_resource(&range, np, &pp->cfg);
			pp->cfg0_size = resource_size(&pp->cfg)/2;
			pp->cfg1_size = resource_size(&pp->cfg)/2;
			pp->cfg0_base = pp->cfg.start;
			pp->cfg1_base = pp->cfg.start + pp->cfg0_size;
		}
	}

	ret = of_pci_parse_bus_range(np, &pp->busn);
	if (ret < 0) {
		pp->busn.name = np->name;
		pp->busn.start = 0;
		pp->busn.end = 0xff;
		pp->busn.flags = IORESOURCE_BUS;
		dev_dbg(pp->dev,
			"failed to parse bus-range property: %d, using default %pR\n",
			ret, &pp->busn);
	}

	mem_offset = pp->mem.start - pp->mem_bus_addr;
	pci_add_resource_offset(&res, &pp->mem, mem_offset);
	pci_add_resource(&res, &pp->busn);
	pp->mem_base = pp->mem.start;

	if (!pp->va_cfg0_base) {
		pp->va_cfg0_base = devm_ioremap(pp->dev, pp->cfg0_base,
						pp->cfg0_size);
		if (!pp->va_cfg0_base) {
			dev_err(pp->dev, "error with ioremap in function\n");
			return -ENOMEM;
		}
	}

	if (!pp->va_cfg1_base) {
		pp->va_cfg1_base = devm_ioremap(pp->dev, pp->cfg1_base,
						pp->cfg1_size);
		if (!pp->va_cfg1_base) {
			dev_err(pp->dev, "error with ioremap\n");
			return -ENOMEM;
		}
	}

	if (of_property_read_u32(np, "num-lanes", &pp->lanes)) {
		dev_err(pp->dev, "Failed to parse the number of lanes\n");
		return -EINVAL;
	}

	/* program correct class for RC */
	pci_axxia_program_rc_class(pp);

	if (axxia_pcie_establish_link(pp))
		dev_warn(pp->dev, "axxia_pcie_establish_link failed\n");

	/* Legacy interrupts */
	pp->irqs = platform_get_irq(pdev, 0);
	if (!pp->irqs) {
		dev_err(pp->dev, "failed to get irq\n");
		return -ENODEV;
	}
	pp->num_msi_irqs = 0;
	for (i = 0; i < AXXIA_MSI_IRQL; i++) {
		pp->msi_irqs[i] = platform_get_irq(pdev, i+2);
		if (pp->msi_irqs[i] <= 0)
			break;
		pp->num_msi_irqs++;
	}
	dev_info(pp->dev, "num_msi = %d, irq = %d\n",
			pp->num_msi_irqs, pp->irqs);

	ret = devm_request_irq(pp->dev, pp->irqs, axxia_pcie_irq_handler,
			       IRQF_SHARED | IRQF_NO_THREAD, "axxia-pcie", pp);
	if (ret) {
		dev_err(pp->dev, "failed to request irq\n");
		return ret;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->irq_domain = irq_domain_add_linear(pp->dev->of_node,
					MAX_MSI_IRQS, &axxia_msi_domain_ops,
					&axxia_dw_pcie_msi_chip);
		if (!pp->irq_domain) {
			dev_err(pp->dev, "irq domain init failed\n");
			return -ENXIO;
		}

		for (i = 0; i < MAX_MSI_IRQS; i++)
			irq_create_mapping(pp->irq_domain, i);
	}

	axxia_pcie_enable_interrupts(pp);

	bus = pci_create_root_bus(&pdev->dev, pp->root_bus_nr,
				  &axxia_pciex_pci_ops, pp, &res);
	if (!bus)
		return 1;
#ifdef CONFIG_PCI_MSI
	axxia_axi_gpreg_readl(pp, AXI_GPREG_MSTR, &val);
	val |= CFG_MSI_MODE;
	axxia_axi_gpreg_writel(pp, val, AXI_GPREG_MSTR);
	bus->msi = &axxia_dw_pcie_msi_chip;
#endif

	pci_scan_child_bus(bus);
	pci_assign_unassigned_bus_resources(bus);
	pci_bus_add_devices(bus);

	return 0;
}

static int axxia_pcie_probe(struct platform_device *pdev)
{
	struct axxia_pcie *axxia_pcie;
	struct pcie_port *pp;
	struct resource *res;
	int ret;
	struct device_node *pei_control;
	const unsigned int *control;
	unsigned int control_value;
	const unsigned int *initialized;
	unsigned int initialized_value;

	axxia_pcie = devm_kzalloc(&pdev->dev, sizeof(*axxia_pcie),
				  GFP_KERNEL);
	if (!axxia_pcie)
		return -ENOMEM;

	/* add this instance to the list */
	axxia_pcie->next = axxia_pcie_controllers;
	axxia_pcie_controllers = axxia_pcie;

	pp = &axxia_pcie->pp;
	pp->dev = &pdev->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pp->dbi_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pp->dbi_base))
		return PTR_ERR(pp->dbi_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "axi_gpreg");
	pp->axi_gpreg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pp->axi_gpreg_base))
		return PTR_ERR(pp->axi_gpreg_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cc_gpreg");
	pp->cc_gpreg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pp->cc_gpreg_base))
		return PTR_ERR(pp->cc_gpreg_base);

	pp->root_bus_nr = 0;
	pei_control = of_find_node_by_name(NULL, "pei_control");

	if (pei_control) {
		control = of_get_property(pei_control, "control", NULL);

		if (NULL == control) {
			pr_err("pcie-axxia: 'control' NOT set!!!\n");
			return -EINVAL;
		}

		control_value = be32_to_cpu(*control);

		initialized = of_get_property(pei_control, "initialized", NULL);

		/*
		  Previously, if the boot loader set 'control', it did
		  not initialized the PEI.  Start with that assumption.
		*/
		initialized_value = 1;
		if (NULL != initialized)
			initialized_value = be32_to_cpu(*initialized);

		/*
		 * always call pei_setup regardless of the 'initialized' value.
		 * this is needed to save the control_value.
		 * the axxia_pei setup will only be done once
		 */
		ret = axxia_pei_setup(control_value, 0);
		if (0 != ret) {
			pr_err("pcie-axxia: PEI setup failed!\n");
			return ret;
		}
	}

	ret = axxia_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int __exit axxia_pcie_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id axxia_pcie_of_match[] = {
	{ .compatible = "intel,axxia-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, axxia_pcie_of_match);

static struct platform_driver axxia_pcie_driver = {
	.probe          = axxia_pcie_probe,
	.remove		= axxia_pcie_remove,
	.driver = {
		.name	= "axxia-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = axxia_pcie_of_match,
	},
};

/*
  ------------------------------------------------------------------------------
  axxia_pcie_reset
*/

int
axxia_pcie_reset(void)
{
	struct pci_bus *pci_bus = NULL;
	int sub_buses;
	unsigned int control;
	struct pcie_port *_pp = NULL;
	struct axxia_pcie *axx_pcie = axxia_pcie_controllers;


	if (0 == axxia_pei_is_control_set())
		return -1;


	control = axxia_pei_get_control();

	/* Re-initialize the PEIs */
	axxia_pei_setup(control, 1);

	/* Re-configure the root complex */
	while (axx_pcie) {
		_pp = &axx_pcie->pp;
		axxia_pcie_setup_rc(_pp);
		axx_pcie = axx_pcie->next;
	}

	/* Re-scan the Bus */
	while (NULL != (pci_bus = pci_find_next_bus(pci_bus))) {
		pr_info("Rescanning PCI Bus %d\n", pci_bus->number);
		sub_buses = pci_rescan_bus(pci_bus);
		pr_info("%d subordinate busses...\n", sub_buses);
	}

	return 0;
}
EXPORT_SYMBOL(axxia_pcie_reset);

static ssize_t
axxia_pcie_reset_trigger(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	axxia_pcie_reset();

	return count;
}

/* Axxia PCIe driver does not allow module unload */

static int pcie_init(void)
{
	axxia_pcie_controllers = NULL;
	return platform_driver_register(&axxia_pcie_driver);
}
subsys_initcall(pcie_init);

static const struct file_operations axxia_pcie_reset_proc_ops = {
	.write      = axxia_pcie_reset_trigger,
	.llseek     = noop_llseek,
};

static int pcie2_init(void)
{
	struct proc_dir_entry *pf = proc_create("driver/axxia_pcie_reset",
						S_IWUSR, NULL,
						&axxia_pcie_reset_proc_ops);

	if (pf == NULL) {
		pr_err("Could not create /proc/driver/axxia_pcie_reset!\n");
		return -1;
	}

	return 0;
}

device_initcall(pcie2_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_DESCRIPTION("Axxia PCIe host controller driver");
MODULE_LICENSE("GPL v2");
