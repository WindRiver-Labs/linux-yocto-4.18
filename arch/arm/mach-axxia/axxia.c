/*
 * arch/arm/mach-axxia/axxia.c
 *
 * Support for the LSI Axxia boards based on ARM cores.
 *
 * Copyright (C) 2012 LSI
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
 */

#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/amba/pl022.h>
#include <linux/amba/pl061.h>
#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/smsc911x.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#ifdef CONFIG_ARM_ARCH_TIMER
#include <asm/arch_timer.h>
#endif
#include <asm/sizes.h>
#include <asm/pmu.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/kexec.h>
#include <asm/mach/time.h>
#include <asm/hardware/cache-l2x0.h>
#include <mach/hardware.h>
#include <mach/timers.h>
#include <mach/axxia-gic.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/lsi-ncr.h>
#include "axxia.h"
#include "pci.h"
#ifdef CONFIG_AXXIA_RIO
#include <mach/rio.h>
#endif

static const char *const axxia_dt_match[] __initconst = {
	"lsi,axm5500",
	NULL
};

static void __iomem *base;

#ifdef CONFIG_KEXEC

//static void __iomem *dickens;

static void set_l3_pstate(u32 newstate)
{
	static const u8 hnf[] = {
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
	};
	int i;
	u32 status;

	for (i = 0; i < ARRAY_SIZE(hnf); ++i)
		writel(newstate, dickens + (hnf[i] << 16) + 0x10);

	for (i = 0; i < ARRAY_SIZE(hnf); ++i) {
		int retry;

		for (retry = 10000; retry > 0; --retry) {
			status = readl(dickens + (hnf[i] << 16) + 0x18);
			if (((status >> 2) & 3) == newstate)
				break;
			udelay(1);
		}
		BUG_ON(retry == 0);
	}
}

static void
flush_l3(void)
{
	/* Switch to SFONLY to flush */
	set_l3_pstate(1);
	/* ...and then back up again */
	set_l3_pstate(3);
}

#endif

static struct map_desc axxia_static_mappings[] __initdata = {
#ifdef CONFIG_DEBUG_LL
	{
		.virtual	=  AXXIA_DEBUG_UART_VIRT,
		.pfn		= __phys_to_pfn(AXXIA_DEBUG_UART_PHYS),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
#endif
};

void __init axxia_dt_map_io(void)
{
	iotable_init(axxia_static_mappings, ARRAY_SIZE(axxia_static_mappings));
}

void __init axxia_dt_init_early(void)
{
}

static struct of_device_id axxia_irq_match[] __initdata = {
	{
		.compatible = "arm,cortex-a15-gic",
		.data = axxia_gic_of_init,
	},
	{ }
};

static void __init axxia_dt_init_irq(void)
{
	of_irq_init(axxia_irq_match);
}

void __init axxia_dt_timer_init(void)
{
	int is_sim;

	is_sim = of_find_compatible_node(NULL, NULL, "lsi,axm5500-sim") != NULL;

	axxia_init_clocks(is_sim);

	of_clk_init(NULL);
	clocksource_of_init();
}

static struct mmci_platform_data mmc_plat_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.status	  = NULL,
	.gpio_wp  = -ENOSYS,
	.gpio_cd  = -ENOSYS
};

static struct of_dev_auxdata axxia_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("arm,primecell", 0x20101E0000ULL,
		       "mmci",  &mmc_plat_data),
	{}
};

static struct resource axxia_pmu_resources[] = {
	[0] = {
		.start  = IRQ_PMU,
		.end    = IRQ_PMU,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device pmu_device = {
	.name			= "arm-pmu",
	.id                     = -1,
	.num_resources		= ARRAY_SIZE(axxia_pmu_resources),
	.resource		= axxia_pmu_resources,
};

static int
axxia_bus_notifier(struct notifier_block *nb, unsigned long event, void *obj)
{
	struct device *dev = obj;

	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	if (!of_property_read_bool(dev->of_node, "dma-coherent"))
		return NOTIFY_DONE;

	set_dma_ops(dev, &arm_coherent_dma_ops);

	return NOTIFY_OK;
}

static struct notifier_block axxia_platform_nb = {
	.notifier_call = axxia_bus_notifier,
};

static struct notifier_block axxia_amba_nb = {
	.notifier_call = axxia_bus_notifier,
};

void __init axxia_dt_init(void)
{
	base = ioremap(0x2010000000, 0x40000);
#ifdef CONFIG_KEXEC
	if (!of_find_compatible_node(NULL, NULL, "lsi,axm5500-sim")) {
		dickens = ioremap(0x2000000000, SZ_4M);
		kexec_reinit = flush_l3;
	}
#endif

	bus_register_notifier(&platform_bus_type, &axxia_platform_nb);
	bus_register_notifier(&amba_bustype, &axxia_amba_nb);

	of_platform_populate(NULL, of_default_bus_match_table,
			     axxia_auxdata_lookup, NULL);
	pm_power_off = NULL; /* TBD */

	axxia_ddr_retention_init();

#ifdef CONFIG_AXXIA_RIO
	axxia_rapidio_init();
#endif

	platform_device_register(&pmu_device);
}

static void axxia_restart(enum reboot_mode reboot, const char *cmd)
{
	writel(0x000000ab, base + 0x31000); /* Access Key */
	writel(0x00000040, base + 0x31004); /* Intrnl Boot, 0xffff0000 Target */
	writel(0x80000000, base + 0x3180c); /* Set ResetReadDone */
	writel(0x00080802, base + 0x31008); /* Chip Reset */
}

DT_MACHINE_START(AXXIA_DT, "LSI Axxia")
	.dt_compat	= axxia_dt_match,
	.smp		= smp_ops(axxia_smp_ops),
	.map_io		= axxia_dt_map_io,
	.init_early	= axxia_dt_init_early,
	.init_irq	= axxia_dt_init_irq,
	.init_time	= axxia_dt_timer_init,
	.init_machine	= axxia_dt_init,
	.restart	= axxia_restart,
#if defined(CONFIG_ZONE_DMA) && defined(CONFIG_ARM_LPAE)
	.dma_zone_size	= (4ULL * SZ_1G) - 1,
#endif
MACHINE_END
