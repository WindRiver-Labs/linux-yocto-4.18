/*
 *  linux/arch/arm/mach-axxia/clock.c
 *
 *  Copyright (C) 2012 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>


#define clk_register_clkdev(_clk, _conid, _devfmt, ...) \
	do { \
		struct clk_lookup *cl; \
		cl = clkdev_alloc(_clk, _conid, _devfmt, ## __VA_ARGS__); \
		clkdev_add(cl); \
	} while (0)

enum clk_ids {
	clk_cpu,
	clk_per,
	clk_mmc,
	clk_apb,
	clk_1mhz,
	NR_CLK_IDS
};

static struct dt_clk_lookup {
	const char  *path;
	const char  *name;
	enum clk_ids id;
	u32          default_freq;
} dt_clks[] = {
	{"/clocks/cpu",        "clk_cpu", clk_cpu, 1400000000 },
	{"/clocks/peripheral", "clk_per", clk_per,  200000000 },
	{"/clocks/emmc",       "clk_mmc", clk_mmc,  200000000 },
};

static struct clk *clk[NR_CLK_IDS];

static void axxia_register_clks(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dt_clks); ++i) {
		struct dt_clk_lookup *c = &dt_clks[i];
		struct device_node *np = of_find_node_by_path(c->path);
		u32 freq;

		if (!np || of_property_read_u32(np, "frequency", &freq)) {
			pr_warn("axxia: No 'frequency' in %s\n", c->path);
			freq = c->default_freq;
		}
		clk[c->id] = clk_register_fixed_rate(NULL, c->name, NULL,
						     CLK_IS_ROOT, freq);
	}

	/* APB clock dummy */
	clk[clk_apb] = clk_register_fixed_rate(NULL, "apb_pclk", NULL,
					       CLK_IS_ROOT, 1000000);

	clk[clk_1mhz] = clk_register_fixed_rate(NULL, "clk_1mhz", NULL,
						CLK_IS_ROOT, 1000000);
}

void __init
axxia_init_clocks(int is_sim)
{
	int i;

	pr_info("axxia: init_clocks: is_sim=%d\n", is_sim);

	axxia_register_clks();

	/* PL011 UARTs */
	clk_register_clkdev(clk[clk_per], NULL, "2010080000.uart");
	clk_register_clkdev(clk[clk_per], NULL, "2010081000.uart");
	clk_register_clkdev(clk[clk_per], NULL, "2010082000.uart");
	clk_register_clkdev(clk[clk_per], NULL, "2010083000.uart");

	/* PL022 SSP */
	clk_register_clkdev(clk[clk_per], NULL, "2010088000.ssp");

	/* I2C */
	clk_register_clkdev(clk[clk_per], NULL, "2010084000.i2c");
	clk_register_clkdev(clk[clk_per], NULL, "2010085000.i2c");
	clk_register_clkdev(clk[clk_per], NULL, "2010086000.i2c");
	clk_register_clkdev(clk[clk_per], NULL, "2010087000.i2c");

	/* SP804 timers */
	clk_register_clkdev(clk[is_sim ? clk_1mhz : clk_per], NULL, "sp804");
	for (i = 0; i < 8; i++)
		clk_register_clkdev(clk[is_sim ? clk_1mhz : clk_per],
				    NULL, "axxia-timer%d", i);

	/* PL180 MMCI */
	clk_register_clkdev(clk[clk_mmc], NULL, "mmci");

	clk_register_clkdev(clk[clk_apb], "apb_pclk", NULL);
}
