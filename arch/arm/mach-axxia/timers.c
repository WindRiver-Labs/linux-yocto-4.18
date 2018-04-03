/*
 *  arch/arm/mach-axxia/timers.c
 *
 *  Copyright (C) 2018 INTEL
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

/*
  This is based on arch/arm/common/timer-sp.c.

  The timers used are SP804s, but, there are 8 timers instead of 2,
  AND the ID registers are missing.
 */

#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sched_clock.h>
#include <linux/clockchips.h>
#include <../drivers/clocksource/timer-sp.h>
//#include <asm/hardware/arm_timer.h>

struct axxia_timer {
	struct clock_event_device  dev;
	struct irqaction           irqaction;
	void __iomem               *base;
	unsigned long              reload;
};

#define timer_to_clock_event(_x) container_of(_x, struct axxia_timer, dev)

static void __iomem *sched_clock_base;

static u64 sp804_read(void)
{
	return ~readl_relaxed(sched_clock_base + TIMER_VALUE);
}

/**
 * axxia_timer_set_mode
 */
static int axxia_set_next_event(unsigned long cycles,
				  struct clock_event_device *evt)
{
	struct axxia_timer *timer = timer_to_clock_event(evt);
	unsigned long ctrl;

	ctrl = readl(timer->base + TIMER_CTRL);
	writel(cycles, timer->base + TIMER_LOAD);
	writel(ctrl | TIMER_CTRL_ENABLE, timer->base + TIMER_CTRL);

	return 0;
}

static int axxia_shutdown(struct clock_event_device *evt)
{
	struct axxia_timer *timer = timer_to_clock_event(evt);
	unsigned long ctrl = TIMER_CTRL_32BIT | TIMER_CTRL_IE;

	pr_info("axxia_timer_shutdown: CPU#%d set on timer %s\n",
		smp_processor_id(), timer->dev.name);

	writel(ctrl, timer->base + TIMER_CTRL);

	return 0;

}

static int axxia_set_oneshot(struct clock_event_device *evt)
{
	struct axxia_timer *timer = timer_to_clock_event(evt);
	unsigned long ctrl = TIMER_CTRL_32BIT | TIMER_CTRL_IE;

	pr_info("axxia_timer_set_oneshot: CPU#%d set on timer %s\n",
		smp_processor_id(), timer->dev.name);

	writel(ctrl, timer->base + TIMER_CTRL);

	/* period set, and timer enabled in 'next_event' hook */
	ctrl |= TIMER_CTRL_ONESHOT;

	writel(ctrl, timer->base + TIMER_CTRL);

	return 0;
}

static int axxia_set_periodic(struct clock_event_device *evt)
{
	struct axxia_timer *timer = timer_to_clock_event(evt);
	unsigned long ctrl = TIMER_CTRL_32BIT | TIMER_CTRL_IE;

	pr_info("axxia_timer_set_periodic: CPU#%d set on timer %s\n",
		smp_processor_id(), timer->dev.name);

	writel(ctrl, timer->base + TIMER_CTRL);

	writel(timer->reload, timer->base + TIMER_LOAD);
	ctrl |= TIMER_CTRL_PERIODIC | TIMER_CTRL_ENABLE;

	writel(ctrl, timer->base + TIMER_CTRL);

	return 0;
}

/**
 * axxia_timer_handler - IRQ handler for the timer.
 *
 */
static irqreturn_t
axxia_timer_handler(int irq, void *dev_id)
{
	struct axxia_timer *timer = (struct axxia_timer *)dev_id;

	/* clear the interrupt */
	writel(1, timer->base + TIMER_INTCLR);

	timer->dev.event_handler(&timer->dev);

	return IRQ_HANDLED;
}

/**
 * axxia_timer_get_clock_rate
 *
 */
static long __init
axxia_timer_get_clock_rate(const char *name)
{
	struct clk *clk;
	long rate;
	int err;

	clk = clk_get_sys("sp804", name);
	if (IS_ERR(clk)) {
		pr_err("sp804: %s clock not found: %d\n", name,
			(int)PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	err = clk_prepare(clk);
	if (err) {
		pr_err("sp804: %s clock failed to prepare: %d\n", name, err);
		clk_put(clk);
		return err;
	}

	err = clk_enable(clk);
	if (err) {
		pr_err("sp804: %s clock failed to enable: %d\n", name, err);
		clk_unprepare(clk);
		clk_put(clk);
		return err;
	}

	rate = clk_get_rate(clk);
	if (rate < 0) {
		pr_err("sp804: %s clock failed to get rate: %ld\n", name, rate);
		clk_disable(clk);
		clk_unprepare(clk);
		clk_put(clk);
	}

	return rate;
}

void __init
axxia_timer_clocksource_init(void __iomem *base, const char *name)
{
	long rate;

	rate = axxia_timer_get_clock_rate(name);
	if (WARN_ON(rate < 0))
		return;

	/* Setup timer 0 as free-running clocksource */
	writel(0, base + TIMER_CTRL);
	writel(0xffffffff, base + TIMER_LOAD);
	writel(0xffffffff, base + TIMER_VALUE);
	writel(TIMER_CTRL_32BIT | TIMER_CTRL_ENABLE | TIMER_CTRL_PERIODIC,
		base + TIMER_CTRL);

	clocksource_mmio_init(base + TIMER_VALUE, name,
		rate, 200, 32, clocksource_mmio_readl_down);

	sched_clock_base = base;
	sched_clock_register(sp804_read, 32, rate);
}

void __init
axxia_timer_clockevents_init(void __iomem *base,
			     unsigned int irq, const char *name)
{
	struct axxia_timer *evt;
	long               rate;

	rate = axxia_timer_get_clock_rate(name);
	if (WARN_ON(rate < 0))
		return;

	evt = kzalloc(sizeof(*evt), GFP_KERNEL);
	if (evt == NULL)
		return;

	evt->dev.features       = CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT,
	evt->dev.set_next_event		= axxia_set_next_event,
	evt->dev.set_state_shutdown	= axxia_shutdown,
	evt->dev.set_state_periodic	= axxia_set_periodic,
	evt->dev.set_state_oneshot	= axxia_set_oneshot,
	evt->dev.rating		= 400,
	evt->dev.name           = name;
	evt->dev.irq            = irq;
	evt->dev.cpumask	= cpu_all_mask,
	evt->base               = base;
	evt->reload             = DIV_ROUND_CLOSEST(rate, HZ);

	evt->irqaction.name     = name;
	evt->irqaction.flags    = IRQF_TIMER | IRQF_IRQPOLL;
	evt->irqaction.handler	= axxia_timer_handler;
	evt->irqaction.dev_id	= evt;

	setup_irq(irq, &evt->irqaction);
	clockevents_config_and_register(&evt->dev, rate, 0xf, 0xffffffff);
}
