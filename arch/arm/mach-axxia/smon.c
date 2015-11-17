/*
 * linux/arch/arm/mach-axxia/smon.c
 *
 * Platform perf helper module for generic VP statistical monitor
 *
 * Copyright (C) 2013 LSI Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#include <linux/io.h>

#include <linux/lsi-ncr.h>

#include "smon.h"

static void memcpy32_fromio(uint32_t *dest, uint32_t *src, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
		dest[i] = ioread32(src + i);
}

static void memcpy32_toio(uint32_t *dest, uint32_t *src, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
		iowrite32(src[i], dest + i);
}

void smon_init_ncp(struct smon_s *smon, uint32_t node, uint32_t target,
	uint32_t offset)
{
	smon->assigned[0] = UNASSIGNED;
	smon->assigned[1] = UNASSIGNED;
	smon->type = NCP_SMON;
	smon->node = node;
	smon->target = target;
	smon->offset = offset;
}

void smon_init_mem(struct smon_s *smon, uint64_t addr, uint32_t offset)
{
	smon->assigned[0] = UNASSIGNED;
	smon->assigned[1] = UNASSIGNED;
	smon->type = MEM_SMON;
	smon->addr = ioremap(addr, SZ_4K);
	smon->offset = offset;

	if (smon->addr == NULL)
		pr_err("axxia perf, smon can't remap memory %lld\n", addr);
}

void smon_stop_if_unassigned(struct smon_s *smon)
{
	uint32_t control = 0;

	if (smon->assigned[0] == UNASSIGNED &&
	    smon->assigned[1] == UNASSIGNED) {
		ncr_write(NCP_REGION_ID(smon->node, smon->target), smon->offset,
			  1 * REG_SZ, &control);
	}
}

uint32_t smon_allocate(struct smon_s *smon, uint8_t event)
{
	if (smon->assigned[0] == UNASSIGNED) {
		smon->events[0] = event;
		smon->assigned[0] = ASSIGNED;
	} else if (smon->assigned[1] == UNASSIGNED) {
		smon->events[1] = event;
		smon->assigned[1] = ASSIGNED;
	} else {
		pr_warn("smon_allocate, no counter availible\n");
		return -ENOCOUNTER;
	}

	return 0;
}

uint32_t smon_deallocate(struct smon_s *smon, uint8_t event)
{
	if ((smon->assigned[0] == ASSIGNED) && (smon->events[0] == event))
		smon->assigned[0] = UNASSIGNED;
	else if ((smon->assigned[1] == ASSIGNED) && (smon->events[1] == event))
		smon->assigned[1] = UNASSIGNED;
	else
		return -ENOCOUNTER;

	return 0;
}

uint32_t smon_event_active(struct smon_s *smon, uint8_t event)
{
	if ((smon->assigned[0] == ASSIGNED) && (smon->events[0] == event))
		return 0;
	else if ((smon->assigned[1] == ASSIGNED) && (smon->events[1] == event))
		return 0;

	return -ENOCOUNTER;
}

uint32_t smon_read(struct smon_s *smon, uint8_t event)
{
	uint32_t deltacount;

	if (smon->type == NCP_SMON)
		ncr_read(NCP_REGION_ID(smon->node, smon->target), smon->offset,
			 8 * REG_SZ, &smon->regs);
	else if (smon->type == MEM_SMON)
		memcpy32_fromio((uint32_t *)&smon->regs,
			(uint32_t *)smon->addr + smon->offset, 8);

	if ((smon->assigned[0] == ASSIGNED) &&
			(smon->events[0] == event)) {
		if (smon->regs.count0 >= smon->lastread[0])
			deltacount = smon->regs.count0 - smon->lastread[0];
		else
			deltacount = 0xffffffff - smon->lastread[0]
					+ smon->regs.count0;

		smon->lastread[0] = smon->regs.count0;

		return deltacount;
	} else if ((smon->assigned[1] == ASSIGNED) &&
			(smon->events[1] == event)) {
		if (smon->regs.count1 >= smon->lastread[1])
			deltacount = smon->regs.count1 - smon->lastread[1];
		else
			deltacount = 0xffffffff - smon->lastread[1]
					+ smon->regs.count1;

		smon->lastread[1] = smon->regs.count1;

		return deltacount;
	}

	return -ENOEVENT;
}

uint32_t smon_start(struct smon_s *smon, uint8_t event)
{
	/* get currect configuration */
	if (smon->type == NCP_SMON)
		ncr_read(NCP_REGION_ID(smon->node, smon->target), smon->offset,
			 8 * REG_SZ, &smon->regs);
	else if (smon->type == MEM_SMON)
		memcpy32_fromio((uint32_t *)&smon->regs,
			(uint32_t *)smon->addr + smon->offset, 8);

	smon->regs.control = 1;	/* run counters */

	if ((smon->assigned[0] == ASSIGNED) && (smon->events[0] == event)) {
		smon->regs.event0 = event;
		smon->regs.count0 = 0;
		smon->lastread[0] = 0;

		if (smon->type == NCP_SMON) {
			/* write configuration, but do not change count reg */
			ncr_write(NCP_REGION_ID(smon->node, smon->target),
				smon->offset, 2 * REG_SZ, &smon->regs);

			/* clear this events counter register */
			ncr_write(NCP_REGION_ID(smon->node, smon->target),
				smon->offset + 4 * REG_SZ, 1 * REG_SZ,
				&smon->regs.count0);
		} else if (smon->type == MEM_SMON) {
			/* write configuration, but do not change count reg */
			memcpy32_toio((uint32_t *)smon->addr + smon->offset,
				(uint32_t *)&smon->regs, 2);

			/* clear this events counter register */
			memcpy32_toio((uint32_t *)smon->addr + smon->offset + 4,
				(uint32_t *)&smon->regs.count0, 1);

		}

	} else if ((smon->assigned[1] == ASSIGNED)
		&& (smon->events[1] == event)) {
		smon->regs.event1 = event;
		smon->regs.count1 = 0;
		smon->lastread[1] = 0;

		if (smon->type == NCP_SMON) {
			/* write configuration, but do not change count reg */
			ncr_write(NCP_REGION_ID(smon->node, smon->target),
				smon->offset, 2 * REG_SZ, &smon->regs);

			/* clear this events counter register */
			ncr_write(NCP_REGION_ID(smon->node, smon->target),
				  smon->offset + 5 * REG_SZ, 1 * REG_SZ,
				  &smon->regs.count1);
		} else if (smon->type == MEM_SMON) {
			/* write configuration, but do not change count reg */
			memcpy32_toio((uint32_t *)smon->addr + smon->offset,
				(uint32_t *)&smon->regs, 2);

			/* clear this events counter register */
			memcpy32_toio((uint32_t *)smon->addr + smon->offset + 5,
				(uint32_t *)&smon->regs.count1, 1);
		}

	} else {
		pr_warn("smon_start, no counter availible\n");
		return -ENOCOUNTER;
	}

	return 0;
}
