/*
 * arch/arm/mach-axxia/perf_event_vp.c
 * included from arch/arm/mach-axxia/perf_event_platform.c
 *
 * Support for the INTEL Axxia boards based on ARM cores.
 *
 * Copyright (C) 2018 INTEL
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
 * Generic VP
 */

static void vp_startup_init(void)
{
}

static uint32_t vp_pmu_event_init(uint32_t event, struct perf_event *pevent)
{
	return 0;
}

static uint32_t vp_pmu_event_add(uint32_t event, struct perf_event *pevent)
{
	return 0;
}

static uint32_t vp_pmu_event_read(uint32_t event, struct perf_event *pevent,
		int flags)
{
	return 0;
}

static uint32_t vp_pmu_event_del(uint32_t event, struct perf_event *pevent,
		int flags)
{
	return 0;
}
