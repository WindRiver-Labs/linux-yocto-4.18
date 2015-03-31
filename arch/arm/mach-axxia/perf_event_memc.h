/*
 * arch/arm/mach-axxia/perf_event_memc.h
 * included from arch/arm/mach-axxia/perf_event_memc.c
 *
 * Support for the LSI Axxia boards based on ARM cores.
 *
 * Copyright (C) 2014 LSI
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

#ifndef __ASM__ARCH_AXXIA_PERF_EVENT_MEMC_H
#define __ASM__ARCH_AXXIA_PERF_EVENT_MEMC_H

#define DDRC0_OFFSET 0x00
#define DDRC0_SMON_MAX (DDRC0_OFFSET + 22)
#define DDRC1_OFFSET 0x100
#define DDRC1_SMON_MAX (DDRC1_OFFSET + 22)

#define ELM0_OFFSET 0x200
#define ELM0_SMON_MAX (ELM0_OFFSET + 15)
#define ELM1_OFFSET 0x300
#define ELM1_SMON_MAX (ELM1_OFFSET + 15)

/* Node */
#define DDRC0 0x0f
#define DDRC1 0x22
/* Target */
#define DDRC_CTRL 0x00
#define DDRC_PERF 0x02
/* Address */
#define CTRL_SMON 0x1fc

#ifdef AXM55XX_R1
#define DDRC_SMON 0x40
#endif
#ifdef AXM55XX_R2
#define DDRC_SMON 0xA0
#endif

/* Settings */
#define SMON_ENABLE 0x20000000

/* Base Address */
#define ELM0 0x2010060000
#define ELM1 0x2010070000
/* SMON Offset */
#define ELM_SMON (0x300/4)

struct smon_s ddrc0_smon;
struct smon_s ddrc1_smon;
struct smon_s elm0_smon;
struct smon_s elm1_smon;

#endif
