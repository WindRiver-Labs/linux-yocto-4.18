/*
 *  arch/arm/mach-axxia/include/mach/timers.h
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
  This is based on arch/arm/include/asm/hardware/timer-sp.h

  See arch/arm/mach-axxia/timers.c for details.
 */

//#include "timer-sp.h"

#define AXXIA_TIMER_1_BASE 0x00
#define AXXIA_TIMER_2_BASE 0x20
#define AXXIA_TIMER_3_BASE 0x40
#define AXXIA_TIMER_4_BASE 0x60
#define AXXIA_TIMER_5_BASE 0x80
#define AXXIA_TIMER_6_BASE 0xa0
#define AXXIA_TIMER_7_BASE 0xc0
#define AXXIA_TIMER_8_BASE 0xe0

void axxia_timer_clocksource_init(void __iomem *, const char *);
void axxia_timer_clockevents_init(void __iomem *, unsigned int, const char *);
