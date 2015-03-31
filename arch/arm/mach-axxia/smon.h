/*
 * Helper module for board specific I2C bus registration
 *
 * Copyright (C) 2014 LSI Corporation.
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
#ifndef __ASM__ARCH_AXXIA_SMON_H
#define __ASM__ARCH_AXXIA_SMON_H

#include <linux/kernel.h>

struct smon_reg_s {
	uint32_t control;
	uint8_t event0;
	uint8_t event1;
	uint16_t reserved;
	uint32_t compare0;
	uint32_t compare1;
	uint32_t count0;
	uint32_t count1;
	uint32_t time;
	uint32_t maxtime;
};

struct smon_s {
	struct smon_reg_s regs;
	uint32_t type; /* NCP_SMON or MEM_SMON */
	uint32_t *addr; /* MEM_SMON */
	uint32_t node; /* NCP_SMON */
	uint32_t target; /* " */
	uint32_t offset;
	uint32_t lastread[2];
	uint8_t assigned[2];
	uint8_t events[2];
};

#define REG_SZ 4

#define MEM_SMON 0
#define NCP_SMON 1

#define UNASSIGNED 0
#define ASSIGNED 1

#define ENOCOUNTER 1
#define ENOEVENT 2

void smon_init_ncp(struct smon_s *smon, uint32_t node, uint32_t target,
	uint32_t offset);
void smon_init_mem(struct smon_s *smon, uint64_t addr, uint32_t offset);
void smon_stop_if_unassigned(struct smon_s *smon);
uint32_t smon_allocate(struct smon_s *smon, uint8_t event);
uint32_t smon_deallocate(struct smon_s *smon, uint8_t event);
uint32_t smon_event_active(struct smon_s *smon, uint8_t event);
uint32_t smon_read(struct smon_s *smon, uint8_t event);
uint32_t smon_start(struct smon_s *smon, uint8_t event);

#endif /* __ASM__ARCH_AXXIA_SMON_H */
