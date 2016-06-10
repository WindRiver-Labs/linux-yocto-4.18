/*
 * drivers/lsi/common/version.h
 *
 * Copyright (C) 2010 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#ifndef __DRIVERS_LSI_ACP_NCR_H
#define __DRIVERS_LSI_ACP_NCR_H

#ifndef NCP_REGION_ID
#define NCP_REGION_ID(node, target) \
((unsigned long) ((((node) & 0xffff) << 16) | ((target) & 0xffff)))
#endif

#ifndef NCP_NODE_ID
#define NCP_NODE_ID(region) (((region) >> 16) & 0xffff)
#endif

#ifndef NCP_TARGET_ID
#define NCP_TARGET_ID(region) ((region) & 0xffff)
#endif

unsigned int ncr_register_read(unsigned int *);
void ncr_register_write(const unsigned int, unsigned int *);
int ncr_read(unsigned int, unsigned int, int, void *);
int ncr_read32(unsigned int, unsigned int, unsigned int *);
int ncr_write(unsigned int, unsigned int, int, void *);
int ncr_write32(unsigned int, unsigned int, unsigned int);
int ncr_read_nolock(unsigned int, unsigned int, int, void *);
int ncr_write_nolock(unsigned int, unsigned int, int, void *);

 /*
  * when defined, the RTE driver module will set/clear
  * the ncr_reset_active flag to indicate when Axxia device
  * reset is in progress. This flag will be checked by the
  * kernel lsi-ncr driver and ddr_retention code.
  */
#ifdef CONFIG_ARCH_AXXIA_NCR_RESET_CHECK
extern int ncr_reset_active;
#endif

#endif /*  __DRIVERS_LSI_ACP_NCR_H */
