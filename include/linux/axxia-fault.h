/*
 * Copyright (C) 2017 Intel <john.jacques@intel.com>
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
 */

#ifndef __DRIVERS_MISC_AXXIA_FAULT_H
#define __DRIVERS_MISC_AXXIA_FAULT_H

asmlinkage int axxia_x9xlf_fault(struct pt_regs *, int, unsigned long);

int axxia_fault_get_mask(void);
void axxia_fault_set_mask(int);

#endif /* __DRIVERS_MISC_AXXIA_FAULT_H */
