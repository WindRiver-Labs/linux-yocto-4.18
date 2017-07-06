/*
 * Copyright (C) 2016 Intel
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

#ifndef __AXXIA_PEI_H
#define __AXXIA_PEI_H

int axxia_pei_reset(unsigned int);
int axxia_pei_setup(unsigned int, unsigned int);
int axxia_pcie_reset(void);

unsigned int axxia_pei_get_control(void);
int axxia_pei_is_control_set(void);


#endif	/* __AXXIA_PEI_H */
