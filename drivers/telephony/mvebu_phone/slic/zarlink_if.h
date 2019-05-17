// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

#ifndef _ZARLINK_IF_H_
#define _ZARLINK_IF_H_

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "mv_phone.h"

/* Interface to Zarlink SLIC driver */
extern void zarlink_if_enable_irq(u32 device);
extern void zarlink_if_disable_irq(u32 device);
extern void zarlink_if_handler_register(void *func);
extern void zarlink_if_handler_unregister(void);
extern void zarlink_if_zsi_interrupt(void);
extern void zarlink_if_spi_read(u32 line, u8 *cmd, u8 cmd_size,
				u8 *data, u8 data_size);
extern void zarlink_if_spi_write(u32 line, u8 *cmd, u8 cmd_size,
				 u8 *data, u8 data_size);

#endif /* _ZARLINK_IF_H_ */

