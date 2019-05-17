// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

#ifndef _DRV_DXT_IF_H
#define _DRV_DXT_IF_H

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "mv_phone.h"

/* Interface to drv_dxt driver */
extern void drv_dxt_if_enable_irq(u32 irq);
extern void drv_dxt_if_disable_irq(u32 irq);
extern int drv_dxt_if_request_irq(u32 irq, irq_handler_t handler, u64 flags,
				  const char *name, void *dev);
extern void drv_dxt_if_free_irq(u32 irq, void *dev);
extern void drv_dxt_if_spi_cs_set(u32 dev_no, u32 hi_lo);
extern int drv_dxt_if_spi_ll_read_write(u8 *tx_data, u32 tx_size,
					u8 *rx_data, u32 rx_size);
extern void drv_dxt_if_signal_interrupt(void);

#endif /* _DRV_DXT_IF_H */
