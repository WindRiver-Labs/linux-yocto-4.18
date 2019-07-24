// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

#include <tal/tal.h>
#include "drv_dxt_if.h"

static int drv_dxt_spi_cs;
static int drv_dxt_irq;
static void *drv_dxt_irq_dev;
static irq_handler_t drv_dxt_irq_handler;

void drv_dxt_if_signal_interrupt(void)
{
	if (drv_dxt_irq_handler == NULL)
		return;

	drv_dxt_irq_handler(drv_dxt_irq, drv_dxt_irq_dev);
}

void drv_dxt_if_enable_irq(u32 irq)
{
	/* We have only one TDM channel */
	tal_intr_enable();
}
EXPORT_SYMBOL(drv_dxt_if_enable_irq);

void drv_dxt_if_disable_irq(u32 irq)
{
	/* We have only one TDM channel */
	tal_intr_disable();
}
EXPORT_SYMBOL(drv_dxt_if_disable_irq);

int drv_dxt_if_request_irq(u32 irq, irq_handler_t handler, u64 flags,
			   const char *name, void *dev)
{
	drv_dxt_irq = irq;
	drv_dxt_irq_dev = dev;
	drv_dxt_irq_handler = handler;

	return 0;
}
EXPORT_SYMBOL(drv_dxt_if_request_irq);

void drv_dxt_if_free_irq(u32 irq, void *dev)
{
	drv_dxt_irq_handler = NULL;
}
EXPORT_SYMBOL(drv_dxt_if_free_irq);

void drv_dxt_if_spi_cs_set(u32 dev_no, u32 hi_lo)
{
	if (hi_lo == 0)
		drv_dxt_spi_cs = dev_no;
	else
		drv_dxt_spi_cs = -1;
}
EXPORT_SYMBOL(drv_dxt_if_spi_cs_set);

int drv_dxt_if_spi_ll_read_write(u8 *tx_data, u32 tx_size,
				 u8 *rx_data, u32 rx_size)
{
	uint16_t *ptr;
	int i;

	if ((tx_size & 1) || (rx_size & 1)) {
		pr_err("drv_dxt_if: SPI transfer is not word aligned!\n");
		return 0;
	}

	ptr = (uint16_t *)tx_data;
	for (i = 0; i < tx_size / 2; i++, ptr++)
		*ptr = htons(*ptr);

	if (rx_data != NULL && rx_size != 0) {
		mv_phone_spi_read(drv_dxt_spi_cs, tx_data, tx_size,
				  rx_data, rx_size, SPI_TYPE_SLIC_LANTIQ);
	} else if (tx_data != NULL && tx_size > 2) {
		mv_phone_spi_write(drv_dxt_spi_cs, tx_data, 2, tx_data + 2,
				   tx_size - 2, SPI_TYPE_SLIC_LANTIQ);
	} else {
		pr_err("drv_dxt_if: Unsupported SPI access mode!\n");
	}

	ptr = (uint16_t *)rx_data;
	for (i = 0; i < rx_size / 2; i++, ptr++)
		*ptr = htons(*ptr);

	return 0;
}
EXPORT_SYMBOL(drv_dxt_if_spi_ll_read_write);

