// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

#include <tal/tal.h>
#include "zarlink_if.h"

static void (*zsi_handler)(unsigned long);

void zarlink_if_enable_irq(u32 device)
{
	tal_intr_enable();
}
EXPORT_SYMBOL(zarlink_if_enable_irq);

void zarlink_if_disable_irq(u32 device)
{
	tal_intr_disable();
}
EXPORT_SYMBOL(zarlink_if_disable_irq);

void zarlink_if_handler_register(void *func)
{
	zsi_handler = func;
}
EXPORT_SYMBOL(zarlink_if_handler_register);

void zarlink_if_handler_unregister(void)
{
	zsi_handler = NULL;
}
EXPORT_SYMBOL(zarlink_if_handler_unregister);

void zarlink_if_zsi_interrupt(void)
{
	if (zsi_handler != NULL)
		zsi_handler(0);
}
EXPORT_SYMBOL(zarlink_if_zsi_interrupt);

void zarlink_if_spi_read(u32 line, u8 *cmd, u8 cmd_size,
			 u8 *data, u8 data_size)
{
	uint32_t slic_device = mv_phone_get_slic_board_type();

	if (slic_device == SLIC_ZARLINK_ID)
		mv_phone_spi_read(line, cmd, cmd_size, data,
				  data_size, SPI_TYPE_SLIC_ZSI);
	else
		mv_phone_spi_read(line, cmd, cmd_size, data,
				  data_size, SPI_TYPE_SLIC_ZARLINK_SILABS);
}
EXPORT_SYMBOL(zarlink_if_spi_read);

void zarlink_if_spi_write(u32 line, u8 *cmd, u8 cmd_size,
			  u8 *data, u8 data_size)
{
	uint32_t slic_device = mv_phone_get_slic_board_type();

	if (slic_device == SLIC_ZARLINK_ID)
		mv_phone_spi_write(line, cmd, cmd_size, data,
				   data_size, SPI_TYPE_SLIC_ZSI);
	else
		mv_phone_spi_write(line, cmd, cmd_size, data,
				   data_size, SPI_TYPE_SLIC_ZARLINK_SILABS);
}
EXPORT_SYMBOL(zarlink_if_spi_write);
