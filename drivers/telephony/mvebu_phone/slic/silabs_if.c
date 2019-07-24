// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

#include <tal/tal.h>
#include "silabs_if.h"

static void (*isi_handler)(unsigned long);

void silabs_if_enable_irq(u32 device)
{
	tal_intr_enable();
}
EXPORT_SYMBOL(silabs_if_enable_irq);

void silabs_if_disable_irq(u32 device)
{
	tal_intr_disable();
}
EXPORT_SYMBOL(silabs_if_disable_irq);

void silabs_if_handler_register(void *func)
{
	isi_handler = func;
}
EXPORT_SYMBOL(silabs_if_handler_register);

void silabs_if_handler_unregister(void)
{
	isi_handler = NULL;
}
EXPORT_SYMBOL(silabs_if_handler_unregister);

void silabs_if_isi_interrupt(void)
{
	if (isi_handler != NULL)
		isi_handler(0);
}
EXPORT_SYMBOL(silabs_if_isi_interrupt);

void silabs_if_spi_init(u32 line)
{
}
EXPORT_SYMBOL(silabs_if_spi_init);

void silabs_if_spi_read(u32 line, u8 *cmd, u8 cmd_size,
			u8 *data, u8 data_size)
{
	uint32_t slic_device = mv_phone_get_slic_board_type();

	if (slic_device == SLIC_SILABS_ID)
		mv_phone_spi_read(line, cmd, cmd_size, data,
				  data_size, SPI_TYPE_SLIC_ISI);
	else
		mv_phone_spi_read(line, cmd, cmd_size, data,
				  data_size, SPI_TYPE_SLIC_ZARLINK_SILABS);
}
EXPORT_SYMBOL(silabs_if_spi_read);

void silabs_if_spi_write(u32 line, u8 *cmd, u8 cmd_size,
			 u8 *data, u8 data_size)
{
	uint32_t slic_device = mv_phone_get_slic_board_type();

	if (slic_device == SLIC_SILABS_ID)
		mv_phone_spi_write(line, cmd, cmd_size, data,
				   data_size, SPI_TYPE_SLIC_ISI);
	else
		mv_phone_spi_write(line, cmd, cmd_size, data,
				   data_size, SPI_TYPE_SLIC_ZARLINK_SILABS);
}
EXPORT_SYMBOL(silabs_if_spi_write);
