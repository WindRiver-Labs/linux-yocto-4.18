// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

#include <linux/spi/spi.h>
#include "mv_phone.h"

#define DRV_NAME "mvebu_phone_spi"

#define MAX_SLIC_DEVICES 32

/* Global array of pointers to SLIC SPI devices */
struct spi_device *slic_devs[MAX_SLIC_DEVICES];

/* Telephony register read via SPI interface. */
void mv_phone_spi_read(u16 dev_id, u8 *cmd_buff, u8 cmd_size,
		       u8 *data_buff, u8 data_size, u32 spi_type)
{
	struct spi_device *slic_spi = slic_devs[dev_id];
	int err;

#ifdef CONFIG_MV_PHONE_SPI_DEBUG
	pr_info("%s():line(%d) Spi ID=%d dev_id=%d Spi CS=%d Spi type=%d\n",
		__func__, __LINE__, slic_spi->master->bus_num, dev_id,
		slic_spi->chip_select, spi_type);
#endif

	err = spi_write_then_read(slic_spi, (const void *)cmd_buff, cmd_size,
				  (void *)data_buff, data_size);
	if (err)
		dev_err(&slic_spi->dev, "SPI read failed\n");

#ifdef CONFIG_MV_PHONE_SPI_DEBUG
	pr_info("CMD = 0x%x, cmd_size = 0x%x, DATA = 0x%x, data_size = 0x%x\n",
		*cmd_buff, cmd_size, *data_buff, data_size);
#endif
}

/* Telephony register write via SPI interface. */
void mv_phone_spi_write(u16 dev_id, u8 *cmd_buff, u8 cmd_size,
			u8 *data_buff, u8 data_size, u32 spi_type)
{
	int err;
	struct spi_message m;
	struct spi_device *slic_spi = slic_devs[dev_id];
	struct spi_transfer t[2] = { { .tx_buf = (const void *)cmd_buff,
				       .len = cmd_size, },
				     { .tx_buf = (const void *)data_buff,
				       .len = data_size, }, };

#ifdef CONFIG_MV_PHONE_SPI_DEBUG
	pr_info("%s():line(%d) Spi ID=%d dev_id=%d Spi CS=%d Spi type=%d\n",
		__func__, __LINE__, slic_spi->master->bus_num, dev_id,
		slic_spi->chip_select, spi_type);
	pr_info("CMD = 0x%x, cmd_size = 0x%x, DATA = 0x%x, data_size = 0x%x\n",
		*cmd_buff, cmd_size, *data_buff, data_size);
#endif

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	err = spi_sync(slic_spi, &m);
	if (err)
		dev_err(&slic_spi->dev, "SPI write failed\n");
}

static int mvebu_phone_spi_probe(struct spi_device *spi)
{
	int err;
	u32 dev_id;

	/* Obtain SLIC ID */
	err = of_property_read_u32(spi->dev.of_node, "slic-id", &dev_id);
	if (err == -EINVAL) {
		/* Assign '0' ID in case the 'slic-id' property is not used */
		dev_id = 0;
	} else if (err) {
		dev_err(&spi->dev, "unable to get SLIC ID\n");
		return err;
	} else if (dev_id >= MAX_SLIC_DEVICES) {
		dev_err(&spi->dev, "SLIC ID (%d) exceeds maximum (%d)\n",
			dev_id, MAX_SLIC_DEVICES - 1);
		return -EINVAL;
	}

	/* Check if this ID wasn't used by previous devices */
	if (slic_devs[dev_id]) {
		dev_err(&spi->dev, "overlapping ID (%d) at bus #%d, CS #%d\n",
			dev_id, spi->master->bus_num, spi->chip_select);
		return -EINVAL;
	}

	slic_devs[dev_id] = spi;

	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "spi setup failed\n");
		return err;
	}

	dev_info(&spi->dev, "registered slic spi device %d at bus #%d, CS #%d",
		 dev_id, spi->master->bus_num, spi->chip_select);

	return 0;
}

static int mvebu_phone_spi_remove(struct spi_device *spi)
{
	return 0;
}

static const struct spi_device_id mvebu_phone_spi_ids[] = {
	{ "mv_slic", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, mvebu_phone_spi_ids);

static struct spi_driver mvebu_phone_spi_driver = {
	.driver = {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table = mvebu_phone_spi_ids,
	.probe	= mvebu_phone_spi_probe,
	.remove	= mvebu_phone_spi_remove,
};

module_spi_driver(mvebu_phone_spi_driver);

MODULE_DESCRIPTION("Marvell Telephony SPI Driver");
MODULE_AUTHOR("Marcin Wojtas <mw@semihalf.com>");
