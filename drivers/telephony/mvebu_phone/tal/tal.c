// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

/* Marvell Telephony Adaptation Layer */
#include "tal.h"

static struct tal_if *tal_if;
static struct tal_mmp_ops *tal_mmp;

enum tal_status tal_init(struct device *dev, struct tal_params *tal_params,
			 struct tal_mmp_ops *mmp_ops)
{
	if (!tal_params || !mmp_ops) {
		pr_err("%s: Error, bad parameters.\n", __func__);
		return TAL_STATUS_BAD_PARAM;
	}

	if (!mmp_ops->tal_mmp_rx_callback || !mmp_ops->tal_mmp_tx_callback) {
		pr_err("%s: Error, MMP callbacks are missing.\n", __func__);
		return TAL_STATUS_BAD_PARAM;
	}

	tal_mmp = mmp_ops;
	if (tal_if && tal_if->init)
		if (tal_if->init(dev, tal_params) != 0)
			return TAL_STATUS_INIT_ERROR;

	return TAL_STATUS_OK;
}
EXPORT_SYMBOL(tal_init);

void tal_exit(struct device *dev)
{
	if (tal_if && tal_if->exit)
		tal_if->exit(dev);

	tal_mmp = NULL;
}
EXPORT_SYMBOL(tal_exit);

void tal_pcm_start(struct device *dev)
{
	if (tal_if && tal_if->pcm_start)
		tal_if->pcm_start(dev);
}
EXPORT_SYMBOL(tal_pcm_start);

void tal_pcm_stop(struct device *dev)
{
	if (tal_if && tal_if->pcm_stop)
		tal_if->pcm_stop(dev);
}
EXPORT_SYMBOL(tal_pcm_stop);

int tal_control(int cmd, void *data)
{
	if (tal_if && tal_if->control)
		return tal_if->control(cmd, data);

	return -EINVAL;
}
EXPORT_SYMBOL(tal_control);

enum tal_status tal_write(struct device *dev, u8 *buffer, int size)
{
	if (tal_if && tal_if->write)
		if (tal_if->write(dev, buffer, size) != 0)
			return TAL_STATUS_BAD_PARAM;

	return TAL_STATUS_OK;
}
EXPORT_SYMBOL(tal_write);

enum tal_status tal_stats_get(struct device *dev, struct tal_stats *tal_stats)
{
	if (tal_stats && tal_if && tal_if->stats_get) {
		tal_if->stats_get(dev, tal_stats);
		return TAL_STATUS_OK;
	}

	return TAL_STATUS_BAD_PARAM;
}
EXPORT_SYMBOL(tal_stats_get);

enum tal_status tal_set_if(struct tal_if *interface, struct device *dev,
			   u32 tdm_index, struct miscdevice *miscdev,
			   bool create_tal_dev)
{
	if (interface && (!interface->init || !interface->exit ||
			  !interface->pcm_start || !interface->pcm_stop)) {
		pr_err("%s: Error, TAL callbacks are missing.\n", __func__);
		return TAL_STATUS_BAD_PARAM;
	}

	tal_if = interface;

	if (create_tal_dev) {
		if (tal_if != NULL)
			return tal_dev_init(dev, tdm_index, miscdev);

		tal_dev_exit(miscdev);
	}

	return TAL_STATUS_OK;
}
EXPORT_SYMBOL(tal_set_if);

enum tal_status tal_mmp_rx(struct device *dev, u8 *buffer, int size)
{
	if (tal_mmp && tal_mmp->tal_mmp_rx_callback) {
		tal_mmp->tal_mmp_rx_callback(dev, buffer, size);
		return TAL_STATUS_OK;
	}

	return TAL_STATUS_BAD_PARAM;
}
EXPORT_SYMBOL(tal_mmp_rx);

enum tal_status tal_mmp_tx(struct device *dev, u8 *buffer, int size)
{
	if (tal_mmp && tal_mmp->tal_mmp_tx_callback) {
		tal_mmp->tal_mmp_tx_callback(dev, buffer, size);
		return TAL_STATUS_OK;
	}

	return TAL_STATUS_BAD_PARAM;
}
EXPORT_SYMBOL(tal_mmp_tx);

void tal_intr_enable(void)
{
	if (tal_if && tal_if->intr_enable)
		tal_if->intr_enable();
}
EXPORT_SYMBOL(tal_intr_enable);

void tal_intr_disable(void)
{
	if (tal_if && tal_if->intr_disable)
		tal_if->intr_disable();
}
EXPORT_SYMBOL(tal_intr_disable);
