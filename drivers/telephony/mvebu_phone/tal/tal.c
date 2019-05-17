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

enum tal_status tal_init(struct tal_params *tal_params,
			 struct tal_mmp_ops *mmp_ops)
{
	if (!tal_params || !mmp_ops) {
		pr_err("%s: Error, bad parameters.\n", __func__);
		return TAL_STAT_BAD_PARAM;
	}

	if (!mmp_ops->tal_mmp_rx_callback || !mmp_ops->tal_mmp_tx_callback) {
		pr_err("%s: Error, MMP callbacks are missing.\n", __func__);
		return TAL_STAT_BAD_PARAM;
	}

	tal_mmp = mmp_ops;
	if (tal_if && tal_if->init)
		if (tal_if->init(tal_params) != 0)
			return TAL_STAT_INIT_ERROR;

	return TAL_STAT_OK;
}
EXPORT_SYMBOL(tal_init);

void tal_exit(void)
{
	if (tal_if && tal_if->exit)
		tal_if->exit();

	tal_mmp = NULL;
}
EXPORT_SYMBOL(tal_exit);

void tal_pcm_start(void)
{
	if (tal_if && tal_if->pcm_start)
		tal_if->pcm_start();
}
EXPORT_SYMBOL(tal_pcm_start);

void tal_pcm_stop(void)
{
	if (tal_if && tal_if->pcm_stop)
		tal_if->pcm_stop();
}
EXPORT_SYMBOL(tal_pcm_stop);

int tal_control(int cmd, void *data)
{
	if (tal_if && tal_if->control)
		return tal_if->control(cmd, data);

	return -EINVAL;
}
EXPORT_SYMBOL(tal_control);

enum tal_status tal_write(u8 *buffer, int size)
{
	if (tal_if && tal_if->write)
		if (tal_if->write(buffer, size) != 0)
			return TAL_STAT_BAD_PARAM;

	return TAL_STAT_OK;
}
EXPORT_SYMBOL(tal_write);

enum tal_status tal_stats_get(struct tal_stats *tal_stats)
{
	if (tal_stats && tal_if && tal_if->stats_get) {
		tal_if->stats_get(tal_stats);
		return TAL_STAT_OK;
	}

	return TAL_STAT_BAD_PARAM;
}
EXPORT_SYMBOL(tal_stats_get);

enum tal_status tal_set_if(struct tal_if *interface)
{
	if (interface && (!interface->init || !interface->exit ||
			  !interface->pcm_start || !interface->pcm_stop)) {
		pr_err("%s: Error, TAL callbacks are missing.\n", __func__);
		return TAL_STAT_BAD_PARAM;
	}

	tal_if = interface;

	return TAL_STAT_OK;
}
EXPORT_SYMBOL(tal_set_if);

enum tal_status tal_mmp_rx(u8 *buffer, int size)
{
	if (tal_mmp && tal_mmp->tal_mmp_rx_callback) {
		tal_mmp->tal_mmp_rx_callback(buffer, size);
		return TAL_STAT_OK;
	}

	return TAL_STAT_BAD_PARAM;
}
EXPORT_SYMBOL(tal_mmp_rx);

enum tal_status tal_mmp_tx(u8 *buffer, int size)
{
	if (tal_mmp && tal_mmp->tal_mmp_tx_callback) {
		tal_mmp->tal_mmp_tx_callback(buffer, size);
		return TAL_STAT_OK;
	}

	return TAL_STAT_BAD_PARAM;
}
EXPORT_SYMBOL(tal_mmp_tx);
