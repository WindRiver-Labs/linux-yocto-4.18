// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

/* Marvell Telephony Adaptation Layer */

#ifndef _TAL_H_
#define _TAL_H_

#include "mv_phone.h"

/* Defines */
#define TAL_MAX_PHONE_LINES	32

/* Enumerators */
enum tal_pcm_format {
	TAL_PCM_FORMAT_1BYTE = 1,
	TAL_PCM_FORMAT_2BYTES = 2,
	TAL_PCM_FORMAT_4BYTES = 4,
};

enum tal_status {
	TAL_STAT_OK = 0,
	TAL_STAT_BAD_PARAM,
	TAL_STAT_INIT_ERROR,
};

/* Structures */
struct tal_params {
	enum tal_pcm_format pcm_format;
	u16 pcm_slot[TAL_MAX_PHONE_LINES];
	u8 sampling_period;
	u16 total_lines;
	bool enable_internal_loopback;
};

struct tal_stats {
	int tdm_init;
	u32 rx_miss;
	u32 tx_miss;
	u32 rx_over;
	u32 tx_under;
	struct mv_phone_extended_stats tdm_ext_stats;
};

struct tal_mmp_ops {
	void (*tal_mmp_rx_callback)(u8 *rx_buff, int size);
	void (*tal_mmp_tx_callback)(u8 *tx_buff, int size);
};

struct tal_if {
	int (*init)(struct tal_params *tal_params);
	void (*exit)(void);
	void (*pcm_start)(void);
	void (*pcm_stop)(void);
	int (*control)(int cmd, void *data);
	int (*write)(u8 *buffer, int size);
	void (*stats_get)(struct tal_stats *tal_stats);
	void (*intr_enable)(void);
	void (*intr_disable)(void);
};

/* API */
enum tal_status tal_init(struct tal_params *tal_params,
			 struct tal_mmp_ops *mmp_ops);
enum tal_status tal_stats_get(struct tal_stats *tal_stats);
void tal_exit(void);
void tal_pcm_start(void);
void tal_pcm_stop(void);
int tal_control(int cmd, void *data);

enum tal_status tal_set_if(struct tal_if *interface);
enum tal_status tal_mmp_rx(u8 *buffer, int size);
enum tal_status tal_mmp_tx(u8 *buffer, int size);
enum tal_status tal_write(u8 *buffer, int size);
void tal_intr_enable(void);
void tal_intr_disable(void);

#endif /* _TAL_H */
