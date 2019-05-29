// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

#ifndef _TAL_DEV_H_
#define _TAL_DEV_H_

#define TAL_DEV_IOCTL_MAGIC		't'

#define TAL_DEV_INIT			_IOWR(TAL_DEV_IOCTL_MAGIC, 1,\
					      struct tal_dev_params)
#define TAL_DEV_EXIT			_IO(TAL_DEV_IOCTL_MAGIC, 2)
#define TAL_DEV_PCM_START		_IO(TAL_DEV_IOCTL_MAGIC, 3)
#define TAL_DEV_PCM_STOP		_IO(TAL_DEV_IOCTL_MAGIC, 4)

struct tal_dev_params {
	unsigned char pcm_format;
	unsigned short total_lines;
	unsigned char enable_internal_loopback;
};

#define	TDM_DEV_TDM_TEST_MODE_ENABLE	_IO(TAL_DEV_IOCTL_MAGIC, 8)
#define	TDM_DEV_TDM_TEST_MODE_DISABLE	_IO(TAL_DEV_IOCTL_MAGIC, 9)

#endif /*_TAL_DEV_H_*/
