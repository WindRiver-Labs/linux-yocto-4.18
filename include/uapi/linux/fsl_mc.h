/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Management Complex (MC) userspace public interface
 *
 * Copyright 2018 NXP
 *
 */
#ifndef _UAPI_FSL_MC_H_
#define _UAPI_FSL_MC_H_

#define MC_CMD_NUM_OF_PARAMS	7

#define RESTOOL_IOCTL_TYPE	'R'
#define RESTOOL_IOCTL_SEQ	0xE0

#define RESTOOL_SEND_MC_COMMAND \
	_IOWR(RESTOOL_IOCTL_TYPE, RESTOOL_IOCTL_SEQ, struct fsl_mc_command)

#endif /* _UAPI_FSL_MC_H_ */
